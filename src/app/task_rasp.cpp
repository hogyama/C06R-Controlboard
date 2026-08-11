#include "tasks.h"
#include "platform/board_config.h"
#include "platform/pin_config.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "service/Rasp/srv_rasp.h"
#include <cstring>

namespace {

Rasp::CameraParameters makeCameraParameters(bool image_mode)
{
    Rasp::CameraParameters parameters{};
    parameters.version = 2;
    parameters.image_mode = image_mode ? 1U : 0U;
    parameters.min_target_area_px = CAMERA_MIN_TARGET_AREA_PX;
    parameters.confidence_full_area_px = CAMERA_CONFIDENCE_FULL_AREA_PX;
    parameters.horizontal_fov_deg_x100 = CAMERA_HORIZONTAL_FOV_DEG_X100;
    memcpy(parameters.hsv_low1, CAMERA_HSV_LOW1, sizeof(parameters.hsv_low1));
    memcpy(parameters.hsv_high1, CAMERA_HSV_HIGH1, sizeof(parameters.hsv_high1));
    memcpy(parameters.hsv_low2, CAMERA_HSV_LOW2, sizeof(parameters.hsv_low2));
    memcpy(parameters.hsv_high2, CAMERA_HSV_HIGH2, sizeof(parameters.hsv_high2));
    parameters.blur_kernel_size = CAMERA_BLUR_KERNEL_SIZE;
    parameters.morph_kernel_size = CAMERA_MORPH_KERNEL_SIZE;
    parameters.transform_flags =
        (CAMERA_ROTATE_180 ? Rasp::CAMERA_TRANSFORM_ROTATE_180 : 0U) |
        (CAMERA_MIRROR_HORIZONTAL
            ? Rasp::CAMERA_TRANSFORM_MIRROR_HORIZONTAL
            : 0U);
    return parameters;
}

// DEBUGではUARTピンへ触れず、電源と制御信号だけを非アクティブにする。
void holdRaspPowerOff()
{
    pinMode(RASP_EN, OUTPUT);
    digitalWrite(RASP_EN, LOW);
    pinMode(RASP_UART_RX, INPUT);
    pinMode(RASP_UART_TX, INPUT);
    pinMode(RASP_HEARTBEAT, INPUT_PULLDOWN);
    pinMode(RASP_CAMERA_READY, INPUT_PULLDOWN);
}

} // namespace

void taskRasp(void *pvParameters)
{
    (void)pvParameters;
    const TickType_t period = pdMS_TO_TICKS(20);
    holdRaspPowerOff();

    SystemData startup_status{};
    xQueuePeek(mbx_system_data, &startup_status, portMAX_DELAY);
    if (startup_status.boot_mode == BootMode::DEBUG) {
        xQueueReset(mbx_camera_data);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    const bool initialized = rasp.init(
        RASP_UART_RX,
        RASP_UART_TX,
        RASP_HEARTBEAT,
        RASP_CAMERA_READY,
        RASP_EN,
        RASP_UART_NUM,
        RASP_UART_BAUD
    );
    if (!initialized) {
        holdRaspPowerOff();
    }

    SystemData system_data{};
    uint32_t last_published_msg_number = 0;
    bool have_published_frame = false;
    bool ready_seen = false;
    bool power_cycle_failed = false;
    uint8_t startup_retry_count = 0;
    uint8_t ready_drop_retry_count = 0;
    uint32_t power_started_ms = 0;
    uint32_t power_off_ms = millis() - 3000U;
    uint32_t ready_low_started_ms = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        xTaskDelayUntil(&last_wake, period);
        rasp.update();
        if (!initialized) {
            continue;
        }

        xQueuePeek(mbx_system_data, &system_data, 0);
        StuckDiagnostics stuck_diagnostics{};
        const uint32_t now = millis();
        const bool wheel_slip_verification =
            xQueuePeek(
                mbx_stuck_diagnostics,
                &stuck_diagnostics,
                0) == pdTRUE &&
            static_cast<uint32_t>(
                now - stuck_diagnostics.timestamp_ms) <= 500U &&
            stuck_diagnostics.trigger_reason == StuckReason::WheelSlip;
        const bool should_power_on =
            system_data.boot_mode != BootMode::DEBUG &&
            (system_data.state == SystemState::STATE_CAMERA_NAV ||
             system_data.state == SystemState::STATE_STUCK_SUSPEND ||
             system_data.state == SystemState::STATE_ESCAPE ||
             system_data.state == SystemState::STATE_UPRIGHT_RECOVERY ||
             system_data.state == SystemState::STATE_GPS_NAV);
        const bool should_receive_camera =
            (system_data.boot_mode == BootMode::SEQUENCE ||
             system_data.boot_mode == BootMode::MANUAL) &&
            (system_data.state == SystemState::STATE_CAMERA_NAV ||
             (system_data.state == SystemState::STATE_STUCK_SUSPEND &&
              wheel_slip_verification));
        // MANUAL is tuning/image mode; SEQUENCE is flight mode.
        const bool requested_image_mode =
            system_data.boot_mode == BootMode::MANUAL;

        if (!should_receive_camera) {
            have_published_frame = false;
            last_published_msg_number = 0;
            xQueueReset(mbx_camera_data);
        }

        const SrvRasp::RaspStatus status = rasp.getStatus();

        if (!should_receive_camera &&
            (status.camera_reception_started || status.waiting_response)) {
            rasp.stopCameraReception();
        }
        // Leaving every Pi-using state clears a latched two-attempt failure.
        if (!should_power_on && !status.power_enabled) {
            power_cycle_failed = false;
            startup_retry_count = 0;
            ready_drop_retry_count = 0;
            continue;
        }

        if (!should_power_on && status.power_enabled) {
            rasp.powerOff();
            power_off_ms = now;
            ready_seen = false;
            power_cycle_failed = false;
            startup_retry_count = 0;
            ready_drop_retry_count = 0;
            continue;
        }

        // Keep RASP_EN low for at least 3 s between power cycles.
        if (should_power_on && !status.power_enabled && !power_cycle_failed) {
            if (now - power_off_ms >= 3000U && rasp.powerOn()) {
                power_started_ms = now;
                ready_seen = false;
                ready_low_started_ms = 0;
            }
            continue;
        }

        if (status.camera_ready && status.heartbeat_alive) {
            ready_seen = true;
            ready_low_started_ms = 0;
        } else if (ready_seen && ready_low_started_ms == 0) {
            ready_low_started_ms = now;
        }

        const bool startup_timed_out =
            should_receive_camera && status.power_enabled && !ready_seen &&
            now - power_started_ms >= 90000U;
        const bool ready_drop_timed_out =
            should_receive_camera && ready_seen &&
            (!status.camera_ready || !status.heartbeat_alive) &&
            ready_low_started_ms != 0 && now - ready_low_started_ms >= 10000U;
        if (startup_timed_out || ready_drop_timed_out) {
            const bool can_retry = startup_timed_out
                ? startup_retry_count++ < 1U
                : ready_drop_retry_count++ < 1U;
            rasp.powerOff();
            power_off_ms = now;
            ready_seen = false;
            ready_low_started_ms = 0;
            power_cycle_failed = !can_retry;
            continue;
        }

        // READY requires a previously received frame, so it cannot gate the
        // first request. Start once the Pi has raised CAMERA_READY and the
        // startup image-mode pin has been applied.
        if (should_receive_camera &&
            status.power_enabled &&
            status.heartbeat_alive &&
            !status.camera_reception_started) {
            const Rasp::CameraParameters parameters =
                makeCameraParameters(requested_image_mode);
            rasp.startCameraReception(parameters);
        }

        if (should_receive_camera) {
            Rasp::Frame frame{};
            uint32_t requested_ms = 0;
            if (rasp.getLatestFrame(frame, requested_ms) &&
                (!have_published_frame ||
                 frame.msg_number != last_published_msg_number)) {
                Rasp::CameraData camera_data{};
                camera_data.frame = frame;
                camera_data.requested_ms = requested_ms;
                camera_data.received_ms = now;
                xQueueOverwrite(mbx_camera_data, &camera_data);
                last_published_msg_number = frame.msg_number;
                have_published_frame = true;
            }
        }
    }
}
