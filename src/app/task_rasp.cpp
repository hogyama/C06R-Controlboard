#include "tasks.h"
#include "platform/board_config.h"
#include "platform/pin_config.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "service/Rasp/srv_rasp.h"

namespace {

// DEBUGではUARTピンへ触れず、電源と制御信号だけを非アクティブにする。
void holdRaspPowerOff()
{
    pinMode(RASP_EN, OUTPUT);
    pinMode(RASP_HANDSHAKE, OUTPUT);
    pinMode(RASP_IMAGE_MODE, OUTPUT);
    digitalWrite(RASP_EN, LOW);
    digitalWrite(RASP_HANDSHAKE, LOW);
    digitalWrite(RASP_IMAGE_MODE, LOW);
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
        RASP_UART_TX,
        RASP_HANDSHAKE,
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
    bool image_mode_applied = false;
    bool ready_seen = false;
    bool power_cycle_failed = false;
    bool power_off_pending = false;
    uint8_t startup_retry_count = 0;
    uint8_t ready_drop_retry_count = 0;
    uint32_t power_started_ms = 0;
    uint32_t power_off_ms = millis() - 3000U;
    uint32_t ready_low_started_ms = 0;
    uint32_t power_off_requested_ms = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        xTaskDelayUntil(&last_wake, period);
        rasp.update();
        if (!initialized) {
            continue;
        }

        xQueuePeek(mbx_system_data, &system_data, 0);
        const bool should_power_on =
            system_data.boot_mode != BootMode::DEBUG &&
            (system_data.state == SystemState::STATE_CAMERA_NAV ||
             system_data.state == SystemState::STATE_ESCAPE ||
             system_data.state == SystemState::STATE_UPRIGHT_RECOVERY ||
             system_data.state == SystemState::STATE_GPS_NAV);
        const bool should_receive_camera =
            (system_data.boot_mode == BootMode::SEQUENCE ||
             system_data.boot_mode == BootMode::MANUAL) &&
            system_data.state == SystemState::STATE_CAMERA_NAV;
        // MANUAL is tuning/image mode; SEQUENCE is flight mode.
        const bool requested_image_mode =
            system_data.boot_mode == BootMode::MANUAL;

        if (!should_receive_camera) {
            have_published_frame = false;
            last_published_msg_number = 0;
            xQueueReset(mbx_camera_data);
        }

        const SrvRasp::RaspStatus status = rasp.getStatus();
        const uint32_t now = millis();

        // Leaving every Pi-using state clears a latched two-attempt failure.
        if (!should_power_on && !status.power_enabled) {
            power_cycle_failed = false;
            startup_retry_count = 0;
            ready_drop_retry_count = 0;
            digitalWrite(RASP_IMAGE_MODE, LOW);
            continue;
        }

        // Make request/image lines inactive, then cut power after 200 ms.
        if (!should_power_on && status.power_enabled) {
            if (!power_off_pending) {
                digitalWrite(RASP_HANDSHAKE, LOW);
                digitalWrite(RASP_IMAGE_MODE, LOW);
                power_off_requested_ms = now;
                power_off_pending = true;
            } else if (now - power_off_requested_ms >= 200U) {
                rasp.powerOff();
                power_off_ms = now;
                power_off_pending = false;
                image_mode_applied = false;
                ready_seen = false;
                power_cycle_failed = false;
                startup_retry_count = 0;
                ready_drop_retry_count = 0;
            }
            continue;
        }
        power_off_pending = false;

        // Keep RASP_EN low for at least 3 s between power cycles.
        if (should_power_on && !status.power_enabled && !power_cycle_failed) {
            if (now - power_off_ms >= 3000U && rasp.powerOn()) {
                power_started_ms = now;
                image_mode_applied = false;
                ready_seen = false;
                ready_low_started_ms = 0;
            }
            continue;
        }

        // Pi reads IMAGE_MODE during startup. Apply it 200 ms after RASP_EN.
        if (status.power_enabled && !image_mode_applied &&
            now - power_started_ms >= 200U) {
            digitalWrite(RASP_IMAGE_MODE, requested_image_mode ? HIGH : LOW);
            image_mode_applied = true;
        }

        if (status.camera_ready) {
            ready_seen = true;
            ready_low_started_ms = 0;
        } else if (ready_seen && ready_low_started_ms == 0) {
            ready_low_started_ms = now;
        }

        const bool startup_timed_out =
            status.power_enabled && !ready_seen &&
            now - power_started_ms >= 90000U;
        const bool ready_drop_timed_out =
            ready_seen && !status.camera_ready &&
            ready_low_started_ms != 0 && now - ready_low_started_ms >= 10000U;
        if (startup_timed_out || ready_drop_timed_out) {
            const bool can_retry = startup_timed_out
                ? startup_retry_count++ < 1U
                : ready_drop_retry_count++ < 1U;
            digitalWrite(RASP_IMAGE_MODE, LOW);
            rasp.powerOff();
            power_off_ms = now;
            image_mode_applied = false;
            ready_seen = false;
            ready_low_started_ms = 0;
            power_cycle_failed = !can_retry;
            continue;
        }

        if (should_receive_camera &&
            rasp.getState() == SrvRasp::RaspState::Ready &&
            !status.camera_reception_started) {
            rasp.startCameraReception();
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
