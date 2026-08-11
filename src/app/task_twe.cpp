#include "tasks.h"
#include "platform/board_config.h"
#include "platform/pin_config.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "service/Twe/srv_twe.h"

namespace {

void holdTwePowerOff()
{
    // TWE_ENはLOW有効。DEBUGではUARTピンへ触れず電源だけを切る。
    pinMode(TWE_EN, OUTPUT);
    digitalWrite(TWE_EN, HIGH);
}

} // namespace

SystemCmdType convertTweCommand(Twe::Data command)
{
    Twe::CommandType cmd_type = command.cmd.type;
    switch (cmd_type)
    {
        case Twe::CommandType::None:
            return SystemCmdType::None;
        case Twe::CommandType::ServoLock:
            return SystemCmdType::ServoLock;
        case Twe::CommandType::ServoUnlock:
            return SystemCmdType::ServoUnlock;
        case Twe::CommandType::StartSequence:
            return SystemCmdType::StartSequence;
        case Twe::CommandType::Reset:
            return SystemCmdType::Reset;
        case Twe::CommandType::InjectAscent:
            return SystemCmdType::AscentDetected;
        case Twe::CommandType::InjectLanding:
            return SystemCmdType::LandingDetected;
        case Twe::CommandType::InjectSeparationFinished:
            return SystemCmdType::SeparationFinished;
        case Twe::CommandType::StartGpsNav:
            return SystemCmdType::ForceGpsNav;
        case Twe::CommandType::StartCameraNav:
            return SystemCmdType::ForceCameraNav;
        case Twe::CommandType::StartEscape:
            return SystemCmdType::ForceEscape;
        case Twe::CommandType::NotifyStuck:
            return SystemCmdType::NotifyStuck;
        case Twe::CommandType::NotifyGoal:
            return SystemCmdType::NotifyGoal;
        case Twe::CommandType::NotifySeparation:
            return SystemCmdType::NotifySeparation;
        case Twe::CommandType::DisableGpsLocalization:
            return SystemCmdType::DisableGpsLocalization;
        case Twe::CommandType::EnableGpsLocalization:
            return SystemCmdType::EnableGpsLocalization;
        case Twe::CommandType::MarkObstacle:
            return SystemCmdType::MarkObstacle;
        default:
            return SystemCmdType::None;
    }
}

void taskTwe(void *pvParameters) {
    constexpr TickType_t period = pdMS_TO_TICKS(100);
    constexpr uint32_t TELEMETRY_PERIOD_MS = 1000;

    // BootMode確定前はTWELITE用UARTを開始しない。
    holdTwePowerOff();
    SystemData startup_status{};
    xQueuePeek(mbx_system_data, &startup_status, portMAX_DELAY);

    // BootModeは起動時固定。DEBUGではtwe.init()を一度も呼ばない。
    if (startup_status.boot_mode == BootMode::DEBUG) {
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    const bool twe_initialized =
        twe.init(TWE_TX, TWE_RX, TWE_EN, TWE_UART_NUM, TWE_UART_BAUD);
    if (!twe_initialized) {
        holdTwePowerOff();
    }

    bool is_twe_power_on = false;
    uint32_t last_telemetry_ms = 0;
    TickType_t last_wake = xTaskGetTickCount();
    while (true){
        xTaskDelayUntil(&last_wake, period);
        if (!twe_initialized) {
            continue;
        }
        SystemData status = {};
        if(xQueuePeek(mbx_system_data, &status, 0) != pdTRUE) continue;
        const bool should_power_on = status.boot_mode == BootMode::MANUAL ||
            (status.boot_mode == BootMode::SEQUENCE &&
             status.state != SystemState::STATE_GOAL &&
             status.state != SystemState::STATE_AWAIT_ASCENT &&
             status.state != SystemState::STATE_ASCENT_TO_LANDING);
        if (should_power_on != is_twe_power_on) {
            if (should_power_on) is_twe_power_on = twe.powerOn();
            else {
                twe.powerOff();
                is_twe_power_on = false;
            }
        }
        if(is_twe_power_on){
            twe.poll();
            Twe::Data data{};
            Twe::TelemetryFrame frame{};
            // 受信
            Twe::MessageType msg_type = twe.readMsg(&data);
            if(msg_type == Twe::MessageType::Command){
                // CAN専用reset/startはtaskStateを通さないため、現在stateを変えない。
                if (data.cmd.type == Twe::CommandType::CanReset ||
                    data.cmd.type == Twe::CommandType::CanStart) {
                    const Can::Command::ActionType action =
                        data.cmd.type == Twe::CommandType::CanReset
                            ? Can::Command::Reset
                            : Can::Command::SequenceStart;
                    xQueueSend(fifo_can_char_cmd, &action, portMAX_DELAY);
                } else {
                    SystemCmdType cmd_type = convertTweCommand(data);
                    if(cmd_type != SystemCmdType::None){
                        xQueueSend(fifo_system_cmd, &cmd_type, portMAX_DELAY);
                    }
                }
            } else if (msg_type == Twe::MessageType::Jog &&
                       status.boot_mode == BootMode::MANUAL) {
                MotionCommandRequest request{};
                request.source = MotionCommandSource::Manual;
                request.velocity_mm_s = data.jog.velocity_mm_s;
                request.omega_rad_s_x100 =
                    data.jog.omega_rad_s_x100;
                request.duration_ms = data.jog.duration_ms;
                request.timestamp_ms = millis();
                xQueueSend(
                    fifo_motion_command_request,
                    &request,
                    pdMS_TO_TICKS(20));
            }
            // 送信
            if(xQueuePeek(mbx_twe_telemetry, &frame, 0) == pdTRUE){
                const uint32_t now_ms = millis();
                if(static_cast<uint32_t>(now_ms - last_telemetry_ms) >=
                    TELEMETRY_PERIOD_MS){
                    twe.sendTelemetry(frame);
                    last_telemetry_ms = now_ms;
                }
            }
        }
    }
}
