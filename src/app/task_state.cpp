#include "tasks.h"
#include "platform/pin_config.h"
#include "app_types.h"
#include "app_queue.h"
#include "service/Can/srv_can.h"
#include "algorithm/astar.h"
#include "platform/field_config.h"
#include "domain/motion/flip_detector.h"

#include <math.h>

namespace {

constexpr uint32_t STATE_PERIOD_MS = 100;
constexpr int32_t CAMERA_NAV_EXIT_DISTANCE_MM = 8000;
// 通常はモーター基板の15秒タイムアウト通知を使う。60秒はCAN通知喪失時だけの保険。
constexpr uint32_t ESCAPE_FALLBACK_TIMEOUT_MS = 60000;
constexpr uint32_t COORDINATE_FRESH_MS = 500;
constexpr float MANUAL_OBSTACLE_DISTANCE_MM = 1000.0f;

bool queueManualObstacle(uint32_t now_ms)
{
    Coordinate coordinate{};
    const uint16_t required_status =
        Domain::Localization::STATUS_POSITION_USABLE |
        Domain::Localization::STATUS_YAW_USABLE;
    if (xQueuePeek(mbx_coordinate, &coordinate, 0) != pdTRUE ||
        coordinate.timestamp_ms == 0U ||
        static_cast<uint32_t>(now_ms - coordinate.timestamp_ms) >
            COORDINATE_FRESH_MS ||
        (coordinate.localization_status_flags & required_status) !=
            required_status ||
        !isfinite(coordinate.heading_rad)) {
        return false;
    }

    // 現在セルを塞ぐとA*の開始点を失うため、機体が向いている1 m前方を登録する。
    MapUpdate update{};
    update.world_x_mm = static_cast<int32_t>(lroundf(
        static_cast<float>(coordinate.x_mm) +
        MANUAL_OBSTACLE_DISTANCE_MM * cosf(coordinate.heading_rad)));
    update.world_y_mm = static_cast<int32_t>(lroundf(
        static_cast<float>(coordinate.y_mm) +
        MANUAL_OBSTACLE_DISTANCE_MM * sinf(coordinate.heading_rad)));
    update.evidence_delta = 5;
    update.radius_cells = 1;
    update.maximum_value = AStar::CELL_BLOCKED;

    AStar::Config map_config{};
    AStar::GridPos cell{};
    if (!AStar::worldToGridChecked(
            static_cast<float>(update.world_x_mm),
            static_cast<float>(update.world_y_mm),
            cell,
            map_config)) {
        return false;
    }

    return xQueueSend(
        fifo_map_update,
        &update,
        pdMS_TO_TICKS(50)) == pdTRUE;
}

BootMode readBootModeAtStartup()
{
    // 両入力はLOWを有効とする。起動時の一度だけ読み、運用中は変更しない。
    pinMode(BOOT_MANUAL, INPUT_PULLUP);
    pinMode(BOOT_DEBUG, INPUT_PULLUP);
    vTaskDelay(pdMS_TO_TICKS(10));

    const bool manual_low = digitalRead(BOOT_MANUAL) == LOW;
    const bool debug_low = digitalRead(BOOT_DEBUG) == LOW;

    // SEQUENCE=LOWかつDEBUG_START=LOWは、通常SEQUENCEより優先してMANUALとする。
    if (debug_low) return BootMode::DEBUG;
    if (manual_low) return BootMode::MANUAL;
    return BootMode::SEQUENCE;
}

// CANへ転送する必要があるシステムコマンドだけを変換する。
bool toCanAction(SystemCmdType command, Can::Command::ActionType& action)
{
    switch (command) {
        case SystemCmdType::Reset:            action = Can::Command::Reset; return true;
        case SystemCmdType::StartSequence:    action = Can::Command::SequenceStart; return true;
        case SystemCmdType::NotifyGoal:       action = Can::Command::NotifyGoal; return true;
        case SystemCmdType::NotifySeparation: action = Can::Command::NotifySeparation; return true;
        case SystemCmdType::NotifyStuck:      action = Can::Command::NotifyStuck; return true;
        case SystemCmdType::NotifyFlipped:    action = Can::Command::NotifyFlipped; return true;
        case SystemCmdType::ConfirmUpright:   action = Can::Command::ConfirmUpright; return true;
        case SystemCmdType::ServoUnlock:      action = Can::Command::ServoUnlock; return true;
        case SystemCmdType::ServoLock:        action = Can::Command::ServoLock; return true;
        default: return false;
    }
}

// 状態確認済みのActionだけをCAN送信FIFOへ入れる。
// taskCanは10ms周期で取り出すため、短時間待って重要コマンドの欠落を避ける。
bool queueCanAction(SystemCmdType command)
{
    Can::Command::ActionType action{};
    if (toCanAction(command, action)) {
        return xQueueSend(fifo_can_char_cmd, &action, pdMS_TO_TICKS(50)) == pdTRUE;
    }
    return false;
}

} // namespace

void taskState(void *pvParameters)
{
    (void)pvParameters;
    const BootMode boot_mode = readBootModeAtStartup();

    SystemData system_data{};
    system_data.state = SystemState::STATE_PRELAUNCH;
    system_data.boot_mode = boot_mode;
    system_data.gps_localization_enabled = true;
    bool force_camera_nav = false;
    SystemState suspended_navigation_state = SystemState::STATE_GPS_NAV;
    uint32_t escape_started_ms = 0;
    bool upright_stop_sent = false;

    xQueueOverwrite(mbx_system_data, &system_data);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(STATE_PERIOD_MS);

    while (true) {
        xTaskDelayUntil(&last_wake, period);
        const uint32_t now_ms = millis();

        // 反転復帰中だけ姿勢を監視し、正常姿勢が500ms連続したら停止指令'U'を送る。
        if (system_data.state == SystemState::STATE_UPRIGHT_RECOVERY &&
            !upright_stop_sent) {
            Domain::Motion::FlipResult flip{};
            const bool upright =
                xQueuePeek(mbx_flip_status, &flip, 0) == pdTRUE &&
                flip.timestamp_us != 0U &&
                static_cast<uint32_t>(
                    now_ms - static_cast<uint32_t>(
                        flip.timestamp_us / 1000ULL)) <=
                    COORDINATE_FRESH_MS &&
                flip.state == Domain::Motion::FlipState::Upright;
            if (upright && queueCanAction(SystemCmdType::ConfirmUpright)) {
                upright_stop_sent = true;
            }
        }

        // モーター基板の完了/失敗通知を失った場合だけ、60秒後にナビへ戻す。
        if ((system_data.state == SystemState::STATE_ESCAPE ||
             system_data.state == SystemState::STATE_UPRIGHT_RECOVERY) &&
            escape_started_ms != 0 &&
            static_cast<uint32_t>(now_ms - escape_started_ms) >=
                ESCAPE_FALLBACK_TIMEOUT_MS) {
            // 強制カメラモードはESCAPE中も保持し、終了後にCAMERA_NAVへ復帰する。
            system_data.state = force_camera_nav
                ? SystemState::STATE_CAMERA_NAV
                : SystemState::STATE_GPS_NAV;
            escape_started_ms = 0;
            upright_stop_sent = false;
        }

        // CAMERA_NAVの自動復帰は強制カメラモード中だけ抑止する。
        if (system_data.state == SystemState::STATE_CAMERA_NAV && !force_camera_nav) {
            Coordinate coordinate{};
            if (xQueuePeek(mbx_coordinate, &coordinate, 0) == pdTRUE &&
                coordinate.is_first_gps_valid) {
                const bool position_updated =
                    (coordinate.source_flags & CORD_SRC_GPS) != 0 ||
                    (coordinate.source_flags & CORD_SRC_ENCODER) != 0;

                // GPSまたはエンコーダで更新された融合座標からゴール距離を求める。
                if (position_updated) {
                    const float goal_dx_mm =
                        static_cast<float>(
                            coordinate.x_mm - FieldConfig::GOAL_X_MM);
                    const float goal_dy_mm =
                        static_cast<float>(
                            coordinate.y_mm - FieldConfig::GOAL_Y_MM);
                    const float distance_to_goal_mm =
                        hypotf(goal_dx_mm, goal_dy_mm);

                    if (distance_to_goal_mm >= CAMERA_NAV_EXIT_DISTANCE_MM) {
                        system_data.state = SystemState::STATE_GPS_NAV;
                    }
                }
            }
        }

        // 1周期中に届いた状態遷移要求を順番に処理する。
        SystemCmdType command = SystemCmdType::None;
        while (xQueueReceive(fifo_system_cmd, &command, 0) == pdTRUE) {
            switch (command) {
                case SystemCmdType::Reset:
                    queueCanAction(command);
                    force_camera_nav = false;
                    system_data.gps_localization_enabled = true;
                    system_data.navigation_reset_count = 0;
                    escape_started_ms = 0;
                    upright_stop_sent = false;
                    system_data.state = SystemState::STATE_PRELAUNCH;
                    break;

                case SystemCmdType::StartSequence:
                    if (system_data.state == SystemState::STATE_PRELAUNCH) {
                        queueCanAction(command);
                        system_data.state = SystemState::STATE_AWAIT_ASCENT;
                    }
                    break;

                case SystemCmdType::AscentDetected:
                    if (system_data.state == SystemState::STATE_AWAIT_ASCENT) {
                        system_data.state = SystemState::STATE_ASCENT_TO_LANDING;
                    }
                    break;

                case SystemCmdType::LandingDetected:
                    if (system_data.state == SystemState::STATE_ASCENT_TO_LANDING) {
                        system_data.state = SystemState::STATE_SEPARATION;
                        queueCanAction(SystemCmdType::NotifySeparation);
                    }
                    break;

                case SystemCmdType::SeparationFinished:
                    if (system_data.state == SystemState::STATE_SEPARATION) {
                        system_data.state = SystemState::STATE_GPS_NAV;
                    }
                    break;

                case SystemCmdType::StartCameraNav:
                    if (system_data.state == SystemState::STATE_GPS_NAV) {
                        system_data.state = SystemState::STATE_CAMERA_NAV;
                    }
                    break;

                case SystemCmdType::StartGpsNav:
                    if (system_data.state == SystemState::STATE_CAMERA_NAV &&
                        !force_camera_nav) {
                        system_data.state = SystemState::STATE_GPS_NAV;
                    }
                    break;

                case SystemCmdType::NotifyStuck:
                    if (system_data.state == SystemState::STATE_GPS_NAV ||
                        system_data.state == SystemState::STATE_CAMERA_NAV ||
                        system_data.state ==
                            SystemState::STATE_STUCK_SUSPEND) {
                        queueCanAction(command);
                        system_data.state = SystemState::STATE_ESCAPE;
                        escape_started_ms = now_ms;
                        upright_stop_sent = false;
                    }
                    break;

                case SystemCmdType::RequestStuckSuspend:
                    if (system_data.state == SystemState::STATE_GPS_NAV ||
                        system_data.state == SystemState::STATE_CAMERA_NAV) {
                        suspended_navigation_state = system_data.state;
                        system_data.state =
                            SystemState::STATE_STUCK_SUSPEND;
                    }
                    break;

                case SystemCmdType::StuckVerificationRejected:
                    if (system_data.state ==
                        SystemState::STATE_STUCK_SUSPEND) {
                        system_data.state = suspended_navigation_state;
                    }
                    break;

                case SystemCmdType::NotifyFlipped:
                    if (system_data.state == SystemState::STATE_GPS_NAV ||
                        system_data.state == SystemState::STATE_CAMERA_NAV ||
                        system_data.state ==
                            SystemState::STATE_STUCK_SUSPEND) {
                        queueCanAction(command);
                        system_data.state = SystemState::STATE_UPRIGHT_RECOVERY;
                        escape_started_ms = now_ms;
                        upright_stop_sent = false;
                    }
                    break;

                case SystemCmdType::StuckResolved:
                    if (system_data.state == SystemState::STATE_ESCAPE ||
                        system_data.state == SystemState::STATE_UPRIGHT_RECOVERY) {
                        // 通常はA*再計画のためGPS_NAVへ戻す。強制カメラ中だけは復帰先を維持する。
                        system_data.state = force_camera_nav
                            ? SystemState::STATE_CAMERA_NAV
                            : SystemState::STATE_GPS_NAV;
                        escape_started_ms = 0;
                        upright_stop_sent = false;
                    }
                    break;

                case SystemCmdType::UprightRecoveryFailed:
                    // 0x04失敗通知は反転復帰中だけ状態遷移へ使用する。
                    if (system_data.state == SystemState::STATE_UPRIGHT_RECOVERY) {
                        system_data.state = force_camera_nav
                            ? SystemState::STATE_CAMERA_NAV
                            : SystemState::STATE_GPS_NAV;
                        escape_started_ms = 0;
                        upright_stop_sent = false;
                    }
                    break;

                case SystemCmdType::NotifyGoal:
                    if (system_data.state == SystemState::STATE_CAMERA_NAV) {
                        queueCanAction(command);
                        system_data.state = SystemState::STATE_GOAL;
                    }
                    break;

                case SystemCmdType::ForceGpsNav:
                    if (system_data.boot_mode == BootMode::MANUAL) {
                        force_camera_nav = false;
                        system_data.state = SystemState::STATE_GPS_NAV;
                    }
                    break;

                case SystemCmdType::ForceCameraNav:
                    if (system_data.boot_mode == BootMode::MANUAL) {
                        force_camera_nav = true;
                        system_data.state = SystemState::STATE_CAMERA_NAV;
                    }
                    break;

                case SystemCmdType::ForceEscape:
                    if (system_data.boot_mode == BootMode::MANUAL) {
                        system_data.state = SystemState::STATE_ESCAPE;
                        escape_started_ms = now_ms;
                        upright_stop_sent = false;

                        // 強制ESCAPEでもモーター基板へ脱出開始を通知する。
                        queueCanAction(SystemCmdType::NotifyStuck);
                    }
                    break;

                case SystemCmdType::NavigationRecoveryReset:
                    if (system_data.state == SystemState::STATE_GPS_NAV &&
                        system_data.navigation_reset_count < UINT16_MAX) {
                        // Mission state is intentionally unchanged. taskLocalization
                        // observes this generation and rebuilds Localization from
                        // GPS samples newer than this request.
                        ++system_data.navigation_reset_count;
                    }
                    break;

                case SystemCmdType::DisableGpsLocalization:
                    if (system_data.boot_mode == BootMode::MANUAL) {
                        system_data.gps_localization_enabled = false;
                    }
                    break;

                case SystemCmdType::EnableGpsLocalization:
                    if (system_data.boot_mode == BootMode::MANUAL) {
                        system_data.gps_localization_enabled = true;
                    }
                    break;

                case SystemCmdType::MarkObstacle:
                    if (system_data.boot_mode == BootMode::MANUAL) {
                        queueManualObstacle(now_ms);
                    }
                    break;

                case SystemCmdType::NotifySeparation:
                    if (system_data.state == SystemState::STATE_SEPARATION) {
                        queueCanAction(command);
                    }
                    break;

                case SystemCmdType::ServoUnlock:
                case SystemCmdType::ServoLock:
                    if (system_data.state == SystemState::STATE_PRELAUNCH) {
                        queueCanAction(command);
                    }
                    break;

                default:
                    break;
            }
        }

        xQueueOverwrite(mbx_system_data, &system_data);
    }
}
