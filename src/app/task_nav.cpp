#include "tasks.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "algorithm/astar.h"
#include "algorithm/pure_pursuit.h"
#include "domain/fusion/fusion_types.h"
#include "platform/field_config.h"

#include <math.h>
#include <string.h>

namespace {

// ジャイロ・エンコーダの100Hz更新に合わせ、カメラ内側制御も10ms周期で行う。
constexpr uint32_t NAV_PERIOD_MS = 10;
constexpr uint32_t JOG_DURATION_MS = 300;
constexpr uint32_t COORDINATE_TIMEOUT_MS = 500;
constexpr uint32_t REPLAN_RETRY_MS = 1000;
constexpr uint32_t INITIAL_MAP_WAIT_MS = 5000;
constexpr uint32_t NAV_MAP_REQUEST_ID = 0x4E41564DU; // ASCII: NAVM

// 100:1 Pololu 20Dを3S LiPoで駆動するため、低速域にも始動トルクを確保する。
constexpr float GPS_BASE_SPEED_MM_S = 600.0f;
constexpr float GPS_MIN_SPEED_MM_S = 400.0f;
constexpr float GPS_MAX_SPEED_MM_S = 750.0f;

// 急旋回時の過大な角速度を抑え、経路追従の行き過ぎを防ぐ。
constexpr float GPS_MAX_TURN_RATE_RAD_S = 3.0f;

// フィールド外ではA*の開始セルを作れないため、最寄りの内側点へ直接戻る。
constexpr float FIELD_RECOVERY_INSET_MM = 1000.0f;
constexpr float FIELD_RECOVERY_ALIGN_RAD = 25.0f * (M_PI / 180.0f);
constexpr float FIELD_RECOVERY_TURN_RATE_RAD_S = 3.0f;
constexpr float FIELD_RECOVERY_HEADING_KP = 2.0f;

// 実測トレッド幅。Pure Pursuitの左右車輪速度換算にも同じ値を使う。
constexpr float ROBOT_TRACK_WIDTH_MM = 180.0f;

constexpr int32_t CAMERA_NAV_ENTER_DISTANCE_MM = 5000;
constexpr uint32_t CAMERA_COMM_TIMEOUT_MS = 1200;
constexpr uint32_t CAMERA_CONTROL_FRAME_TIMEOUT_MS = 400;
constexpr uint32_t CAMERA_GYRO_TIMEOUT_MS = 100;
constexpr uint32_t CAMERA_GYRO_MAX_DT_MS = 100;

// 検出直後はカメラ値が揺れやすいため、連続検出を確認するまで停止する。

constexpr uint8_t CAMERA_TARGET_CONFIRM_FRAMES = 2;

// この機体は微小旋回が難しいため、±15度は正面として静止画像2枚で確認する。
constexpr int16_t CAMERA_FORWARD_DEAD_BAND_DEG10 = 150;
// 逆補正を使った後だけは、±30度までを粗い正面として低速前進を許可する。
constexpr int16_t CAMERA_COARSE_FORWARD_DEAD_BAND_DEG10 = 300;
constexpr int16_t CAMERA_GOAL_MAX_ANGLE_DEG10 = 100;
constexpr uint8_t CAMERA_GOAL_MIN_CONFIDENCE = 70;
constexpr uint16_t CAMERA_GOAL_OCCUPANCY_PERMILLE = 700;
constexpr uint8_t CAMERA_GOAL_REQUIRED_FRAMES = 5;
constexpr uint8_t CAMERA_ANGLE_HISTORY_SIZE = 2;

constexpr float CAMERA_FAR_FORWARD_SPEED_MM_S = 800.0f;
constexpr float CAMERA_NEAR_FORWARD_SPEED_MM_S = 450.0f;
constexpr uint16_t CAMERA_NEAR_OCCUPANCY_PERMILLE = 250;

// taskNavが停止判断できなくなった場合にも短時間でJogを失効させる。
constexpr uint32_t CAMERA_CONTROL_COMMAND_MS = 80;

// 静止摩擦を確実に越えるため、通常補正・探索とも旋回角速度を固定する。
constexpr float CAMERA_TURN_OMEGA_RAD_S = 3.3f;
constexpr float CAMERA_TURN_REACHED_RAD = 5.0f * (M_PI / 180.0f);
constexpr float CAMERA_SEARCH_REACHED_RAD = 2.0f * (M_PI / 180.0f);
constexpr float CAMERA_MAX_GYRO_RATE_RAD_S = 20.0f;
constexpr float CAMERA_COAST_PREDICTION_S = 0.06f;
constexpr float CAMERA_STILL_GYRO_RAD_S = 0.30f;
constexpr float CAMERA_DELAY_GYRO_DEADBAND_RAD_S = 0.05f;
constexpr float CAMERA_STILL_ENCODER_MM_S = 100.0f;
constexpr uint32_t CAMERA_STILL_CONFIRM_MS = 200;
constexpr uint32_t CAMERA_ENCODER_TIMEOUT_MS = 100;

// 明示的な未検出ごとに45度旋回する
// 探索では逆方向の補正旋回を行わない。
constexpr float CAMERA_SEARCH_STEP_RAD = 45.0f * (M_PI / 180.0f);
constexpr float CAMERA_TARGET_MAX_TURN_RAD = 45.0f * (M_PI / 180.0f);
constexpr float CAMERA_SEARCH_FULL_TURN_RAD = 2.0f * M_PI;
constexpr float CAMERA_SEARCH_LEG_BASE_MM = 500.0f;
constexpr float CAMERA_SEARCH_LEG_MAX_MM = 1500.0f;
constexpr float CAMERA_SEARCH_MOVE_SPEED_MM_S = 800.0f;
constexpr float CAMERA_SEARCH_CORNER_RAD = 90.0f * (M_PI / 180.0f);

constexpr uint8_t CAMERA_GYRO_HISTORY_SIZE = 64;

enum class CameraPhase : uint8_t {
    WaitFrame,
    TurnDriving,
    TurnCoasting,
    StopSettling,
    Forward,
    GoalConfirm,
    SearchTranslate,
    LinkHold,
};

enum class CameraTurnPurpose : uint8_t {
    None,
    Target,
    SearchStep,
    SearchCorner,
};

enum class CameraAfterStop : uint8_t {
    WaitFrame,
    StartSearch,
    StartGoalConfirm,
};

struct CameraGyroSample {
    uint32_t timestamp_ms;
    float z_rad_s;
};

// カメラジャイロ履歴
struct CameraGyroHistory {
    CameraGyroSample samples[CAMERA_GYRO_HISTORY_SIZE]{};
    uint8_t count = 0;
    uint8_t write_index = 0;

    void clear()
    {
        count = 0;
        write_index = 0;
    }

    void add(const Can::Data::AngularVelocity& gyro)
    {
        if (count > 0) {
            // 最新データのタイムスタンプが同じ場合は追加しない
            const uint8_t newest_index =
                (write_index + CAMERA_GYRO_HISTORY_SIZE - 1U) %
                CAMERA_GYRO_HISTORY_SIZE;
            if (samples[newest_index].timestamp_ms == gyro.ts_ms) return;
        }
        samples[write_index] = {gyro.ts_ms, gyro.z_rad_s};
        write_index = (write_index + 1U) % CAMERA_GYRO_HISTORY_SIZE;
        if (count < CAMERA_GYRO_HISTORY_SIZE) ++count;
        // カウントが最大に達した場合、古いデータは上書きされる
    }

    bool integrateDelay(
        uint32_t from_ms,
        uint32_t to_ms,
        float& angle_rad) const
    {
        angle_rad = 0.0f;
        if (count < 2 || from_ms == 0 || to_ms <= from_ms) return false;

        const uint8_t first =
            (write_index + CAMERA_GYRO_HISTORY_SIZE - count) %
            CAMERA_GYRO_HISTORY_SIZE;
        bool covered_from = false;
        bool used_interval = false;

        for (uint8_t i = 0; i + 1U < count; ++i) {
            // 2点の間の積分区間が、指定されたfrom_ms～to_msに重なっている場合のみ積分する
            const CameraGyroSample& a =
                samples[(first + i) % CAMERA_GYRO_HISTORY_SIZE];
            const CameraGyroSample& b =
                samples[(first + i + 1U) % CAMERA_GYRO_HISTORY_SIZE];
            if (b.timestamp_ms <= a.timestamp_ms) continue;
            if (b.timestamp_ms < from_ms || a.timestamp_ms > to_ms) continue;

            // 積分区間の開始・終了点
            const uint32_t segment_start =
                a.timestamp_ms < from_ms ? from_ms : a.timestamp_ms;
            const uint32_t segment_end =
                b.timestamp_ms > to_ms ? to_ms : b.timestamp_ms;
            if (segment_end <= segment_start) continue;

            // 2点間の時間差（端以外は積分区間）
            const float interval_ms =
                static_cast<float>(b.timestamp_ms - a.timestamp_ms);

            // 積分区間の開始・終了点の角速度を線形補間する
            const float start_ratio =
                (segment_start - a.timestamp_ms) / interval_ms;
            const float end_ratio =
                (segment_end - a.timestamp_ms) / interval_ms;
            
            // 積分区間の開始・終了点の角速度（端は線形補完）
            float start_rate =
                a.z_rad_s + (b.z_rad_s - a.z_rad_s) * start_ratio;
            float end_rate =
                a.z_rad_s + (b.z_rad_s - a.z_rad_s) * end_ratio;

            // 画像遅延補正は静止ノイズを角度へ積算しない。
            if (fabsf(start_rate) < CAMERA_DELAY_GYRO_DEADBAND_RAD_S) {
                start_rate = 0.0f;
            }
            if (fabsf(end_rate) < CAMERA_DELAY_GYRO_DEADBAND_RAD_S) {
                end_rate = 0.0f;
            }

            // 台形積分
            angle_rad += 0.5f * (start_rate + end_rate) *
                ((segment_end - segment_start) / 1000.0f);
            covered_from = covered_from || a.timestamp_ms <= from_ms;
            used_interval = true;
        }
        return covered_from && used_interval;
    }
};

MotionCommandSource active_command_source =
    MotionCommandSource::GpsNavigation;

/**
 * Jog送信関数
 */
void publishJog(
    float velocity_mm_s,
    float omega_rad_s,
    uint32_t duration_ms = JOG_DURATION_MS)
{
    MotionCommandRequest request{};
    request.source = active_command_source;
    request.velocity_mm_s = static_cast<int16_t>(lroundf(velocity_mm_s));
    request.omega_rad_s_x100 =
        static_cast<int16_t>(lroundf(omega_rad_s * 100.0f));
    request.duration_ms = static_cast<uint16_t>(duration_ms);
    request.timestamp_ms = millis();
    xQueueSend(fifo_motion_command_request, &request, 0);
}

/**
 * Stop送信関数
 */
void publishStop()
{
    publishJog(0.0f, 0.0f);
}

/**
 * Nav
 */
float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

bool isOutsideField(const Coordinate& coordinate)
{
    return coordinate.x_mm < 0 || coordinate.y_mm < 0 ||
        coordinate.x_mm > FieldConfig::SIZE_X_MM ||
        coordinate.y_mm > FieldConfig::SIZE_Y_MM;
}

bool isInsideFieldInset(const Coordinate& coordinate)
{
    return coordinate.x_mm >= FIELD_RECOVERY_INSET_MM &&
        coordinate.y_mm >= FIELD_RECOVERY_INSET_MM &&
        coordinate.x_mm <=
            FieldConfig::SIZE_X_MM - FIELD_RECOVERY_INSET_MM &&
        coordinate.y_mm <=
            FieldConfig::SIZE_Y_MM - FIELD_RECOVERY_INSET_MM;
}

void driveTowardNearestFieldPoint(const Coordinate& coordinate)
{
    const float target_x_mm = clampFloat(
        static_cast<float>(coordinate.x_mm),
        FIELD_RECOVERY_INSET_MM,
        static_cast<float>(FieldConfig::SIZE_X_MM) -
            FIELD_RECOVERY_INSET_MM);
    const float target_y_mm = clampFloat(
        static_cast<float>(coordinate.y_mm),
        FIELD_RECOVERY_INSET_MM,
        static_cast<float>(FieldConfig::SIZE_Y_MM) -
            FIELD_RECOVERY_INSET_MM);
    const float target_heading_rad = atan2f(
        target_y_mm - static_cast<float>(coordinate.y_mm),
        target_x_mm - static_cast<float>(coordinate.x_mm));
    const float heading_error_rad = PurePursuit::normalizeAngleRad(
        target_heading_rad - coordinate.heading_rad);

    // まずその場旋回で最短方向へ向き、整列後だけ前進する。
    if (fabsf(heading_error_rad) > FIELD_RECOVERY_ALIGN_RAD) {
        publishJog(
            0.0f,
            heading_error_rad >= 0.0f
                ? FIELD_RECOVERY_TURN_RATE_RAD_S
                : -FIELD_RECOVERY_TURN_RATE_RAD_S);
        return;
    }

    const float omega_rad_s = clampFloat(
        heading_error_rad * FIELD_RECOVERY_HEADING_KP,
        -GPS_MAX_TURN_RATE_RAD_S,
        GPS_MAX_TURN_RATE_RAD_S);
    publishJog(GPS_MIN_SPEED_MM_S, omega_rad_s);
}

void publishNavigationProgress(
    uint32_t timestamp_ms,
    uint32_t path_revision,
    const PurePursuit::Output* output)
{
    NavigationProgress progress{};
    progress.timestamp_ms = timestamp_ms;
    progress.path_revision = path_revision;
    if (output != nullptr) {
        progress.nearest_index = output->nearest_index;
        progress.target_index = output->target_index;
        progress.distance_to_goal_mm = output->distance_to_goal_mm;
        progress.valid = output->valid && !output->reached_goal;
    }
    xQueueOverwrite(mbx_navigation_progress, &progress);
}

void requestState(SystemCmdType command)
{
    // 状態要求は重要イベントなので、taskStateが受け取るまで短時間待つ。
    xQueueSend(fifo_system_cmd, &command, portMAX_DELAY);
}

bool coordinateIsUsable(const Coordinate& coordinate, uint32_t now_ms)
{
    const bool fresh =
        static_cast<uint32_t>(now_ms - coordinate.timestamp_ms) <= COORDINATE_TIMEOUT_MS;
    const uint16_t required_status =
        Domain::Fusion::STATUS_POSITION_USABLE |
        Domain::Fusion::STATUS_YAW_USABLE;
    const bool fusion_usable =
        (coordinate.fusion_status_flags & required_status) ==
        required_status;

    return fresh && fusion_usable && coordinate.is_first_gps_valid;
}

bool cameraGyroIsUsable(
    const Can::Data::AngularVelocity& angular_velocity,
    uint32_t now_ms)
{
    return
        static_cast<uint32_t>(now_ms - angular_velocity.ts_ms) <=
            CAMERA_GYRO_TIMEOUT_MS &&
        isfinite(angular_velocity.z_rad_s) &&
        fabsf(angular_velocity.z_rad_s) <= CAMERA_MAX_GYRO_RATE_RAD_S;
}

bool driveCameraTurn(
    float target_turn_rad,
    float accumulated_turn_rad,
    float gyro_z_rad_s,
    float reached_rad = CAMERA_TURN_REACHED_RAD)
{
    const float remaining_turn_rad =
        target_turn_rad - accumulated_turn_rad;
    const float abs_remaining_turn_rad = fabsf(remaining_turn_rad);
    const float turn_direction = remaining_turn_rad >= 0.0f ? 1.0f : -1.0f;
    const float same_direction_rate =
        fmaxf(0.0f, turn_direction * gyro_z_rad_s);
    const float predicted_coast_angle =
        same_direction_rate * CAMERA_COAST_PREDICTION_S;
    const float stop_angle = reached_rad + predicted_coast_angle;

    if (abs_remaining_turn_rad <= stop_angle) {
        publishStop();
        return true;
    }

    const float omega_rad_s =
        turn_direction * CAMERA_TURN_OMEGA_RAD_S;

    publishJog(0.0f, omega_rad_s, CAMERA_CONTROL_COMMAND_MS);
    return false;
}

void loadInitialMapFromFlash()
{
    FlashDebugRequest request{};
    request.type = FlashDebugRequestType::ReadLatestMap;
    request.response_target = FlashResponseTarget::Navigation;
    request.request_id = NAV_MAP_REQUEST_ID;

    if (xQueueSend(
            fifo_flash_debug_request,
            &request,
            pdMS_TO_TICKS(INITIAL_MAP_WAIT_MS)) != pdTRUE) {
        return;
    }

    FlashDebugResponse response{};
    if (xQueueReceive(
            mbx_flash_nav_response,
            &response,
            pdMS_TO_TICKS(INITIAL_MAP_WAIT_MS)) != pdTRUE ||
        response.request_id != NAV_MAP_REQUEST_ID ||
        response.type != FlashDebugRequestType::ReadLatestMap ||
        response.result != FlashDebugResult::Ok ||
        response.data_size != AStar::MAP_BYTES) {
        return;
    }

    // Flashは読み出しだけを行い、共有地図への反映は唯一のwriterであるtaskNavが行う。
    if (xSemaphoreTake(mutex_grid_map, portMAX_DELAY) == pdTRUE) {
        memcpy(grid_map, response.data, AStar::MAP_BYTES);
        grid_map_update_count = 0;
        xSemaphoreGive(mutex_grid_map);
    }
}

} // namespace

void taskNav(void *pvParameters)
{
    (void)pvParameters;

    // 新規地図はUNKNOWNで作り、taskNavだけが以後の更新を行う。
    if (xSemaphoreTake(mutex_grid_map, portMAX_DELAY) == pdTRUE) {
        memset(grid_map, 0x77, AStar::MAP_BYTES);
        grid_map_update_count = 0;
        xSemaphoreGive(mutex_grid_map);
    }

    // 保存MAPが存在する場合はUNKNOWN初期地図を置き換える。
    loadInitialMapFromFlash();

    astar_config.cell_size_mm = FieldConfig::MAP_CELL_SIZE_MM;
    astar_config.origin_x_mm = FieldConfig::MAP_ORIGIN_X_MM;
    astar_config.origin_y_mm = FieldConfig::MAP_ORIGIN_Y_MM;

    // ナビゲーション速度はモーターが確実に始動できる範囲へ設定する。
    pp_config.base_speed_mm_s = GPS_BASE_SPEED_MM_S;
    pp_config.min_speed_mm_s = GPS_MIN_SPEED_MM_S;
    pp_config.max_speed_mm_s = GPS_MAX_SPEED_MM_S;
    pp_config.max_turn_rate_rad_s = GPS_MAX_TURN_RATE_RAD_S;
    pp_config.wheel_base_mm = ROBOT_TRACK_WIDTH_MM;

    bool path_valid = false;
    bool field_recovery_active = false;
    uint32_t planned_map_update_count = 0;
    uint32_t last_replan_ms = 0;
    SystemState previous_state = SystemState::STATE_PRELAUNCH;
    AStar::GridPos last_traversed_cell{};
    bool have_last_traversed_cell = false;

    uint32_t last_camera_msg_number = 0;
    uint32_t last_camera_frame_ms = 0;
    uint8_t camera_target_confirm_count = 0;
    uint8_t camera_goal_detect_count = 0;
    int16_t camera_angle_history[CAMERA_ANGLE_HISTORY_SIZE] = {};
    uint8_t camera_angle_history_count = 0;
    uint8_t camera_angle_history_write_index = 0;
    int16_t camera_filtered_angle_deg10 = 0;
    CameraPhase camera_phase = CameraPhase::WaitFrame;
    CameraAfterStop camera_after_stop = CameraAfterStop::WaitFrame;
    CameraTurnPurpose camera_turn_purpose = CameraTurnPurpose::None;
    float camera_turn_target_rad = 0.0f;
    float camera_accumulated_turn_rad = 0.0f;
    float camera_previous_gyro_z_rad_s = 0.0f;
    uint32_t camera_previous_gyro_timestamp_ms = 0;
    uint32_t camera_still_started_ms = 0;
    // 正面へ収まるまで、停止後画像による逆方向補正は1回だけ許可する。
    int8_t camera_last_target_turn_direction = 0;
    bool camera_target_reverse_used = false;
    bool camera_coarse_forward_active = false;
    uint8_t camera_large_error_confirm_count = 0;
    int8_t camera_large_error_direction = 0;
    CameraGyroHistory camera_gyro_history{};

    Can::Data::Encoder camera_previous_encoder{};
    bool camera_have_previous_encoder = false;
    float camera_left_velocity_mm_s = 0.0f;
    float camera_right_velocity_mm_s = 0.0f;
    uint32_t camera_encoder_velocity_ms = 0;

    bool camera_search_requested = false;
    float camera_search_accumulated_rad = 0.0f;
    uint8_t camera_search_leg_index = 0;
    int32_t camera_search_start_left_mm = 0;
    int32_t camera_search_start_right_mm = 0;
    float camera_search_target_distance_mm = 0.0f;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(NAV_PERIOD_MS);

    while (true) {
        xTaskDelayUntil(&last_wake, period);
        const uint32_t now_ms = millis();

        // taskStuckから届いた更新をまとめ、実際に変化したときだけcountを増やす。
        bool map_changed = false;
        MapUpdate update{};
        if (xSemaphoreTake(mutex_grid_map, portMAX_DELAY) == pdTRUE) {
            while (xQueueReceive(fifo_map_update, &update, 0) == pdTRUE) {
                AStar::GridPos cell{};
                if (!AStar::worldToGridChecked(
                        static_cast<float>(update.world_x_mm),
                        static_cast<float>(update.world_y_mm),
                        cell,
                        astar_config)) {
                    continue;
                }

                map_changed =
                    AStar::applyEvidenceKernel(
                        grid_map,
                        cell,
                        update.evidence_delta,
                        update.radius_cells,
                        update.maximum_value) ||
                    map_changed;
            }

            if (map_changed) {
                grid_map_update_count++;
            }
            xSemaphoreGive(mutex_grid_map);
        }

        if (map_changed) {
            path_valid = false;
        }

        SystemData status{};
        if (xQueuePeek(mbx_system_data, &status, 0) != pdTRUE) {
            active_command_source = MotionCommandSource::Safety;
            publishStop();
            continue;
        }

        if (status.state == SystemState::STATE_GPS_NAV) {
            active_command_source = MotionCommandSource::GpsNavigation;
        } else if (status.state == SystemState::STATE_CAMERA_NAV) {
            active_command_source = MotionCommandSource::CameraNavigation;
        } else if (status.state == SystemState::STATE_ESCAPE ||
                   status.state == SystemState::STATE_UPRIGHT_RECOVERY) {
            active_command_source = MotionCommandSource::Escape;
        } else {
            active_command_source =
                status.boot_mode != BootMode::MANUAL
                    ? MotionCommandSource::Safety
                    : MotionCommandSource::Stop;
        }

        // 状態へ入った直後は、その状態専用の追従履歴を初期化する。
        if (status.state != previous_state) {
            path_valid = false;
            pp_path_count = 0;
            pp_state = PurePursuit::PathState{};
            have_last_traversed_cell = false;
            field_recovery_active = false;

            if (status.state == SystemState::STATE_CAMERA_NAV) {
                last_camera_msg_number = 0;
                last_camera_frame_ms = 0;
                camera_target_confirm_count = 0;
                camera_goal_detect_count = 0;
                memset(camera_angle_history, 0, sizeof(camera_angle_history));
                camera_angle_history_count = 0;
                camera_angle_history_write_index = 0;
                camera_filtered_angle_deg10 = 0;
                camera_phase = CameraPhase::WaitFrame;
                camera_after_stop = CameraAfterStop::WaitFrame;
                camera_turn_purpose = CameraTurnPurpose::None;
                camera_turn_target_rad = 0.0f;
                camera_accumulated_turn_rad = 0.0f;
                camera_previous_gyro_z_rad_s = 0.0f;
                camera_previous_gyro_timestamp_ms = 0;
                camera_still_started_ms = 0;
                camera_last_target_turn_direction = 0;
                camera_target_reverse_used = false;
                camera_coarse_forward_active = false;
                camera_large_error_confirm_count = 0;
                camera_large_error_direction = 0;
                camera_gyro_history.clear();
                camera_previous_encoder = {};
                camera_have_previous_encoder = false;
                camera_left_velocity_mm_s = 0.0f;
                camera_right_velocity_mm_s = 0.0f;
                camera_encoder_velocity_ms = 0;
                camera_search_requested = false;
                camera_search_accumulated_rad = 0.0f;
                camera_search_leg_index = 0;
                camera_search_start_left_mm = 0;
                camera_search_start_right_mm = 0;
                camera_search_target_distance_mm = 0.0f;
            }
            previous_state = status.state;
            publishNavigationProgress(now_ms, planned_map_update_count, nullptr);
        }

        if (status.boot_mode == BootMode::DEBUG) {
            publishStop();
            continue;
        }

        if (status.state == SystemState::STATE_GPS_NAV) {
            Coordinate coordinate{};
            if (xQueuePeek(mbx_coordinate, &coordinate, 0) != pdTRUE ||
                !coordinateIsUsable(coordinate, now_ms)) {
                publishStop();
                publishNavigationProgress(
                    now_ms, planned_map_update_count, nullptr);
                continue;
            }

            // フィールド外では障害物地図を使わず、最寄りの内側点へ直行する。
            // 内側へ戻った次周期から通常のA*経路追従を再開する。
            if (isOutsideField(coordinate)) {
                field_recovery_active = true;
            }
            // GPS誤差で境界を往復しないよう、1 m内側まで復帰制御を維持する。
            if (field_recovery_active &&
                !isInsideFieldInset(coordinate)) {
                path_valid = false;
                have_last_traversed_cell = false;
                publishNavigationProgress(
                    now_ms, planned_map_update_count, nullptr);
                driveTowardNearestFieldPoint(coordinate);
                continue;
            }
            field_recovery_active = false;

            AStar::GridPos traversed_cell{};
            if (AStar::worldToGridChecked(
                    static_cast<float>(coordinate.x_mm),
                    static_cast<float>(coordinate.y_mm),
                    traversed_cell,
                    astar_config) &&
                (!have_last_traversed_cell ||
                 traversed_cell.x != last_traversed_cell.x ||
                 traversed_cell.y != last_traversed_cell.y)) {
                if (xSemaphoreTake(mutex_grid_map, portMAX_DELAY) == pdTRUE) {
                    const uint8_t old_value = AStar::getCell(
                        grid_map, traversed_cell.x, traversed_cell.y);
                    const uint8_t new_value = AStar::adjustCell(
                        grid_map,
                        traversed_cell.x,
                        traversed_cell.y,
                        -4);
                    if (new_value != old_value) {
                        grid_map_update_count++;
                        path_valid = false;
                    }
                    xSemaphoreGive(mutex_grid_map);
                }
                last_traversed_cell = traversed_cell;
                have_last_traversed_cell = true;
            }

            const float goal_dx_mm =
                static_cast<float>(coordinate.x_mm - FieldConfig::GOAL_X_MM);
            const float goal_dy_mm =
                static_cast<float>(coordinate.y_mm - FieldConfig::GOAL_Y_MM);
            const float distance_to_goal_mm =
                hypotf(goal_dx_mm, goal_dy_mm);

            // GPSゴール5m以内では、必ずカメラナビへ引き継ぐ。
            if (distance_to_goal_mm <= CAMERA_NAV_ENTER_DISTANCE_MM) {
                publishStop();
                publishNavigationProgress(
                    now_ms, planned_map_update_count, nullptr);
                requestState(SystemCmdType::StartCameraNav);
                continue;
            }

            uint32_t current_map_update_count = 0;
            if (xSemaphoreTake(mutex_grid_map, portMAX_DELAY) == pdTRUE) {
                current_map_update_count = grid_map_update_count;
                xSemaphoreGive(mutex_grid_map);
            }

            const bool retry_due =
                static_cast<uint32_t>(now_ms - last_replan_ms) >= REPLAN_RETRY_MS;
            const bool should_replan =
                !path_valid || planned_map_update_count != current_map_update_count;

            if (should_replan && retry_due) {
                last_replan_ms = now_ms;
                AStar::Result result = AStar::findPathFromWorldForPurePursuit(
                    grid_map,
                    static_cast<float>(coordinate.x_mm),
                    static_cast<float>(coordinate.y_mm),
                    static_cast<float>(FieldConfig::GOAL_X_MM),
                    static_cast<float>(FieldConfig::GOAL_Y_MM),
                    pp_path,
                    128,
                    grid_path,
                    128,
                    astar_work,
                    astar_config
                );

                // 通常経路が無い場合だけ、値15を値14相当の高コストとして再探索する。
                // 地図の証拠値自体は変更せず、次回もまず通常の安全な探索を行う。
                if (!result.found || result.path_overflow ||
                    result.path_count == 0) {
                    AStar::Config relaxed_config = astar_config;
                    relaxed_config.allow_blocked_as_high_cost = true;
                    // GPS誤差で地図余白を一時的に外れた場合も、最寄りの
                    // 境界セルへ丸めて停止ループを避ける。
                    const AStar::GridPos relaxed_start =
                        AStar::worldToGrid(
                            static_cast<float>(coordinate.x_mm),
                            static_cast<float>(coordinate.y_mm),
                            relaxed_config);
                    const AStar::GridPos relaxed_goal =
                        AStar::worldToGrid(
                            static_cast<float>(FieldConfig::GOAL_X_MM),
                            static_cast<float>(FieldConfig::GOAL_Y_MM),
                            relaxed_config);
                    result = AStar::findPathForPurePursuit(
                        grid_map,
                        relaxed_start,
                        relaxed_goal,
                        pp_path,
                        128,
                        grid_path,
                        128,
                        astar_work,
                        relaxed_config
                    );
                }

                path_valid = result.found && !result.path_overflow && result.path_count > 0;
                pp_path_count = path_valid ? result.path_count : 0;
                planned_map_update_count = current_map_update_count;
                pp_state = PurePursuit::PathState{};
            }

            if (!path_valid) {
                publishStop();
                publishNavigationProgress(
                    now_ms, planned_map_update_count, nullptr);
                continue;
            }

            const PurePursuit::Pose pose{
                static_cast<float>(coordinate.x_mm),
                static_cast<float>(coordinate.y_mm),
                coordinate.heading_rad
            };
            const PurePursuit::Output output = PurePursuit::calculatePathFollowing(
                pose,
                pp_path,
                pp_path_count,
                pp_state,
                pp_config
            );
            publishNavigationProgress(
                now_ms, planned_map_update_count, &output);

            if (!output.valid || output.reached_goal) {
                publishStop();
            } else {
                publishJog(output.linear_velocity_mm_s, output.angular_velocity_rad_s);
            }
            continue;
        }

        if (status.state == SystemState::STATE_CAMERA_NAV) {
            publishNavigationProgress(
                now_ms, planned_map_update_count, nullptr);

            // 100Hzのジャイロを履歴へ保存し、旋回中と惰性中はデッドバンドなしで積分する。
            Can::Data::AngularVelocity camera_gyro{};
            const bool camera_gyro_usable =
                xQueuePeek(mbx_can_angular_velocity, &camera_gyro, 0) == pdTRUE &&
                cameraGyroIsUsable(camera_gyro, now_ms);
            if (camera_gyro_usable) {
                camera_gyro_history.add(camera_gyro);
                if ((camera_phase == CameraPhase::TurnDriving ||
                     camera_phase == CameraPhase::TurnCoasting) &&
                    camera_previous_gyro_timestamp_ms != 0 &&
                    camera_gyro.ts_ms != camera_previous_gyro_timestamp_ms) {
                    const uint32_t gyro_dt_ms = static_cast<uint32_t>(
                        camera_gyro.ts_ms - camera_previous_gyro_timestamp_ms);
                    if (gyro_dt_ms <= CAMERA_GYRO_MAX_DT_MS) {
                        camera_accumulated_turn_rad += 0.5f *
                            (camera_previous_gyro_z_rad_s +
                             camera_gyro.z_rad_s) *
                            (gyro_dt_ms / 1000.0f);
                    }
                }
                if (camera_phase == CameraPhase::TurnDriving ||
                    camera_phase == CameraPhase::TurnCoasting) {
                    camera_previous_gyro_z_rad_s = camera_gyro.z_rad_s;
                    camera_previous_gyro_timestamp_ms = camera_gyro.ts_ms;
                }
            }

            // 累積エンコーダの実timestamp差から左右速度を求める。
            Can::Data::Encoder camera_encoder{};
            const bool camera_has_encoder =
                xQueuePeek(mbx_can_encoder, &camera_encoder, 0) == pdTRUE;
            if (camera_has_encoder &&
                (!camera_have_previous_encoder ||
                 camera_encoder.ts_ms != camera_previous_encoder.ts_ms)) {
                if (camera_have_previous_encoder &&
                    camera_encoder.ts_ms > camera_previous_encoder.ts_ms) {
                    const uint32_t encoder_dt_ms = static_cast<uint32_t>(
                        camera_encoder.ts_ms - camera_previous_encoder.ts_ms);
                    if (encoder_dt_ms <= CAMERA_ENCODER_TIMEOUT_MS) {
                        camera_left_velocity_mm_s =
                            (camera_encoder.left_mm -
                             camera_previous_encoder.left_mm) *
                            (1000.0f / encoder_dt_ms);
                        camera_right_velocity_mm_s =
                            (camera_encoder.right_mm -
                             camera_previous_encoder.right_mm) *
                            (1000.0f / encoder_dt_ms);
                        camera_encoder_velocity_ms = camera_encoder.ts_ms;
                    } else {
                        camera_encoder_velocity_ms = 0;
                    }
                }
                camera_previous_encoder = camera_encoder;
                camera_have_previous_encoder = true;
            }

            const bool camera_encoder_velocity_usable =
                camera_encoder_velocity_ms != 0 &&
                static_cast<uint32_t>(
                    now_ms - camera_encoder_velocity_ms) <=
                    CAMERA_ENCODER_TIMEOUT_MS;
            const bool camera_is_still_now =
                camera_gyro_usable && camera_encoder_velocity_usable &&
                fabsf(camera_gyro.z_rad_s) < CAMERA_STILL_GYRO_RAD_S &&
                fabsf(camera_left_velocity_mm_s) <
                    CAMERA_STILL_ENCODER_MM_S &&
                fabsf(camera_right_velocity_mm_s) <
                    CAMERA_STILL_ENCODER_MM_S;
            if (camera_is_still_now) {
                if (camera_still_started_ms == 0) {
                    camera_still_started_ms = now_ms;
                }
            } else {
                camera_still_started_ms = 0;
            }
            const bool camera_still_confirmed =
                camera_still_started_ms != 0 &&
                static_cast<uint32_t>(now_ms - camera_still_started_ms) >=
                    CAMERA_STILL_CONFIRM_MS;

            auto startCameraTurn = [&](
                CameraTurnPurpose purpose,
                float target_rad) {
                camera_phase = CameraPhase::TurnDriving;
                camera_turn_purpose = purpose;
                camera_turn_target_rad = target_rad;
                camera_accumulated_turn_rad = 0.0f;
                camera_still_started_ms = 0;
                camera_previous_gyro_z_rad_s =
                    camera_gyro_usable ? camera_gyro.z_rad_s : 0.0f;
                camera_previous_gyro_timestamp_ms =
                    camera_gyro_usable ? camera_gyro.ts_ms : 0;
            };

            auto startSearchTranslation = [&]() {
                if (!camera_has_encoder) {
                    camera_phase = CameraPhase::StopSettling;
                    camera_after_stop = CameraAfterStop::WaitFrame;
                    return;
                }
                const uint8_t expansion =
                    static_cast<uint8_t>(camera_search_leg_index / 2U + 1U);
                camera_search_target_distance_mm = fminf(
                    CAMERA_SEARCH_LEG_BASE_MM * expansion,
                    CAMERA_SEARCH_LEG_MAX_MM);
                camera_search_start_left_mm = camera_encoder.left_mm;
                camera_search_start_right_mm = camera_encoder.right_mm;
                camera_phase = CameraPhase::SearchTranslate;
                camera_still_started_ms = 0;
            };

            Rasp::CameraData camera_data{};
            const bool has_frame =
                xQueuePeek(mbx_camera_data, &camera_data, 0) == pdTRUE;
            const Rasp::Frame& frame = camera_data.frame;
            const bool new_frame = has_frame && frame.msg_number != last_camera_msg_number;

            if (new_frame) {
                last_camera_msg_number = frame.msg_number;
                last_camera_frame_ms = camera_data.received_ms;
                if (camera_phase == CameraPhase::LinkHold) {
                    camera_phase = CameraPhase::WaitFrame;
                }

                float delay_yaw_rad = 0.0f;
                camera_gyro_history.integrateDelay(
                    camera_data.requested_ms,
                    camera_data.received_ms,
                    delay_yaw_rad);
                const int32_t compensated_angle_deg10 = static_cast<int32_t>(
                    lroundf(frame.angle_error_deg10 -
                        delay_yaw_rad * (1800.0f / M_PI)));
                const int16_t latest_angle_deg10 = static_cast<int16_t>(
                    constrain(compensated_angle_deg10, -1800, 1800));

                if (frame.target_found != 0) {
                    // 低周期画像の遅れを抑えるため、最新90%・直前10%だけを混ぜる。
                    if (camera_angle_history_count > 0) {
                        const uint8_t previous_index =
                            (camera_angle_history_write_index +
                             CAMERA_ANGLE_HISTORY_SIZE - 1U) %
                            CAMERA_ANGLE_HISTORY_SIZE;
                        const int16_t previous_angle_deg10 =
                            camera_angle_history[previous_index];
                        const bool same_direction =
                            (latest_angle_deg10 >= 0) ==
                            (previous_angle_deg10 >= 0);
                        camera_filtered_angle_deg10 = same_direction
                            ? static_cast<int16_t>(
                                (9 * static_cast<int32_t>(latest_angle_deg10) +
                                 previous_angle_deg10) / 10)
                            : latest_angle_deg10;
                    } else {
                        camera_filtered_angle_deg10 = latest_angle_deg10;
                    }

                    camera_angle_history[camera_angle_history_write_index] =
                        latest_angle_deg10;
                    camera_angle_history_write_index =
                        (camera_angle_history_write_index + 1U) %
                        CAMERA_ANGLE_HISTORY_SIZE;
                    if (camera_angle_history_count < CAMERA_ANGLE_HISTORY_SIZE) {
                        ++camera_angle_history_count;
                    }

                    const bool goal_candidate =
                        frame.confidence >= CAMERA_GOAL_MIN_CONFIDENCE &&
                        frame.occupancy_permille >= CAMERA_GOAL_OCCUPANCY_PERMILLE &&
                        abs(camera_filtered_angle_deg10) <=
                            CAMERA_GOAL_MAX_ANGLE_DEG10;

                    // 目標を再発見したら、それまでの拡大四角形探索を最初からやり直す。
                    camera_search_requested = false;
                    camera_search_accumulated_rad = 0.0f;
                    camera_search_leg_index = 0;

                    if (camera_phase == CameraPhase::GoalConfirm) {
                        if (goal_candidate && camera_still_confirmed) {
                            if (camera_goal_detect_count <
                                CAMERA_GOAL_REQUIRED_FRAMES) {
                                ++camera_goal_detect_count;
                            }
                            if (camera_goal_detect_count >=
                                CAMERA_GOAL_REQUIRED_FRAMES) {
                                publishStop();
                                requestState(SystemCmdType::NotifyGoal);
                                camera_goal_detect_count = 0;
                                continue;
                            }
                        } else {
                            camera_goal_detect_count = 0;
                            camera_phase = CameraPhase::WaitFrame;
                        }
                    }

                    if (goal_candidate &&
                        camera_phase != CameraPhase::GoalConfirm) {
                        // 走行中の最初の候補は数えず、完全静止後の次フレームから数える。
                        publishStop();
                        camera_phase = CameraPhase::StopSettling;
                        camera_after_stop =
                            CameraAfterStop::StartGoalConfirm;
                        camera_turn_purpose = CameraTurnPurpose::None;
                        camera_goal_detect_count = 0;
                        camera_target_confirm_count = 0;
                        camera_still_started_ms = 0;
                    } else if (camera_phase == CameraPhase::WaitFrame) {
                        const int16_t abs_angle =
                            abs(camera_filtered_angle_deg10);
                        if (frame.occupancy_permille >=
                                CAMERA_GOAL_OCCUPANCY_PERMILLE &&
                            frame.confidence < CAMERA_GOAL_MIN_CONFIDENCE) {
                            publishStop();
                            camera_target_confirm_count = 0;
                        } else if (abs_angle >
                                   CAMERA_FORWARD_DEAD_BAND_DEG10) {
                            const int8_t requested_direction =
                                camera_filtered_angle_deg10 >= 0 ? 1 : -1;
                            const bool reversing =
                                camera_last_target_turn_direction != 0 &&
                                requested_direction !=
                                    camera_last_target_turn_direction;
                            if (camera_target_reverse_used &&
                                abs_angle <=
                                    CAMERA_COARSE_FORWARD_DEAD_BAND_DEG10) {
                                // 逆補正後の小さな行き過ぎは、再旋回せず低速前進で吸収する。
                                camera_large_error_confirm_count = 0;
                                camera_large_error_direction = 0;
                                camera_coarse_forward_active = true;
                                if (camera_target_confirm_count <
                                    CAMERA_TARGET_CONFIRM_FRAMES) {
                                    ++camera_target_confirm_count;
                                }
                                if (camera_target_confirm_count >=
                                    CAMERA_TARGET_CONFIRM_FRAMES) {
                                    camera_phase = CameraPhase::Forward;
                                }
                            } else if (camera_target_reverse_used) {
                                // 2回目の逆補正は即実行せず、同方向の停止画像2枚で再取得する。
                                publishStop();
                                camera_target_confirm_count = 0;
                                camera_coarse_forward_active = false;
                                if (camera_large_error_direction ==
                                    requested_direction) {
                                    if (camera_large_error_confirm_count <
                                        CAMERA_TARGET_CONFIRM_FRAMES) {
                                        ++camera_large_error_confirm_count;
                                    }
                                } else {
                                    camera_large_error_direction =
                                        requested_direction;
                                    camera_large_error_confirm_count = 1;
                                }
                                if (camera_large_error_confirm_count >=
                                    CAMERA_TARGET_CONFIRM_FRAMES) {
                                    // 補正履歴を解除し、この画像を新しい補正サイクルの開始にする。
                                    camera_last_target_turn_direction =
                                        requested_direction;
                                    camera_target_reverse_used = false;
                                    camera_large_error_confirm_count = 0;
                                    camera_large_error_direction = 0;
                                    const float requested_turn_rad =
                                        camera_filtered_angle_deg10 *
                                        (M_PI / 1800.0f);
                                    startCameraTurn(
                                        CameraTurnPurpose::Target,
                                        constrain(
                                            requested_turn_rad,
                                            -CAMERA_TARGET_MAX_TURN_RAD,
                                            CAMERA_TARGET_MAX_TURN_RAD));
                                }
                            } else {
                                camera_target_confirm_count = 0;
                                camera_large_error_confirm_count = 0;
                                camera_large_error_direction = 0;
                                camera_coarse_forward_active = false;
                                if (reversing) {
                                    camera_target_reverse_used = true;
                                }
                                camera_last_target_turn_direction =
                                    requested_direction;
                                const float requested_turn_rad =
                                    camera_filtered_angle_deg10 *
                                    (M_PI / 1800.0f);
                                startCameraTurn(
                                    CameraTurnPurpose::Target,
                                    constrain(
                                        requested_turn_rad,
                                        -CAMERA_TARGET_MAX_TURN_RAD,
                                        CAMERA_TARGET_MAX_TURN_RAD));
                            }
                        } else {
                            camera_last_target_turn_direction = 0;
                            camera_target_reverse_used = false;
                            camera_coarse_forward_active = false;
                            camera_large_error_confirm_count = 0;
                            camera_large_error_direction = 0;
                            if (camera_target_confirm_count <
                                CAMERA_TARGET_CONFIRM_FRAMES) {
                                ++camera_target_confirm_count;
                            }
                            if (camera_target_confirm_count >=
                                CAMERA_TARGET_CONFIRM_FRAMES) {
                                camera_phase = CameraPhase::Forward;
                            }
                        }
                    } else if (camera_phase == CameraPhase::Forward) {
                        if (camera_coarse_forward_active &&
                            abs(camera_filtered_angle_deg10) <=
                                CAMERA_FORWARD_DEAD_BAND_DEG10) {
                            // 低速前進中に正面へ戻ったら、次の制御周期から通常前進へ戻す。
                            camera_coarse_forward_active = false;
                            camera_last_target_turn_direction = 0;
                            camera_target_reverse_used = false;
                        }
                        const int16_t allowed_angle_deg10 =
                            camera_coarse_forward_active
                                ? CAMERA_COARSE_FORWARD_DEAD_BAND_DEG10
                                : CAMERA_FORWARD_DEAD_BAND_DEG10;
                        if (abs(camera_filtered_angle_deg10) >
                                allowed_angle_deg10 ||
                            frame.occupancy_permille >=
                                CAMERA_GOAL_OCCUPANCY_PERMILLE) {
                            publishStop();
                            camera_phase = CameraPhase::StopSettling;
                            camera_after_stop = CameraAfterStop::WaitFrame;
                            camera_target_confirm_count = 0;
                            camera_still_started_ms = 0;
                        }
                    } else if (
                        camera_phase == CameraPhase::SearchTranslate) {
                        publishStop();
                        camera_phase = CameraPhase::StopSettling;
                        camera_after_stop = CameraAfterStop::WaitFrame;
                        camera_still_started_ms = 0;
                    }
                } else {
                    // 探索へ進むのは、通信の鮮度切れではなく明示的な未検出だけ。
                    camera_goal_detect_count = 0;
                    camera_target_confirm_count = 0;
                    camera_last_target_turn_direction = 0;
                    camera_target_reverse_used = false;
                    camera_coarse_forward_active = false;
                    camera_large_error_confirm_count = 0;
                    camera_large_error_direction = 0;
                    camera_search_requested = true;
                    if (camera_phase == CameraPhase::Forward ||
                        camera_phase == CameraPhase::SearchTranslate ||
                        camera_phase == CameraPhase::GoalConfirm) {
                        publishStop();
                        camera_phase = CameraPhase::StopSettling;
                        camera_after_stop = CameraAfterStop::StartSearch;
                        camera_turn_purpose = CameraTurnPurpose::None;
                        camera_still_started_ms = 0;
                    } else if (camera_phase == CameraPhase::WaitFrame) {
                        if (camera_still_confirmed) {
                            startCameraTurn(
                                CameraTurnPurpose::SearchStep,
                                CAMERA_SEARCH_STEP_RAD);
                            camera_search_requested = false;
                        } else {
                            camera_phase = CameraPhase::StopSettling;
                            camera_after_stop =
                                CameraAfterStop::StartSearch;
                            camera_still_started_ms = 0;
                        }
                    }
                }
            }

            const uint32_t camera_frame_age_ms =
                last_camera_frame_ms != 0
                    ? static_cast<uint32_t>(now_ms - last_camera_frame_ms)
                    : UINT32_MAX;

            // 1200ms通信断では途中の制御判断を破棄し、復帰後の新規フレームを待つ。
            if (camera_frame_age_ms > CAMERA_COMM_TIMEOUT_MS) {
                publishStop();
                camera_phase = CameraPhase::LinkHold;
                camera_after_stop = CameraAfterStop::WaitFrame;
                camera_turn_purpose = CameraTurnPurpose::None;
                camera_turn_target_rad = 0.0f;
                camera_accumulated_turn_rad = 0.0f;
                camera_previous_gyro_timestamp_ms = 0;
                camera_target_confirm_count = 0;
                camera_goal_detect_count = 0;
                camera_last_target_turn_direction = 0;
                camera_target_reverse_used = false;
                camera_coarse_forward_active = false;
                camera_large_error_confirm_count = 0;
                camera_large_error_direction = 0;
                camera_search_requested = false;
                continue;
            }

            // 400ms鮮度切れは移動だけを止める。未検出や探索開始とは解釈しない。
            if (camera_frame_age_ms > CAMERA_CONTROL_FRAME_TIMEOUT_MS &&
                (camera_phase == CameraPhase::TurnDriving ||
                 camera_phase == CameraPhase::Forward ||
                 camera_phase == CameraPhase::SearchTranslate)) {
                publishStop();
                camera_phase = CameraPhase::StopSettling;
                camera_after_stop = CameraAfterStop::WaitFrame;
                camera_turn_purpose = CameraTurnPurpose::None;
                camera_previous_gyro_timestamp_ms = 0;
                camera_still_started_ms = 0;
            }

            if (camera_phase == CameraPhase::TurnDriving) {
                if (!camera_gyro_usable) {
                    publishStop();
                    camera_phase = CameraPhase::StopSettling;
                    camera_after_stop = CameraAfterStop::WaitFrame;
                    camera_turn_purpose = CameraTurnPurpose::None;
                    camera_previous_gyro_timestamp_ms = 0;
                    camera_still_started_ms = 0;
                } else {
                    const float reached_rad =
                        camera_turn_purpose == CameraTurnPurpose::SearchStep
                            ? CAMERA_SEARCH_REACHED_RAD
                            : CAMERA_TURN_REACHED_RAD;
                    if (driveCameraTurn(
                            camera_turn_target_rad,
                            camera_accumulated_turn_rad,
                            camera_gyro.z_rad_s,
                            reached_rad)) {
                        camera_phase = CameraPhase::TurnCoasting;
                        camera_still_started_ms = 0;
                    }
                }
                continue;
            }

            if (camera_phase == CameraPhase::TurnCoasting) {
                publishStop();
                if (!camera_still_confirmed) continue;

                const CameraTurnPurpose completed_purpose =
                    camera_turn_purpose;
                const float completed_turn_rad =
                    camera_accumulated_turn_rad;
                camera_turn_purpose = CameraTurnPurpose::None;
                camera_previous_gyro_timestamp_ms = 0;
                camera_still_started_ms = 0;

                if (completed_purpose == CameraTurnPurpose::SearchStep) {
                    camera_search_accumulated_rad +=
                        fabsf(completed_turn_rad);
                    if (camera_search_accumulated_rad >=
                        CAMERA_SEARCH_FULL_TURN_RAD) {
                        camera_search_accumulated_rad = 0.0f;
                        if (camera_search_leg_index == 0) {
                            startSearchTranslation();
                        } else {
                            startCameraTurn(
                                CameraTurnPurpose::SearchCorner,
                                CAMERA_SEARCH_CORNER_RAD);
                        }
                    } else {
                        camera_phase = CameraPhase::WaitFrame;
                    }
                } else if (
                    completed_purpose == CameraTurnPurpose::SearchCorner) {
                    startSearchTranslation();
                } else if (camera_search_requested) {
                    startCameraTurn(
                        CameraTurnPurpose::SearchStep,
                        CAMERA_SEARCH_STEP_RAD);
                    camera_search_requested = false;
                } else {
                    camera_phase = CameraPhase::WaitFrame;
                }
                continue;
            }

            if (camera_phase == CameraPhase::StopSettling) {
                publishStop();
                if (!camera_still_confirmed) continue;

                const CameraAfterStop action = camera_after_stop;
                camera_after_stop = CameraAfterStop::WaitFrame;
                camera_still_started_ms = 0;
                if (action == CameraAfterStop::StartGoalConfirm) {
                    camera_goal_detect_count = 0;
                    camera_phase = CameraPhase::GoalConfirm;
                } else if (action == CameraAfterStop::StartSearch) {
                    startCameraTurn(
                        CameraTurnPurpose::SearchStep,
                        CAMERA_SEARCH_STEP_RAD);
                    camera_search_requested = false;
                } else {
                    camera_phase = CameraPhase::WaitFrame;
                }
                continue;
            }

            if (camera_phase == CameraPhase::SearchTranslate) {
                if (!camera_has_encoder ||
                    !camera_encoder_velocity_usable) {
                    publishStop();
                    continue;
                }
                const float left_distance_mm = static_cast<float>(
                    camera_encoder.left_mm - camera_search_start_left_mm);
                const float right_distance_mm = static_cast<float>(
                    camera_encoder.right_mm - camera_search_start_right_mm);
                const float travelled_mm = fabsf(
                    0.5f * (left_distance_mm + right_distance_mm));

                if (travelled_mm >= camera_search_target_distance_mm) {
                    publishStop();
                    if (camera_search_leg_index < UINT8_MAX) {
                        ++camera_search_leg_index;
                    }
                    camera_search_accumulated_rad = 0.0f;
                    camera_phase = CameraPhase::StopSettling;
                    camera_after_stop = CameraAfterStop::WaitFrame;
                    camera_still_started_ms = 0;
                } else {
                    publishJog(
                        CAMERA_SEARCH_MOVE_SPEED_MM_S,
                        0.0f,
                        CAMERA_CONTROL_COMMAND_MS);
                }
                continue;
            }

            if (camera_phase == CameraPhase::Forward) {
                if (!has_frame || frame.target_found == 0) {
                    publishStop();
                } else {
                    const float forward_speed = camera_coarse_forward_active
                        ? CAMERA_NEAR_FORWARD_SPEED_MM_S
                        : (frame.occupancy_permille <
                            CAMERA_NEAR_OCCUPANCY_PERMILLE
                            ? CAMERA_FAR_FORWARD_SPEED_MM_S
                            : CAMERA_NEAR_FORWARD_SPEED_MM_S);
                    publishJog(
                        forward_speed,
                        0.0f,
                        CAMERA_CONTROL_COMMAND_MS);
                }
                continue;
            }

            // WaitFrame・GoalConfirm・LinkHoldは、新規フレームによる遷移まで停止する。
            publishStop();
            continue;
        }

        publishStop();
    }
}
