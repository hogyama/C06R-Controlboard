#include "tasks.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "algorithm/astar.h"
#include "algorithm/pure_pursuit.h"
#include "domain/localization/localization_types.h"
#include "domain/navigation/camera_navigator.h"
#include "platform/field_config.h"

#include <esp_timer.h>
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
constexpr float GPS_BASE_SPEED_MM_S = 700.0f;
constexpr float GPS_MIN_SPEED_MM_S = 500.0f;
constexpr float GPS_MAX_SPEED_MM_S = 850.0f;
constexpr float YAW_RECOVERY_SPEED_MM_S = 450.0f;
constexpr uint32_t YAW_ACQUIRE_DRIVE_MS = 1500;
constexpr uint32_t YAW_ACQUIRE_RETRY_WAIT_MS = 2000;
constexpr uint8_t YAW_ACQUIRE_MAX_ATTEMPTS = 2;
constexpr uint32_t YAW_RESET_RETRY_MS = 10000;
constexpr uint32_t YAW_RECOVERY_STABILIZE_MS = 1000;
constexpr uint32_t GPS_WARP_STABILIZE_MS = 1000;
constexpr float GPS_WARP_STABILIZE_SPEED_MM_S = 450.0f;

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


MotionCommandSource active_command_source =
    MotionCommandSource::GpsNavigation;
NavigationRecoveryPhase active_recovery_phase =
    NavigationRecoveryPhase::None;
uint16_t active_navigation_reset_count = 0;
PathMode active_path_mode = PathMode::None;

/**
 * Jog送信関数
 */
void publishJog(
    float velocity_mm_s,
    float omega_rad_s,
    uint32_t duration_ms = JOG_DURATION_MS,
    NavHoldReason hold_reason = NavHoldReason::None)
{
    MotionCommandRequest request{};
    request.source = active_command_source;
    request.velocity_mm_s = static_cast<int16_t>(lroundf(velocity_mm_s));
    request.omega_rad_s_x100 =
        static_cast<int16_t>(lroundf(omega_rad_s * 100.0f));
    request.duration_ms = static_cast<uint16_t>(duration_ms);
    request.timestamp_ms = millis();
    request.nav_hold_reason = hold_reason;
    request.recovery_phase = active_recovery_phase;
    request.navigation_reset_count = active_navigation_reset_count;
    xQueueSend(fifo_motion_command_request, &request, 0);
}

/**
 * Stop送信関数
 */
void publishStop(NavHoldReason hold_reason = NavHoldReason::None)
{
    publishJog(0.0f, 0.0f, JOG_DURATION_MS, hold_reason);
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
    publishJog(GPS_BASE_SPEED_MM_S, omega_rad_s);
}

void driveDirectlyToward(
    const Coordinate& coordinate,
    float target_x_mm,
    float target_y_mm,
    float speed_mm_s = GPS_BASE_SPEED_MM_S)
{
    const float heading = atan2f(
        target_y_mm - static_cast<float>(coordinate.y_mm),
        target_x_mm - static_cast<float>(coordinate.x_mm));
    const float error = PurePursuit::normalizeAngleRad(
        heading - coordinate.heading_rad);
    if (fabsf(error) > 45.0f * (M_PI / 180.0f)) {
        publishJog(0.0f, error >= 0.0f
            ? GPS_MAX_TURN_RATE_RAD_S : -GPS_MAX_TURN_RATE_RAD_S);
        return;
    }
    publishJog(
        speed_mm_s,
        clampFloat(error * 2.0f,
            -GPS_MAX_TURN_RATE_RAD_S,
            GPS_MAX_TURN_RATE_RAD_S));
}

void publishNavigationProgress(
    uint32_t timestamp_ms,
    uint32_t path_revision,
    const PurePursuit::Output* output)
{
    NavigationProgress progress{};
    progress.timestamp_ms = timestamp_ms;
    progress.path_revision = path_revision;
    progress.path_mode = active_path_mode;
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
        Domain::Localization::STATUS_POSITION_USABLE |
        Domain::Localization::STATUS_YAW_USABLE;
    const bool localization_usable =
        (coordinate.localization_status_flags & required_status) ==
        required_status;

    return fresh && localization_usable && coordinate.is_first_gps_valid;
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
    bool path_chunked = false;
    bool direct_path_fallback = false;
    uint8_t pure_pursuit_invalid_count = 0;
    bool field_recovery_active = false;
    uint32_t planned_map_update_count = 0;
    uint32_t last_replan_ms = 0;
    SystemState previous_state = SystemState::STATE_PRELAUNCH;
    NavigationRecoveryPhase recovery_phase =
        NavigationRecoveryPhase::None;
    NavHoldReason recovery_reason = NavHoldReason::None;
    uint32_t recovery_phase_started_ms = 0;
    uint32_t last_navigation_reset_request_ms = 0;
    uint8_t yaw_acquire_attempts = 0;
    uint32_t observed_gps_warp_count = 0;
    uint32_t gps_warp_stabilize_until_ms = 0;
    AStar::GridPos last_traversed_cell{};
    bool have_last_traversed_cell = false;


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
            active_recovery_phase = NavigationRecoveryPhase::None;
            active_navigation_reset_count = 0;
            publishStop();
            continue;
        }
        active_navigation_reset_count =
            status.navigation_reset_count;
        active_recovery_phase = recovery_phase;

        if (status.state == SystemState::STATE_GPS_NAV) {
            active_command_source = MotionCommandSource::GpsNavigation;
        } else if (status.state == SystemState::STATE_CAMERA_NAV) {
            active_command_source = MotionCommandSource::CameraNavigation;
        } else if (status.state == SystemState::STATE_ESCAPE ||
                   status.state == SystemState::STATE_UPRIGHT_RECOVERY) {
            active_command_source = MotionCommandSource::Escape;
        } else if (status.state == SystemState::STATE_STUCK_SUSPEND) {
            // taskStuck owns the bounded verification probe. Keep taskNav's
            // stop request at the lowest priority so it cannot mask the probe.
            active_command_source = MotionCommandSource::Stop;
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
            recovery_phase = NavigationRecoveryPhase::None;
            recovery_reason = NavHoldReason::None;
            recovery_phase_started_ms = 0;
            last_navigation_reset_request_ms = 0;
            yaw_acquire_attempts = 0;
            active_recovery_phase = NavigationRecoveryPhase::None;
            active_path_mode = PathMode::None;

            if (status.state == SystemState::STATE_CAMERA_NAV) {
                CameraNavigation::reset();
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
            const bool has_coordinate =
                xQueuePeek(mbx_coordinate, &coordinate, 0) == pdTRUE;
            const bool coordinate_fresh = has_coordinate &&
                coordinate.timestamp_ms != 0U &&
                static_cast<uint32_t>(
                    now_ms - coordinate.timestamp_ms) <=
                    COORDINATE_TIMEOUT_MS;
            const bool position_usable = coordinate_fresh &&
                (coordinate.localization_status_flags &
                 Domain::Localization::STATUS_POSITION_USABLE) != 0U;
            const bool yaw_usable = coordinate_fresh &&
                (coordinate.localization_status_flags &
                 Domain::Localization::STATUS_YAW_USABLE) != 0U;
            const bool coordinate_usable =
                coordinate_fresh && position_usable && yaw_usable &&
                coordinate.is_first_gps_valid;
            if (has_coordinate &&
                coordinate.gps_warp_count != observed_gps_warp_count) {
                observed_gps_warp_count = coordinate.gps_warp_count;
                gps_warp_stabilize_until_ms =
                    now_ms + GPS_WARP_STABILIZE_MS;
                path_valid = false;
                pp_state = PurePursuit::PathState{};
                last_replan_ms = now_ms - REPLAN_RETRY_MS;
            }
            const bool gps_warp_stabilizing =
                static_cast<int32_t>(
                    now_ms - gps_warp_stabilize_until_ms) < 0;

            const bool yaw_course_acquire_allowed =
                coordinate_fresh && position_usable && !yaw_usable &&
                coordinate.is_first_gps_valid &&
                coordinate.gps_health !=
                    Domain::Localization::SensorHealth::Failed &&
                coordinate.imu_health ==
                    Domain::Localization::SensorHealth::Failed &&
                coordinate.encoder_health ==
                    Domain::Localization::SensorHealth::Failed &&
                coordinate.magnetic_health ==
                    Domain::Localization::SensorHealth::Failed;

            if (recovery_phase == NavigationRecoveryPhase::None &&
                !coordinate_usable) {
                active_path_mode = PathMode::None;
                recovery_phase = yaw_course_acquire_allowed &&
                        yaw_acquire_attempts < YAW_ACQUIRE_MAX_ATTEMPTS
                    ? NavigationRecoveryPhase::YawCourseAcquire
                    : NavigationRecoveryPhase::AwaitFreshGps;
                recovery_phase_started_ms = now_ms;
                path_valid = false;
                if (!has_coordinate || !coordinate_fresh) {
                    recovery_reason =
                        NavHoldReason::CoordinateUnavailable;
                } else if (!position_usable) {
                    recovery_reason = NavHoldReason::PositionUnusable;
                } else if (!yaw_usable) {
                    recovery_reason = NavHoldReason::YawUnusable;
                } else recovery_reason = NavHoldReason::LocalizationFailed;
            }

            if (recovery_phase != NavigationRecoveryPhase::None) {
                if (coordinate_usable) {
                    if (recovery_phase !=
                        NavigationRecoveryPhase::Stabilizing) {
                        recovery_phase =
                            NavigationRecoveryPhase::Stabilizing;
                        recovery_phase_started_ms = now_ms;
                    } else if (static_cast<uint32_t>(
                            now_ms - recovery_phase_started_ms) >=
                            YAW_RECOVERY_STABILIZE_MS) {
                        recovery_phase = NavigationRecoveryPhase::None;
                        recovery_reason = NavHoldReason::None;
                        recovery_phase_started_ms = 0U;
                        yaw_acquire_attempts = 0U;
                        path_valid = false;
                    }
                }

                if (recovery_phase != NavigationRecoveryPhase::None) {
                    if (recovery_phase ==
                            NavigationRecoveryPhase::YawCourseAcquire &&
                        static_cast<uint32_t>(
                            now_ms - recovery_phase_started_ms) >=
                            YAW_ACQUIRE_DRIVE_MS) {
                        ++yaw_acquire_attempts;
                        recovery_phase =
                            NavigationRecoveryPhase::AwaitFreshGps;
                        recovery_phase_started_ms = now_ms;
                        recovery_reason = NavHoldReason::YawUnusable;
                    } else if (recovery_phase ==
                                   NavigationRecoveryPhase::AwaitFreshGps &&
                               yaw_course_acquire_allowed &&
                               yaw_acquire_attempts <
                                   YAW_ACQUIRE_MAX_ATTEMPTS &&
                               static_cast<uint32_t>(
                                   now_ms - recovery_phase_started_ms) >=
                                   YAW_ACQUIRE_RETRY_WAIT_MS) {
                        recovery_phase =
                            NavigationRecoveryPhase::YawCourseAcquire;
                        recovery_phase_started_ms = now_ms;
                    } else if (recovery_phase ==
                                   NavigationRecoveryPhase::AwaitFreshGps &&
                               has_coordinate &&
                               coordinate.localization_quality ==
                                   Domain::Localization::Quality::Failed &&
                               static_cast<uint32_t>(
                                   now_ms -
                                   last_navigation_reset_request_ms) >=
                                   YAW_RESET_RETRY_MS) {
                        requestState(SystemCmdType::NavigationRecoveryReset);
                        last_navigation_reset_request_ms = now_ms;
                        recovery_reason = NavHoldReason::RecoveryReset;
                    }

                    active_command_source =
                        MotionCommandSource::NavigationRecovery;
                    active_recovery_phase = recovery_phase;
                    active_navigation_reset_count =
                        status.navigation_reset_count;
                    if (recovery_phase ==
                        NavigationRecoveryPhase::YawCourseAcquire) {
                        publishJog(
                            YAW_RECOVERY_SPEED_MM_S,
                            0.0f,
                            JOG_DURATION_MS,
                            recovery_reason);
                    } else {
                        publishStop(recovery_reason);
                    }
                    publishNavigationProgress(
                        now_ms, planned_map_update_count, nullptr);
                    continue;
                }
            }

            active_recovery_phase = NavigationRecoveryPhase::None;
            active_command_source = MotionCommandSource::GpsNavigation;
            if (!coordinateIsUsable(coordinate, now_ms)) {
                publishStop(NavHoldReason::LocalizationFailed);
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
                active_path_mode = PathMode::FieldRecovery;
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
                publishStop(NavHoldReason::GoalTransition);
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
                active_path_mode = PathMode::AStarNormal;
                AStar::Result result = AStar::findPathFromWorldForPurePursuit(
                    grid_map,
                    static_cast<float>(coordinate.x_mm),
                    static_cast<float>(coordinate.y_mm),
                    static_cast<float>(FieldConfig::GOAL_X_MM),
                    static_cast<float>(FieldConfig::GOAL_Y_MM),
                    pp_path,
                    128,
                    grid_path,
                    AStar::CELL_COUNT,
                    astar_work,
                    astar_config
                );

                // 通常経路が無い場合だけ、値15を値14相当の高コストとして再探索する。
                // 地図の証拠値自体は変更せず、次回もまず通常の安全な探索を行う。
                if (!result.found ||
                    result.path_count == 0) {
                    active_path_mode = PathMode::AStarRelaxed;
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
                        AStar::CELL_COUNT,
                        astar_work,
                        relaxed_config
                    );
                }

                if (!result.found || result.path_count == 0U) {
                    active_path_mode = PathMode::AStarEmergency;
                    AStar::Config emergency_config = astar_config;
                    emergency_config.allow_blocked_as_high_cost = true;
                    emergency_config.unknown_extra_cost = 0U;
                    result = AStar::findPathFromWorldForPurePursuit(
                        grid_map,
                        static_cast<float>(coordinate.x_mm),
                        static_cast<float>(coordinate.y_mm),
                        static_cast<float>(FieldConfig::GOAL_X_MM),
                        static_cast<float>(FieldConfig::GOAL_Y_MM),
                        pp_path,
                        128,
                        grid_path,
                        AStar::CELL_COUNT,
                        astar_work,
                        emergency_config);
                }

                path_valid = result.found && result.path_count > 0;
                pp_path_count = path_valid ? result.path_count : 0;
                path_chunked = path_valid && result.path_overflow;
                if (path_chunked) {
                    active_path_mode = PathMode::AStarChunked;
                }
                direct_path_fallback = !path_valid;
                if (direct_path_fallback) {
                    active_path_mode = PathMode::DirectFallback;
                    pp_path[0] = {
                        static_cast<float>(coordinate.x_mm),
                        static_cast<float>(coordinate.y_mm)};
                    pp_path[1] = {
                        static_cast<float>(FieldConfig::GOAL_X_MM),
                        static_cast<float>(FieldConfig::GOAL_Y_MM)};
                    pp_path_count = 2U;
                    path_valid = true;
                    path_chunked = false;
                }
                planned_map_update_count = current_map_update_count;
                pp_state = PurePursuit::PathState{};
                pure_pursuit_invalid_count = 0U;
            }

            if (!path_valid) {
                publishStop(NavHoldReason::PathUnavailable);
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

            if (!output.valid) {
                path_valid = false;
                last_replan_ms = now_ms - REPLAN_RETRY_MS;
                if (pure_pursuit_invalid_count < UINT8_MAX) {
                    ++pure_pursuit_invalid_count;
                }
                if (pure_pursuit_invalid_count >= 3U ||
                    direct_path_fallback) {
                    active_path_mode = PathMode::DirectFallback;
                    driveDirectlyToward(
                        coordinate,
                        static_cast<float>(FieldConfig::GOAL_X_MM),
                        static_cast<float>(FieldConfig::GOAL_Y_MM),
                        gps_warp_stabilizing
                            ? GPS_WARP_STABILIZE_SPEED_MM_S
                            : GPS_BASE_SPEED_MM_S);
                } else {
                    publishStop(NavHoldReason::PathUnavailable);
                }
            } else if (output.reached_goal) {
                path_valid = false;
                last_replan_ms = now_ms - REPLAN_RETRY_MS;
                publishStop(path_chunked
                    ? NavHoldReason::PathUnavailable
                    : NavHoldReason::GoalTransition);
            } else {
                pure_pursuit_invalid_count = 0U;
                const float velocity_mm_s = gps_warp_stabilizing
                    ? clampFloat(
                        output.linear_velocity_mm_s,
                        -GPS_WARP_STABILIZE_SPEED_MM_S,
                        GPS_WARP_STABILIZE_SPEED_MM_S)
                    : output.linear_velocity_mm_s;
                publishJog(velocity_mm_s, output.angular_velocity_rad_s);
            }
            continue;
        }

        if (status.state == SystemState::STATE_CAMERA_NAV) {
            active_path_mode = PathMode::None;
            publishNavigationProgress(
                now_ms, planned_map_update_count, nullptr);

            CameraNavigation::Input input{};
            input.now_ms = now_ms;
            input.now_us = static_cast<uint64_t>(esp_timer_get_time());
            input.has_camera =
                xQueuePeek(mbx_camera_data, &input.camera, 0) == pdTRUE;
            input.has_gyroscope =
                xQueuePeek(mbx_gyroscope, &input.gyroscope, 0) == pdTRUE;
            input.has_encoder =
                xQueuePeek(mbx_can_encoder, &input.encoder, 0) == pdTRUE;

            const CameraNavigation::Output output =
                CameraNavigation::update(input);
            publishJog(
                output.velocity_mm_s,
                output.omega_rad_s,
                output.duration_ms);
            if (output.link_lost) {
                Coordinate fallback_coordinate{};
                if (xQueuePeek(
                        mbx_coordinate,
                        &fallback_coordinate,
                        0) == pdTRUE &&
                    coordinateIsUsable(fallback_coordinate, now_ms)) {
                    requestState(SystemCmdType::StartGpsNav);
                }
            }
            if (output.goal_reached) {
                requestState(SystemCmdType::NotifyGoal);
            }
            continue;
        }

        publishStop();
    }
}
