#include "tasks.h"
#include "app_types.h"
#include "app_queue.h"
#include "algorithm/astar.h"
#include "domain/fusion/fusion_types.h"
#include "domain/motion/stuck_detector.h"

#include <math.h>

namespace {

constexpr uint32_t STUCK_TASK_PERIOD_MS = 100;
constexpr uint32_t ENCODER_FRESH_MS = 300;
constexpr uint32_t GYRO_FRESH_MS = 300;
constexpr uint32_t GPS_FRESH_MS = 2500;
constexpr uint32_t COORDINATE_FRESH_MS = 500;
constexpr uint32_t NAVIGATION_PROGRESS_FRESH_MS = 500;
constexpr uint32_t MAX_INTEGRATION_DT_MS = 200;
constexpr int32_t MAX_ENCODER_DELTA_MM = 1000;
constexpr float MAX_WHEEL_VELOCITY_MM_S = 5000.0f;
constexpr float MAX_GYRO_YAW_RATE_RAD_S = 20.0f;
constexpr float OBSTACLE_DISTANCE_MM = 1000.0f;
constexpr float SIDE_OBSTACLE_DISTANCE_MM = 700.0f;
constexpr uint32_t FLIPPED_CONFIRM_MS = 500;

bool isFresh(
    uint32_t now_ms,
    uint32_t timestamp_ms,
    uint32_t timeout_ms)
{
    return timestamp_ms != 0 &&
        static_cast<uint32_t>(now_ms - timestamp_ms) <= timeout_ms;
}

void sendStuckNotification(StuckReason reason)
{
    const SystemCmdType command = reason == StuckReason::Flipped
        ? SystemCmdType::NotifyFlipped
        : SystemCmdType::NotifyStuck;
    xQueueSend(fifo_system_cmd, &command, portMAX_DELAY);
}

bool isPhysicalObstacleReason(StuckReason reason)
{
    switch (reason) {
        case StuckReason::TranslationBlocked:
        case StuckReason::RotationBlocked:
        case StuckReason::EncoderGpsMismatch:
        case StuckReason::GpsNoProgress:
        case StuckReason::LeftWheelBlocked:
        case StuckReason::RightWheelBlocked:
        case StuckReason::WheelSlip:
        case StuckReason::PathNoProgress:
        case StuckReason::Oscillation:
            return true;

        case StuckReason::None:
        case StuckReason::Flipped:
        case StuckReason::MotionUnobservable:
        case StuckReason::SensorFault:
            return false;
    }
    return false;
}

bool registerObstacle(
    const Coordinate& coordinate,
    float commanded_velocity_mm_s,
    StuckReason reason,
    uint8_t& obstacle_cell_x,
    uint8_t& obstacle_cell_y)
{
    if (!isPhysicalObstacleReason(reason)) return false;

    const uint16_t required_status =
        Domain::Fusion::STATUS_POSITION_USABLE |
        Domain::Fusion::STATUS_YAW_USABLE;
    if ((coordinate.fusion_status_flags & required_status) !=
        required_status) {
        return false;
    }

    const float direction =
        commanded_velocity_mm_s >= 0.0f ? 1.0f : -1.0f;
    const float forward_x = cosf(coordinate.heading_rad);
    const float forward_y = sinf(coordinate.heading_rad);
    const float left_x = -forward_y;
    const float left_y = forward_x;

    MapUpdate update{};
    update.world_x_mm = coordinate.x_mm;
    update.world_y_mm = coordinate.y_mm;
    update.evidence_delta = 3;
    update.radius_cells = 1;
    update.maximum_value = 14;

    switch (reason) {
        case StuckReason::TranslationBlocked:
            update.world_x_mm = static_cast<int32_t>(
                coordinate.x_mm +
                direction * OBSTACLE_DISTANCE_MM * forward_x);
            update.world_y_mm = static_cast<int32_t>(
                coordinate.y_mm +
                direction * OBSTACLE_DISTANCE_MM * forward_y);
            update.evidence_delta = 5;
            update.maximum_value = AStar::CELL_BLOCKED;
            break;

        case StuckReason::LeftWheelBlocked:
            update.world_x_mm = static_cast<int32_t>(
                coordinate.x_mm +
                0.5f * direction * OBSTACLE_DISTANCE_MM * forward_x +
                SIDE_OBSTACLE_DISTANCE_MM * left_x);
            update.world_y_mm = static_cast<int32_t>(
                coordinate.y_mm +
                0.5f * direction * OBSTACLE_DISTANCE_MM * forward_y +
                SIDE_OBSTACLE_DISTANCE_MM * left_y);
            update.evidence_delta = 4;
            break;

        case StuckReason::RightWheelBlocked:
            update.world_x_mm = static_cast<int32_t>(
                coordinate.x_mm +
                0.5f * direction * OBSTACLE_DISTANCE_MM * forward_x -
                SIDE_OBSTACLE_DISTANCE_MM * left_x);
            update.world_y_mm = static_cast<int32_t>(
                coordinate.y_mm +
                0.5f * direction * OBSTACLE_DISTANCE_MM * forward_y -
                SIDE_OBSTACLE_DISTANCE_MM * left_y);
            update.evidence_delta = 4;
            break;

        case StuckReason::RotationBlocked:
            update.evidence_delta = 3;
            break;

        case StuckReason::EncoderGpsMismatch:
        case StuckReason::WheelSlip:
            update.evidence_delta = 4;
            break;

        case StuckReason::GpsNoProgress:
            update.world_x_mm = static_cast<int32_t>(
                coordinate.x_mm +
                direction * OBSTACLE_DISTANCE_MM * forward_x);
            update.world_y_mm = static_cast<int32_t>(
                coordinate.y_mm +
                direction * OBSTACLE_DISTANCE_MM * forward_y);
            update.evidence_delta = 2;
            update.maximum_value = 12;
            break;

        case StuckReason::PathNoProgress:
            update.world_x_mm = static_cast<int32_t>(
                coordinate.x_mm +
                direction * OBSTACLE_DISTANCE_MM * forward_x);
            update.world_y_mm = static_cast<int32_t>(
                coordinate.y_mm +
                direction * OBSTACLE_DISTANCE_MM * forward_y);
            update.evidence_delta = 3;
            update.radius_cells = 2;
            update.maximum_value = 13;
            break;

        case StuckReason::Oscillation:
            update.evidence_delta = 2;
            update.radius_cells = 2;
            update.maximum_value = 12;
            break;

        case StuckReason::None:
        case StuckReason::Flipped:
        case StuckReason::MotionUnobservable:
        case StuckReason::SensorFault:
            return false;
    }

    if (coordinate.position_std_mm > 3000U ||
        coordinate.yaw_std_rad > 0.60f) {
        update.evidence_delta =
            update.evidence_delta > 2
                ? update.evidence_delta - 2
                : 1;
        update.radius_cells = 2;
        if (update.maximum_value > 12U) update.maximum_value = 12U;
    } else if (coordinate.position_std_mm > 1500U ||
               coordinate.yaw_std_rad > 0.35f) {
        update.evidence_delta =
            update.evidence_delta > 1
                ? update.evidence_delta - 1
                : 1;
        update.radius_cells = 2;
        if (update.maximum_value > 14U) update.maximum_value = 14U;
    }

    AStar::Config map_config{};
    AStar::GridPos cell{};
    if (!AStar::worldToGridChecked(
            static_cast<float>(update.world_x_mm),
            static_cast<float>(update.world_y_mm),
            cell,
            map_config)) {
        return false;
    }
    if (xQueueSend(
            fifo_map_update,
            &update,
            pdMS_TO_TICKS(200)) != pdTRUE) {
        return false;
    }

    obstacle_cell_x = cell.x;
    obstacle_cell_y = cell.y;
    return true;
}

void clearPublishedStatus(uint32_t now_ms)
{
    StuckStatus clear_status{};
    clear_status.reason = StuckReason::None;
    clear_status.obstacle_cell_x = UINT8_MAX;
    clear_status.obstacle_cell_y = UINT8_MAX;
    clear_status.timestamp_ms = now_ms;
    xQueueOverwrite(mbx_stuck_status, &clear_status);
}

} // namespace

void taskStuck(void *pvParameters)
{
    (void)pvParameters;

    Domain::Motion::StuckDetector detector{};
    uint32_t previous_loop_ms = millis();
    uint32_t flipped_since_ms = 0;
    bool stuck_notified = false;
    StuckReason published_reason = StuckReason::None;

    Can::Data::Encoder previous_encoder{};
    bool have_previous_encoder = false;
    float encoder_left_velocity_mm_s = 0.0f;
    float encoder_right_velocity_mm_s = 0.0f;
    uint32_t encoder_velocity_timestamp_ms = 0;
    uint32_t previous_gps_timestamp_ms = 0;

    clearPublishedStatus(previous_loop_ms);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(STUCK_TASK_PERIOD_MS);

    while (true) {
        xTaskDelayUntil(&last_wake, period);
        const uint32_t now_ms = millis();
        uint32_t dt_ms =
            static_cast<uint32_t>(now_ms - previous_loop_ms);
        previous_loop_ms = now_ms;
        if (dt_ms > MAX_INTEGRATION_DT_MS) {
            dt_ms = MAX_INTEGRATION_DT_MS;
        }

        SystemData status{};
        const bool has_status =
            xQueuePeek(mbx_system_data, &status, 0) == pdTRUE;
        const bool navigation_active =
            has_status &&
            (status.state == SystemState::STATE_GPS_NAV ||
             status.state == SystemState::STATE_CAMERA_NAV) &&
            status.boot_mode != BootMode::DEBUG;

        if (!navigation_active) {
            detector.reset(now_ms);
            have_previous_encoder = false;
            encoder_velocity_timestamp_ms = 0;
            flipped_since_ms = 0;
            stuck_notified = false;
            if (published_reason != StuckReason::None &&
                status.state != SystemState::STATE_ESCAPE &&
                status.state != SystemState::STATE_UPRIGHT_RECOVERY) {
                clearPublishedStatus(now_ms);
                published_reason = StuckReason::None;
            }
            continue;
        }

        if (published_reason != StuckReason::None && !stuck_notified) {
            clearPublishedStatus(now_ms);
            published_reason = StuckReason::None;
        }
        if (stuck_notified) continue;

        JogData jog{};
        const bool jog_valid =
            xQueuePeek(mbx_can_jog_cmd, &jog, 0) == pdTRUE &&
            static_cast<uint32_t>(now_ms - jog.timestamp_ms) <
                jog.duration_ms;

        Coordinate coordinate{};
        const bool coordinate_fresh =
            xQueuePeek(mbx_coordinate, &coordinate, 0) == pdTRUE &&
            isFresh(
                now_ms,
                coordinate.timestamp_ms,
                COORDINATE_FRESH_MS);
        const bool fusion_initialized =
            coordinate_fresh &&
            (coordinate.fusion_status_flags &
             Domain::Fusion::STATUS_INITIALIZED) != 0U;
        const bool fusion_position_usable =
            fusion_initialized &&
            (coordinate.fusion_status_flags &
             Domain::Fusion::STATUS_POSITION_USABLE) != 0U;
        const bool fusion_yaw_usable =
            fusion_initialized &&
            (coordinate.fusion_status_flags &
             Domain::Fusion::STATUS_YAW_USABLE) != 0U;

        Can::Data::Encoder encoder{};
        const bool encoder_available =
            xQueuePeek(mbx_can_encoder, &encoder, 0) == pdTRUE &&
            isFresh(now_ms, encoder.ts_ms, ENCODER_FRESH_MS);
        const bool encoder_trusted =
            encoder_available &&
            (!coordinate_fresh ||
             (coordinate.fusion_status_flags &
              Domain::Fusion::STATUS_ENCODER_UNHEALTHY) == 0U);
        bool encoder_updated = false;
        if (encoder_available) {
            if (have_previous_encoder &&
                encoder.ts_ms != previous_encoder.ts_ms) {
                const uint32_t encoder_dt_ms =
                    static_cast<uint32_t>(
                        encoder.ts_ms - previous_encoder.ts_ms);
                const int32_t delta_left =
                    encoder.left_mm - previous_encoder.left_mm;
                const int32_t delta_right =
                    encoder.right_mm - previous_encoder.right_mm;
                if (encoder_dt_ms > 0U &&
                    encoder_dt_ms <= ENCODER_FRESH_MS &&
                    abs(delta_left) <= MAX_ENCODER_DELTA_MM &&
                    abs(delta_right) <= MAX_ENCODER_DELTA_MM) {
                    const float scale =
                        1000.0f / static_cast<float>(encoder_dt_ms);
                    const float left_velocity_mm_s =
                        static_cast<float>(delta_left) * scale;
                    const float right_velocity_mm_s =
                        static_cast<float>(delta_right) * scale;
                    if (fabsf(left_velocity_mm_s) <=
                            MAX_WHEEL_VELOCITY_MM_S &&
                        fabsf(right_velocity_mm_s) <=
                            MAX_WHEEL_VELOCITY_MM_S) {
                        encoder_left_velocity_mm_s =
                            left_velocity_mm_s;
                        encoder_right_velocity_mm_s =
                            right_velocity_mm_s;
                        encoder_velocity_timestamp_ms = now_ms;
                        encoder_updated = true;
                    }
                }
            }
            previous_encoder = encoder;
            have_previous_encoder = true;
        } else {
            have_previous_encoder = false;
            encoder_velocity_timestamp_ms = 0;
            encoder_left_velocity_mm_s = 0.0f;
            encoder_right_velocity_mm_s = 0.0f;
        }
        const bool encoder_velocity_available =
            encoder_trusted &&
            isFresh(
                now_ms,
                encoder_velocity_timestamp_ms,
                ENCODER_FRESH_MS);

        Can::Data::AngularVelocity angular{};
        const bool gyro_available =
            xQueuePeek(
                mbx_can_angular_velocity,
                &angular,
                0) == pdTRUE &&
            isFresh(now_ms, angular.ts_ms, GYRO_FRESH_MS) &&
            isfinite(angular.z_rad_s) &&
            fabsf(angular.z_rad_s) <= MAX_GYRO_YAW_RATE_RAD_S;

        Domain::Fusion::GpsUpdate gps_data{};
        const bool gps_available =
            xQueuePeek(
                mbx_gps_local_observation,
                &gps_data,
                0) == pdTRUE &&
            gps_data.fix_ok &&
            gps_data.fix_type >= 3U &&
            gps_data.horizontal_accuracy_mm <= 3000U &&
            isFresh(now_ms, gps_data.timestamp_ms, GPS_FRESH_MS);
        const bool gps_updated =
            gps_available &&
            gps_data.timestamp_ms != previous_gps_timestamp_ms;
        if (gps_updated) {
            previous_gps_timestamp_ms = gps_data.timestamp_ms;
        }

        NavigationProgress navigation_progress{};
        const bool path_available =
            xQueuePeek(
                mbx_navigation_progress,
                &navigation_progress,
                0) == pdTRUE &&
            navigation_progress.valid &&
            isFresh(
                now_ms,
                navigation_progress.timestamp_ms,
                NAVIGATION_PROGRESS_FRESH_MS);

        Domain::Motion::DetectorSample sample{};
        sample.timestamp_ms = now_ms;
        sample.dt_ms = dt_ms;
        sample.navigation_active = navigation_active;
        sample.command_valid = jog_valid;
        sample.command_velocity_mm_s =
            jog_valid ? jog.velocity_mm_s : 0.0f;
        sample.command_yaw_rate_rad_s =
            jog_valid ? jog.omega_rad_s : 0.0f;
        sample.encoder_available = encoder_velocity_available;
        sample.encoder_updated = encoder_updated;
        sample.encoder_left_velocity_mm_s =
            encoder_left_velocity_mm_s;
        sample.encoder_right_velocity_mm_s =
            encoder_right_velocity_mm_s;
        sample.gyro_available = gyro_available;
        sample.gyro_yaw_rate_rad_s =
            gyro_available ? angular.z_rad_s : 0.0f;
        sample.fusion_available = fusion_initialized;
        sample.fusion_position_usable = fusion_position_usable;
        sample.fusion_yaw_usable = fusion_yaw_usable;
        sample.fusion_forward_velocity_mm_s =
            fusion_initialized
                ? coordinate.forward_velocity_mm_s
                : 0.0f;
        sample.fusion_yaw_rate_rad_s =
            fusion_initialized ? coordinate.yaw_rate_rad_s : 0.0f;
        sample.fusion_x_mm = coordinate.x_mm;
        sample.fusion_y_mm = coordinate.y_mm;
        const bool gps_trusted =
            gps_available &&
            (!coordinate_fresh ||
             (coordinate.fusion_status_flags &
              Domain::Fusion::STATUS_GPS_UNHEALTHY) == 0U);
        sample.gps_available = gps_trusted;
        sample.gps_updated = gps_updated;
        sample.gps_horizontal_accuracy_mm =
            gps_data.horizontal_accuracy_mm;
        sample.gps_x_mm = gps_data.x_mm;
        sample.gps_y_mm = gps_data.y_mm;
        sample.path_available = path_available;
        sample.path_revision = navigation_progress.path_revision;
        sample.path_nearest_index =
            navigation_progress.nearest_index;
        sample.path_distance_to_goal_mm =
            navigation_progress.distance_to_goal_mm;

        Domain::Motion::Assessment assessment =
            detector.update(sample);

        StuckReason reason = StuckReason::None;
        if (coordinate_fresh &&
            coordinate.attitude == Attitude::Flipped) {
            if (flipped_since_ms == 0) {
                flipped_since_ms = now_ms;
            }
            if (static_cast<uint32_t>(
                    now_ms - flipped_since_ms) >=
                FLIPPED_CONFIRM_MS) {
                reason = StuckReason::Flipped;
            }
        } else {
            flipped_since_ms = 0;
        }

        if (reason == StuckReason::None &&
            (assessment.condition ==
                 Domain::Motion::Condition::Stuck ||
             assessment.condition ==
                 Domain::Motion::Condition::SensorFault)) {
            reason = assessment.reason;
        }
        if (reason == StuckReason::None) continue;

        uint8_t obstacle_cell_x = UINT8_MAX;
        uint8_t obstacle_cell_y = UINT8_MAX;
        if (coordinate_fresh) {
            registerObstacle(
                coordinate,
                sample.command_velocity_mm_s,
                reason,
                obstacle_cell_x,
                obstacle_cell_y);
        }

        StuckStatus stuck_status{};
        stuck_status.reason = reason;
        stuck_status.obstacle_cell_x = obstacle_cell_x;
        stuck_status.obstacle_cell_y = obstacle_cell_y;
        stuck_status.timestamp_ms = now_ms;
        xQueueOverwrite(mbx_stuck_status, &stuck_status);
        published_reason = reason;

        sendStuckNotification(reason);
        stuck_notified = true;
    }
}
