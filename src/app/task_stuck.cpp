#include "tasks.h"
#include "app_types.h"
#include "app_queue.h"
#include "algorithm/astar.h"
#include "domain/localization/localization_types.h"
#include "domain/motion/flip_detector.h"
#include "domain/motion/stuck_detector.h"
#include "domain/sensor/sensor_freshness.h"
#include "platform/sensor_axis_transform.h"

#include <math.h>
#include <esp_timer.h>

namespace {

constexpr uint32_t STUCK_TASK_PERIOD_MS = 100;
constexpr uint32_t ENCODER_FRESH_MS = 300;
constexpr uint32_t GYRO_FRESH_MS = 300;
constexpr uint32_t GPS_FRESH_MS = 2500;
constexpr uint32_t COORDINATE_FRESH_MS = 500;
constexpr uint32_t MAX_INTEGRATION_DT_MS = 200;
constexpr int32_t MAX_ENCODER_DELTA_MM = 1000;
constexpr float MAX_WHEEL_VELOCITY_MM_S = 5000.0f;
constexpr float MAX_GYRO_YAW_RATE_RAD_S = 20.0f;
constexpr float OBSTACLE_DISTANCE_MM = 1000.0f;
constexpr uint32_t VERIFY_SETTLE_MS = 1000;
constexpr uint32_t VERIFY_FRAME_TIMEOUT_MS = 5000;
constexpr uint32_t VERIFY_PROBE_MS = 1000;
constexpr uint32_t VERIFY_COMMAND_MS = 300;
constexpr uint32_t VERIFY_COOLDOWN_MS = 5000;
constexpr uint32_t VERIFY_TOTAL_TIMEOUT_MS = 20000;
constexpr int16_t VERIFY_TRANSLATION_MM_S = 220;
constexpr int16_t VERIFY_ROTATION_X100 = 40;
constexpr uint8_t VERIFY_HASH_SAMPLES = 3;
constexpr uint8_t VERIFY_MAX_ATTEMPTS = 2;
constexpr uint8_t HASH_UNCHANGED_MAX_BITS = 6;
constexpr uint8_t HASH_MOVED_MIN_BITS = 12;
constexpr uint8_t RECURRENCE_ESCAPE_COUNT = 3;
constexpr uint32_t RECURRENCE_WINDOW_MS = 90000;

bool isFresh(
    uint32_t now_ms,
    uint32_t timestamp_ms,
    uint32_t timeout_ms)
{
    return timestamp_ms != 0 &&
        static_cast<uint32_t>(now_ms - timestamp_ms) <= timeout_ms;
}

void sendStuckNotification()
{
    const SystemCmdType command = SystemCmdType::NotifyStuck;
    xQueueSend(fifo_system_cmd, &command, portMAX_DELAY);
}

bool registerObstacle(
    const Coordinate& coordinate,
    float commanded_velocity_mm_s,
    StuckReason reason,
    uint8_t& obstacle_cell_x,
    uint8_t& obstacle_cell_y)
{
    if (reason == StuckReason::None) return false;

    const uint16_t required_status =
        Domain::Localization::STATUS_POSITION_USABLE |
        Domain::Localization::STATUS_YAW_USABLE;
    if ((coordinate.localization_status_flags & required_status) !=
        required_status) {
        return false;
    }

    const float direction =
        commanded_velocity_mm_s >= 0.0f ? 1.0f : -1.0f;
    const float forward_x = cosf(coordinate.heading_rad);
    const float forward_y = sinf(coordinate.heading_rad);
    MapUpdate update{};
    update.world_x_mm = coordinate.x_mm;
    update.world_y_mm = coordinate.y_mm;
    update.evidence_delta = 3;
    update.radius_cells = 1;
    update.maximum_value = 14;

    switch (reason) {
        case StuckReason::WheelBlocked:
            update.world_x_mm = static_cast<int32_t>(
                coordinate.x_mm +
                direction * OBSTACLE_DISTANCE_MM * forward_x);
            update.world_y_mm = static_cast<int32_t>(
                coordinate.y_mm +
                direction * OBSTACLE_DISTANCE_MM * forward_y);
            update.evidence_delta = 5;
            update.maximum_value = AStar::CELL_BLOCKED;
            break;

        case StuckReason::RotationBlocked:
            update.evidence_delta = 3;
            break;

        case StuckReason::WheelSlip:
            update.evidence_delta = 4;
            break;
        case StuckReason::None:
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

uint8_t reasonSlot(StuckReason reason)
{
    switch (reason) {
        case StuckReason::WheelBlocked: return 0;
        case StuckReason::WheelSlip: return 1;
        case StuckReason::RotationBlocked: return 2;
        default: return UINT8_MAX;
    }
}

struct RecurrenceTracker {
    uint32_t times_ms[RECURRENCE_ESCAPE_COUNT]{};
    uint8_t count = 0;

    void expire(uint32_t now_ms)
    {
        uint8_t kept = 0;
        for (uint8_t i = 0; i < count; ++i) {
            if (static_cast<uint32_t>(now_ms - times_ms[i]) <=
                RECURRENCE_WINDOW_MS) {
                times_ms[kept++] = times_ms[i];
            }
        }
        count = kept;
    }

    void record(uint32_t now_ms)
    {
        expire(now_ms);
        if (count < RECURRENCE_ESCAPE_COUNT) {
            times_ms[count++] = now_ms;
            return;
        }
        for (uint8_t i = 1; i < RECURRENCE_ESCAPE_COUNT; ++i) {
            times_ms[i - 1U] = times_ms[i];
        }
        times_ms[RECURRENCE_ESCAPE_COUNT - 1U] = now_ms;
    }

    bool escapeRequired(uint32_t now_ms)
    {
        expire(now_ms);
        return count >= RECURRENCE_ESCAPE_COUNT;
    }

    uint8_t recentCount(uint32_t now_ms)
    {
        expire(now_ms);
        return count;
    }

    void clear()
    {
        count = 0;
    }
};

using VerifyPhase = StuckVerificationPhase;

struct VerificationContext {
    VerifyPhase phase = VerifyPhase::Idle;
    StuckReason reason = StuckReason::None;
    int8_t translation_direction = 1;
    int8_t rotation_direction = 1;
    uint32_t phase_started_ms = 0;
    uint32_t verification_started_ms = 0;
    uint32_t cooldown_until_ms = 0;
    uint8_t attempt = 0;
    uint8_t stuck_votes = 0;
    uint64_t before_hashes[VERIFY_HASH_SAMPLES]{};
    uint64_t after_hashes[VERIFY_HASH_SAMPLES]{};
    uint8_t before_count = 0;
    uint8_t after_count = 0;
    uint32_t last_camera_message = 0;
    int32_t probe_start_left_mm = 0;
    int32_t probe_start_right_mm = 0;
    int32_t probe_end_left_mm = 0;
    int32_t probe_end_right_mm = 0;
    bool have_probe_encoder = false;
    float probe_gyro_angle_rad = 0.0f;
    bool have_probe_gyro = false;
    uint64_t last_probe_gyro_timestamp_us = 0;
    float probe_start_acceleration_x_g = 0.0f;
    float probe_max_acceleration_delta_g = 0.0f;
    bool have_probe_acceleration = false;
    uint8_t result = 0;
    uint8_t hash_distance_bits = UINT8_MAX;
    uint64_t representative_before_hash = 0;
    uint64_t representative_after_hash = 0;
};

void requestSystemState(SystemCmdType command)
{
    xQueueSend(fifo_system_cmd, &command, portMAX_DELAY);
}

void publishVerificationMotion(
    int16_t velocity_mm_s,
    int16_t omega_x100)
{
    MotionCommandRequest request{};
    request.source = MotionCommandSource::StuckVerification;
    request.velocity_mm_s = velocity_mm_s;
    request.omega_rad_s_x100 = omega_x100;
    request.duration_ms = VERIFY_COMMAND_MS;
    request.timestamp_ms = millis();
    xQueueSend(fifo_motion_command_request, &request, 0);
}

uint64_t majorityHash(const uint64_t hashes[VERIFY_HASH_SAMPLES])
{
    uint64_t result = 0;
    for (uint8_t bit = 0; bit < 64U; ++bit) {
        const uint64_t mask = UINT64_C(1) << bit;
        uint8_t set_count = 0;
        for (uint8_t i = 0; i < VERIFY_HASH_SAMPLES; ++i) {
            if ((hashes[i] & mask) != 0U) ++set_count;
        }
        if (set_count >= 2U) result |= mask;
    }
    return result;
}

uint8_t hammingDistance(uint64_t first, uint64_t second)
{
    uint64_t difference = first ^ second;
    uint8_t count = 0;
    while (difference != 0U) {
        difference &= difference - 1U;
        ++count;
    }
    return count;
}

bool hashesStable(const uint64_t hashes[VERIFY_HASH_SAMPLES])
{
    return hammingDistance(hashes[0], hashes[1]) <=
               HASH_UNCHANGED_MAX_BITS &&
        hammingDistance(hashes[0], hashes[2]) <=
               HASH_UNCHANGED_MAX_BITS &&
        hammingDistance(hashes[1], hashes[2]) <=
               HASH_UNCHANGED_MAX_BITS;
}

} // namespace

void taskStuck(void *pvParameters)
{
    (void)pvParameters;

    Domain::Motion::StuckDetector detector{};
    Domain::Motion::FlipDetector flip_detector{};
    VerificationContext verification{};
    RecurrenceTracker recurrence[3]{};
    uint32_t previous_loop_ms = millis();
    uint32_t previous_gps_timestamp_ms = 0;
    bool confirmed_published = false;
    bool flip_notified = false;

    Can::Data::Encoder previous_encoder{};
    bool have_previous_encoder = false;
    float encoder_left_velocity_mm_s = 0.0f;
    float encoder_right_velocity_mm_s = 0.0f;
    uint32_t encoder_velocity_timestamp_ms = 0;

    clearPublishedStatus(previous_loop_ms);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(STUCK_TASK_PERIOD_MS);

    while (true) {
        xTaskDelayUntil(&last_wake, period);
        const uint32_t now_ms = millis();
        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
        uint32_t dt_ms = static_cast<uint32_t>(now_ms - previous_loop_ms);
        previous_loop_ms = now_ms;
        if (dt_ms > MAX_INTEGRATION_DT_MS) dt_ms = MAX_INTEGRATION_DT_MS;

        SystemData status{};
        const bool has_status =
            xQueuePeek(mbx_system_data, &status, 0) == pdTRUE;
        const bool navigation_active =
            has_status &&
            (status.state == SystemState::STATE_GPS_NAV ||
             status.state == SystemState::STATE_CAMERA_NAV) &&
            status.boot_mode != BootMode::DEBUG;
        const bool suspend_active =
            has_status &&
            status.state == SystemState::STATE_STUCK_SUSPEND;

        if (confirmed_published && navigation_active) {
            clearPublishedStatus(now_ms);
            confirmed_published = false;
        }

        JogData jog{};
        const bool jog_valid =
            xQueuePeek(mbx_can_jog_cmd, &jog, 0) == pdTRUE &&
            static_cast<uint32_t>(now_ms - jog.timestamp_ms) <
                jog.duration_ms;

        Coordinate coordinate{};
        const bool coordinate_fresh =
            xQueuePeek(mbx_coordinate, &coordinate, 0) == pdTRUE &&
            isFresh(now_ms, coordinate.timestamp_ms, COORDINATE_FRESH_MS);

        Can::Data::Encoder encoder{};
        const bool encoder_available =
            xQueuePeek(mbx_can_encoder, &encoder, 0) == pdTRUE &&
            Sensor::sampleIsFresh(
                encoder.metadata, now_us, ENCODER_FRESH_MS * 1000ULL);
        if (encoder_available) {
            if (have_previous_encoder &&
                encoder.metadata.timestamp_us !=
                    previous_encoder.metadata.timestamp_us) {
                const uint32_t encoder_dt_ms = static_cast<uint32_t>(
                    (encoder.metadata.timestamp_us -
                     previous_encoder.metadata.timestamp_us) / 1000ULL);
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
                    const float left_velocity = delta_left * scale;
                    const float right_velocity = delta_right * scale;
                    if (fabsf(left_velocity) <= MAX_WHEEL_VELOCITY_MM_S &&
                        fabsf(right_velocity) <= MAX_WHEEL_VELOCITY_MM_S) {
                        encoder_left_velocity_mm_s = left_velocity;
                        encoder_right_velocity_mm_s = right_velocity;
                        encoder_velocity_timestamp_ms = now_ms;
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
            encoder_available &&
            isFresh(
                now_ms,
                encoder_velocity_timestamp_ms,
                ENCODER_FRESH_MS) &&
            (!coordinate_fresh ||
             (coordinate.localization_status_flags &
              Domain::Localization::STATUS_ENCODER_UNHEALTHY) == 0U);

        Sensor::GyroscopeData gyroscope{};
        Sensor::AccelerometerData acceleration{};
        bool gyro_available =
            xQueuePeek(mbx_gyroscope, &gyroscope, 0) == pdTRUE &&
            Sensor::sampleIsFresh(
                gyroscope.metadata, now_us, GYRO_FRESH_MS * 1000ULL) &&
            isfinite(gyroscope.z_rad_s) &&
            fabsf(gyroscope.z_rad_s) <= MAX_GYRO_YAW_RATE_RAD_S;
        SensorAxisTransform::Vector3 body_gyroscope{};
        if (gyro_available) {
            body_gyroscope = SensorAxisTransform::imuToBody(
                gyroscope.metadata.source,
                gyroscope.x_rad_s,
                gyroscope.y_rad_s,
                gyroscope.z_rad_s);
            gyro_available = isfinite(body_gyroscope.z) &&
                fabsf(body_gyroscope.z) <= MAX_GYRO_YAW_RATE_RAD_S;
        }

        const auto drainGyroscopeRing = [&](
            QueueHandle_t ring, Sensor::Source source) {
            Sensor::GyroscopeData sample{};
            while (xQueueReceive(ring, &sample, 0) == pdTRUE) {
                if (verification.phase != VerifyPhase::Probe ||
                    gyroscope.metadata.source != source ||
                    sample.metadata.received_us <
                        static_cast<uint64_t>(verification.phase_started_ms) *
                            1000ULL) {
                    continue;
                }
                const auto body = SensorAxisTransform::imuToBody(
                    source,
                    sample.x_rad_s,
                    sample.y_rad_s,
                    sample.z_rad_s);
                if (!isfinite(body.z) ||
                    fabsf(body.z) > MAX_GYRO_YAW_RATE_RAD_S) {
                    continue;
                }
                if (verification.last_probe_gyro_timestamp_us != 0U &&
                    sample.metadata.timestamp_us >
                        verification.last_probe_gyro_timestamp_us) {
                    const uint64_t interval_us =
                        sample.metadata.timestamp_us -
                        verification.last_probe_gyro_timestamp_us;
                    if (interval_us <= 100000ULL) {
                        verification.probe_gyro_angle_rad +=
                            body.z * static_cast<float>(interval_us) * 1e-6f;
                        verification.have_probe_gyro = true;
                    }
                }
                verification.last_probe_gyro_timestamp_us =
                    sample.metadata.timestamp_us;
            }
        };
        drainGyroscopeRing(
            fifo_stuck_board_gyroscope, Sensor::Source::BoardI2c);
        drainGyroscopeRing(fifo_stuck_can_gyroscope, Sensor::Source::Can);

        const bool acceleration_available =
            xQueuePeek(mbx_acceleration, &acceleration, 0) == pdTRUE &&
            Sensor::sampleIsFresh(
                acceleration.metadata, now_us, GYRO_FRESH_MS * 1000ULL) &&
            isfinite(acceleration.x_m_s2) &&
            isfinite(acceleration.y_m_s2) &&
            isfinite(acceleration.z_m_s2);
        SensorAxisTransform::Vector3 body_acceleration{};
        Sensor::AccelerometerData body_acceleration_sample{};
        if (acceleration_available) {
            body_acceleration = SensorAxisTransform::imuToBody(
                acceleration.metadata.source,
                acceleration.x_m_s2 / 9.80665f,
                acceleration.y_m_s2 / 9.80665f,
                acceleration.z_m_s2 / 9.80665f);
            body_acceleration_sample.x_m_s2 =
                body_acceleration.x * 9.80665f;
            body_acceleration_sample.y_m_s2 =
                body_acceleration.y * 9.80665f;
            body_acceleration_sample.z_m_s2 =
                body_acceleration.z * 9.80665f;
            body_acceleration_sample.metadata = acceleration.metadata;
        }
        const Domain::Motion::FlipResult flip = flip_detector.update(
            now_us,
            body_acceleration_sample);
        xQueueOverwrite(mbx_flip_status, &flip);
        const bool flip_detected =
            flip.state == Domain::Motion::FlipState::Flipped;
        if (flip_detected && (navigation_active || suspend_active) &&
            !flip_notified) {
            const SystemCmdType command = SystemCmdType::NotifyFlipped;
            xQueueSend(fifo_system_cmd, &command, portMAX_DELAY);
            flip_notified = true;
        } else if (flip.state == Domain::Motion::FlipState::Upright &&
                   navigation_active) {
            flip_notified = false;
        }

        Domain::Localization::GpsObservation gps_data{};
        const bool gps_available =
            xQueuePeek(mbx_gps_local_observation, &gps_data, 0) == pdTRUE &&
            gps_data.position_valid &&
            gps_data.fix_type >= 3U &&
            gps_data.horizontal_accuracy_m <= 3.0f &&
            Sensor::sampleIsFresh(
                gps_data.metadata, now_us, GPS_FRESH_MS * 1000ULL);
        const bool gps_updated =
            gps_available &&
            static_cast<uint32_t>(gps_data.metadata.timestamp_us / 1000ULL) !=
                previous_gps_timestamp_ms;
        if (gps_updated) previous_gps_timestamp_ms = static_cast<uint32_t>(
            gps_data.metadata.timestamp_us / 1000ULL);

        const Domain::Motion::DetectorDiagnostics detector_diagnostics =
            detector.diagnostics(now_ms);
        StuckDiagnostics diagnostics{};
        diagnostics.timestamp_ms = now_ms;
        diagnostics.scores = detector.scores();
        diagnostics.verification_phase = verification.phase;
        diagnostics.trigger_reason = verification.reason;
        for (uint8_t i = 0; i < 3U; ++i) {
            diagnostics.recurrence_count[i] =
                recurrence[i].recentCount(now_ms);
        }
        diagnostics.verification_attempt = verification.attempt;
        diagnostics.verification_stuck_votes = verification.stuck_votes;
        diagnostics.verification_result = verification.result;
        diagnostics.hash_distance_bits = verification.hash_distance_bits;
        diagnostics.hash_before = verification.representative_before_hash;
        diagnostics.hash_after = verification.representative_after_hash;
        diagnostics.probe_left_delta_mm = static_cast<int16_t>(constrain(
            verification.probe_end_left_mm -
                verification.probe_start_left_mm,
            -32768L,
            32767L));
        diagnostics.probe_right_delta_mm = static_cast<int16_t>(constrain(
            verification.probe_end_right_mm -
                verification.probe_start_right_mm,
            -32768L,
            32767L));
        diagnostics.probe_gyro_angle_mrad = static_cast<int16_t>(constrain(
            lroundf(verification.probe_gyro_angle_rad * 1000.0f),
            -32768L,
            32767L));
        diagnostics.gps_window_age_ms =
            detector_diagnostics.gps_window_age_ms;
        diagnostics.gps_displacement_mm =
            detector_diagnostics.gps_displacement_mm;
        diagnostics.gps_encoder_distance_mm =
            detector_diagnostics.gps_encoder_distance_mm;
        diagnostics.gps_sample_count =
            detector_diagnostics.gps_sample_count;
        diagnostics.gps_speed_mismatch_count =
            detector_diagnostics.gps_speed_mismatch_count;
        diagnostics.wheel_blocked_active_ms =
            detector_diagnostics.wheel_blocked_active_ms;
        diagnostics.wheel_slip_active_ms =
            detector_diagnostics.wheel_slip_active_ms;
        diagnostics.rotation_blocked_active_ms =
            detector_diagnostics.rotation_blocked_active_ms;
        diagnostics.encoder_left_velocity_mm_s = static_cast<int16_t>(
            constrain(lroundf(encoder_left_velocity_mm_s), -32768L, 32767L));
        diagnostics.encoder_right_velocity_mm_s = static_cast<int16_t>(
            constrain(lroundf(encoder_right_velocity_mm_s), -32768L, 32767L));
        xQueueOverwrite(mbx_stuck_diagnostics, &diagnostics);

        auto publishConfirmed = [&](StuckReason reason) {
            uint8_t obstacle_cell_x = UINT8_MAX;
            uint8_t obstacle_cell_y = UINT8_MAX;
            if (coordinate_fresh) {
                registerObstacle(
                    coordinate,
                    static_cast<float>(verification.translation_direction) *
                        VERIFY_TRANSLATION_MM_S,
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
            sendStuckNotification();
            confirmed_published = true;
            verification.result = 1;
            verification.phase = VerifyPhase::Idle;
        };

        auto rejectVerification = [&](bool movement_confirmed) {
            const uint8_t slot = reasonSlot(verification.reason);
            if (movement_confirmed && slot < 3U) recurrence[slot].clear();
            detector.completeVerification(
                verification.reason,
                movement_confirmed);
            requestSystemState(SystemCmdType::StuckVerificationRejected);
            clearPublishedStatus(now_ms);
            verification.cooldown_until_ms = now_ms + VERIFY_COOLDOWN_MS;
            verification.result = movement_confirmed ? 2U : 3U;
            verification.phase = VerifyPhase::Idle;
        };

        auto finishInconclusive = [&]() {
            const uint8_t slot = reasonSlot(verification.reason);
            const bool no_independent_evidence =
                !encoder_available && !gyro_available &&
                !acceleration_available && !gps_available;
            if (no_independent_evidence ||
                (verification.reason == StuckReason::WheelSlip &&
                 slot < 3U && recurrence[slot].escapeRequired(now_ms))) {
                publishConfirmed(verification.reason);
            } else {
                rejectVerification(false);
            }
        };

        auto startNextAttempt = [&]() {
            ++verification.attempt;
            verification.before_count = 0;
            verification.after_count = 0;
            verification.last_camera_message = 0;
            verification.have_probe_encoder = false;
            verification.probe_gyro_angle_rad = 0.0f;
            verification.have_probe_gyro = false;
            verification.last_probe_gyro_timestamp_us = 0;
            verification.have_probe_acceleration = false;
            verification.probe_max_acceleration_delta_g = 0.0f;
            verification.phase = VerifyPhase::SettleBefore;
            verification.phase_started_ms = now_ms;
            publishVerificationMotion(0, 0);
        };

        if (verification.phase != VerifyPhase::Idle) {
            if (verification.verification_started_ms != 0U &&
                static_cast<uint32_t>(
                    now_ms - verification.verification_started_ms) >=
                    VERIFY_TOTAL_TIMEOUT_MS) {
                publishVerificationMotion(0, 0);
                publishConfirmed(verification.reason);
                continue;
            }
            if (verification.phase == VerifyPhase::WaitSuspend) {
                if (suspend_active) {
                    verification.phase = VerifyPhase::SettleBefore;
                    verification.phase_started_ms = now_ms;
                    verification.attempt = 0;
                    verification.stuck_votes = 0;
                    verification.before_count = 0;
                    verification.after_count = 0;
                    verification.last_probe_gyro_timestamp_us = 0;
                    verification.have_probe_acceleration = false;
                    publishVerificationMotion(0, 0);
                } else if (static_cast<uint32_t>(
                        now_ms - verification.phase_started_ms) > 2000U) {
                    detector.completeVerification(
                        verification.reason,
                        false);
                    verification.phase = VerifyPhase::Idle;
                }
                continue;
            }

            if (!suspend_active) {
                verification.phase = VerifyPhase::Idle;
                continue;
            }

            if (flip_detected) {
                verification.phase = VerifyPhase::Idle;
                continue;
            }

            switch (verification.phase) {
                case VerifyPhase::SettleBefore:
                    publishVerificationMotion(0, 0);
                    if (static_cast<uint32_t>(
                            now_ms - verification.phase_started_ms) >=
                        VERIFY_SETTLE_MS) {
                        if (verification.reason == StuckReason::WheelSlip) {
                            verification.before_count = 0;
                            verification.last_camera_message = 0;
                            verification.phase = VerifyPhase::CaptureBefore;
                        } else {
                            verification.have_probe_encoder =
                                encoder_available;
                            if (encoder_available) {
                                verification.probe_start_left_mm =
                                    encoder.left_mm;
                                verification.probe_start_right_mm =
                                    encoder.right_mm;
                            }
                            verification.probe_gyro_angle_rad = 0.0f;
                            verification.have_probe_gyro = false;
                            verification.last_probe_gyro_timestamp_us = 0;
                            verification.have_probe_acceleration =
                                acceleration_available;
                            verification.probe_start_acceleration_x_g =
                                body_acceleration.x;
                            verification.probe_max_acceleration_delta_g = 0.0f;
                            verification.phase = VerifyPhase::Probe;
                        }
                        verification.phase_started_ms = now_ms;
                    }
                    break;

                case VerifyPhase::CaptureBefore:
                case VerifyPhase::CaptureAfter: {
                    if (verification.reason != StuckReason::WheelSlip) {
                        rejectVerification(false);
                        break;
                    }
                    publishVerificationMotion(0, 0);
                    Rasp::CameraData camera{};
                    const bool fresh_frame =
                        xQueuePeek(mbx_camera_data, &camera, 0) == pdTRUE &&
                        camera.received_ms >= verification.phase_started_ms &&
                        camera.frame.msg_number !=
                            verification.last_camera_message;
                    if (fresh_frame) {
                        verification.last_camera_message =
                            camera.frame.msg_number;
                        if (verification.phase ==
                            VerifyPhase::CaptureBefore) {
                            if (verification.before_count <
                                VERIFY_HASH_SAMPLES) {
                                verification.before_hashes[
                                    verification.before_count++] =
                                    camera.frame.scene_hash;
                            }
                        } else if (verification.after_count <
                                   VERIFY_HASH_SAMPLES) {
                            verification.after_hashes[
                                verification.after_count++] =
                                camera.frame.scene_hash;
                        }
                    }

                    if (verification.phase == VerifyPhase::CaptureBefore &&
                        verification.before_count >= VERIFY_HASH_SAMPLES) {
                        verification.have_probe_encoder = encoder_available;
                        if (encoder_available) {
                            verification.probe_start_left_mm = encoder.left_mm;
                            verification.probe_start_right_mm = encoder.right_mm;
                        }
                        verification.probe_gyro_angle_rad = 0.0f;
                        verification.have_probe_gyro = false;
                        verification.last_probe_gyro_timestamp_us = 0;
                        verification.have_probe_acceleration =
                            acceleration_available;
                        verification.probe_start_acceleration_x_g =
                            body_acceleration.x;
                        verification.probe_max_acceleration_delta_g = 0.0f;
                        verification.phase = VerifyPhase::Probe;
                        verification.phase_started_ms = now_ms;
                        break;
                    }

                    if (verification.phase == VerifyPhase::CaptureAfter &&
                        verification.after_count >= VERIFY_HASH_SAMPLES) {
                        const uint64_t before = majorityHash(
                            verification.before_hashes);
                        const uint64_t after = majorityHash(
                            verification.after_hashes);
                        const uint8_t distance =
                            hammingDistance(before, after);
                        verification.representative_before_hash = before;
                        verification.representative_after_hash = after;
                        verification.hash_distance_bits = distance;
                        const bool visual_measurement_valid =
                            hashesStable(verification.before_hashes) &&
                            hashesStable(verification.after_hashes) &&
                            (!verification.have_probe_gyro ||
                             fabsf(verification.probe_gyro_angle_rad) < 0.10f);
                        if (visual_measurement_valid &&
                            distance >= HASH_MOVED_MIN_BITS) {
                            rejectVerification(true);
                            break;
                        }

                        const float translation_distance = fabsf(0.5f *
                            static_cast<float>(
                                (verification.probe_end_left_mm -
                                 verification.probe_start_left_mm) +
                                (verification.probe_end_right_mm -
                                 verification.probe_start_right_mm)));
                        const bool specific_evidence =
                            verification.have_probe_encoder &&
                            translation_distance >= 132.0f;
                        const bool acceleration_still =
                            verification.have_probe_acceleration &&
                            verification.probe_max_acceleration_delta_g <
                                0.08f;
                        const float gps_speed_mm_s = sqrtf(
                            gps_data.velocity_east_m_s *
                                gps_data.velocity_east_m_s +
                            gps_data.velocity_north_m_s *
                                gps_data.velocity_north_m_s) * 1000.0f;
                        const bool gps_still = gps_available &&
                            gps_data.velocity_valid &&
                            gps_data.speed_accuracy_m_s <= 0.5f &&
                            gps_speed_mm_s +
                                2.0f * gps_data.speed_accuracy_m_s *
                                    1000.0f <= 100.0f;

                        if (visual_measurement_valid &&
                            distance <= HASH_UNCHANGED_MAX_BITS &&
                            specific_evidence) {
                            if (gps_still || acceleration_still) {
                                publishConfirmed(verification.reason);
                                break;
                            }
                            ++verification.stuck_votes;
                            if (verification.stuck_votes >= 2U) {
                                publishConfirmed(verification.reason);
                            } else {
                                startNextAttempt();
                            }
                        } else if (verification.attempt + 1U >=
                                   VERIFY_MAX_ATTEMPTS) {
                            finishInconclusive();
                        } else {
                            startNextAttempt();
                        }
                        break;
                    }

                    if (static_cast<uint32_t>(
                            now_ms - verification.phase_started_ms) >=
                        VERIFY_FRAME_TIMEOUT_MS) {
                        if (verification.attempt + 1U >=
                            VERIFY_MAX_ATTEMPTS) {
                            finishInconclusive();
                        } else {
                            startNextAttempt();
                        }
                    }
                    break;
                }

                case VerifyPhase::Probe: {
                    if (verification.reason ==
                        StuckReason::RotationBlocked) {
                        publishVerificationMotion(
                            0,
                            static_cast<int16_t>(
                                verification.rotation_direction *
                                VERIFY_ROTATION_X100));
                    } else {
                        publishVerificationMotion(
                            static_cast<int16_t>(
                                verification.translation_direction *
                                VERIFY_TRANSLATION_MM_S),
                            0);
                    }
                    if (acceleration_available &&
                        verification.have_probe_acceleration) {
                        const float delta = fabsf(
                            body_acceleration.x -
                            verification.probe_start_acceleration_x_g);
                        if (delta >
                            verification.probe_max_acceleration_delta_g) {
                            verification.probe_max_acceleration_delta_g = delta;
                        }
                    }
                    if (static_cast<uint32_t>(
                            now_ms - verification.phase_started_ms) >=
                        VERIFY_PROBE_MS) {
                        if (encoder_available) {
                            verification.probe_end_left_mm = encoder.left_mm;
                            verification.probe_end_right_mm = encoder.right_mm;
                        } else {
                            verification.have_probe_encoder = false;
                        }
                        publishVerificationMotion(0, 0);
                        if (verification.reason == StuckReason::WheelSlip) {
                            verification.phase = VerifyPhase::SettleAfter;
                            verification.phase_started_ms = now_ms;
                            break;
                        }

                        const float left_distance = fabsf(
                            static_cast<float>(
                                verification.probe_end_left_mm -
                                verification.probe_start_left_mm));
                        const float right_distance = fabsf(
                            static_cast<float>(
                                verification.probe_end_right_mm -
                                verification.probe_start_right_mm));
                        const float encoder_turn_rad = fabsf(
                            static_cast<float>(
                                (verification.probe_end_right_mm -
                                 verification.probe_start_right_mm) -
                                (verification.probe_end_left_mm -
                                 verification.probe_start_left_mm)) /
                            180.0f);
                        bool confirmed = false;
                        bool movement_confirmed = false;
                        switch (verification.reason) {
                            case StuckReason::WheelBlocked:
                                if (verification.have_probe_encoder) {
                                    confirmed =
                                        fminf(left_distance, right_distance) <
                                        44.0f;
                                    movement_confirmed = !confirmed;
                                }
                                break;

                            case StuckReason::RotationBlocked:
                                if (verification.have_probe_encoder &&
                                    verification.have_probe_gyro) {
                                    const bool wheels_locked =
                                        fmaxf(left_distance, right_distance) <
                                            44.0f;
                                    const bool body_not_rotating =
                                        encoder_turn_rad >= 0.24f &&
                                        fabsf(
                                            verification.probe_gyro_angle_rad) <
                                            0.08f;
                                    confirmed = wheels_locked ||
                                        body_not_rotating;
                                    movement_confirmed =
                                        fabsf(
                                            verification.probe_gyro_angle_rad) >=
                                        0.12f;
                                } else if (verification.have_probe_encoder) {
                                    confirmed =
                                        fmaxf(left_distance, right_distance) <
                                        44.0f;
                                }
                                break;

                            default:
                                break;
                        }
                        if (confirmed) {
                            publishConfirmed(verification.reason);
                        } else {
                            rejectVerification(movement_confirmed);
                        }
                    }
                    break;
                }

                case VerifyPhase::SettleAfter:
                    publishVerificationMotion(0, 0);
                    if (static_cast<uint32_t>(
                            now_ms - verification.phase_started_ms) >=
                        VERIFY_SETTLE_MS) {
                        verification.after_count = 0;
                        verification.last_camera_message = 0;
                        verification.phase = VerifyPhase::CaptureAfter;
                        verification.phase_started_ms = now_ms;
                    }
                    break;

                case VerifyPhase::Idle:
                case VerifyPhase::WaitSuspend:
                    break;
            }
            continue;
        }

        if (!navigation_active) {
            detector.reset(now_ms);
            have_previous_encoder = false;
            encoder_velocity_timestamp_ms = 0;
            continue;
        }

        if (flip_detected) {
            continue;
        }

        Domain::Motion::DetectorSample sample{};
        sample.timestamp_ms = now_ms;
        sample.dt_ms = dt_ms;
        sample.navigation_active = true;
        sample.command_valid = jog_valid;
        sample.command_velocity_mm_s =
            jog_valid ? jog.velocity_mm_s : 0.0f;
        sample.command_yaw_rate_rad_s =
            jog_valid ? jog.omega_rad_s : 0.0f;
        sample.encoder_available = encoder_velocity_available;
        sample.encoder_left_mm = encoder.left_mm;
        sample.encoder_right_mm = encoder.right_mm;
        sample.encoder_left_velocity_mm_s = encoder_left_velocity_mm_s;
        sample.encoder_right_velocity_mm_s = encoder_right_velocity_mm_s;
        sample.gyro_available = gyro_available;
        sample.gyro_yaw_rate_rad_s =
            gyro_available ? body_gyroscope.z : 0.0f;
        sample.acceleration_available = acceleration_available;
        sample.acceleration_x_g = body_acceleration.x;
        sample.acceleration_y_g = body_acceleration.y;
        sample.acceleration_z_g = body_acceleration.z;
        sample.gps_available = gps_available;
        sample.gps_updated = gps_updated;
        sample.gps_horizontal_accuracy_mm =
            static_cast<uint32_t>(lroundf(
                gps_data.horizontal_accuracy_m * 1000.0f));
        sample.gps_x_mm = static_cast<int32_t>(lroundf(gps_data.east_m * 1000.0f));
        sample.gps_y_mm = static_cast<int32_t>(lroundf(gps_data.north_m * 1000.0f));
        sample.gps_velocity_available = gps_available &&
            gps_data.velocity_valid &&
            isfinite(gps_data.velocity_east_m_s) &&
            isfinite(gps_data.velocity_north_m_s) &&
            isfinite(gps_data.speed_accuracy_m_s);
        sample.gps_velocity_east_mm_s =
            gps_data.velocity_east_m_s * 1000.0f;
        sample.gps_velocity_north_mm_s =
            gps_data.velocity_north_m_s * 1000.0f;
        sample.gps_speed_accuracy_mm_s =
            gps_data.speed_accuracy_m_s * 1000.0f;

        const Domain::Motion::Assessment assessment =
            detector.update(sample);
        if (!assessment.suspend_requested ||
            static_cast<int32_t>(
                now_ms - verification.cooldown_until_ms) < 0) {
            continue;
        }

        const uint8_t slot = reasonSlot(assessment.reason);
        if (slot >= 3U) continue;
        if (assessment.reason == StuckReason::WheelSlip) {
            recurrence[slot].record(now_ms);
        }
        verification.reason = assessment.reason;
        verification.result = 0;
        verification.hash_distance_bits = UINT8_MAX;
        verification.representative_before_hash = 0;
        verification.representative_after_hash = 0;
        verification.probe_start_left_mm = 0;
        verification.probe_start_right_mm = 0;
        verification.probe_end_left_mm = 0;
        verification.probe_end_right_mm = 0;
        verification.translation_direction =
            sample.command_velocity_mm_s < 0.0f ? -1 : 1;
        verification.rotation_direction =
            sample.command_yaw_rate_rad_s < 0.0f ? -1 : 1;
        verification.phase = VerifyPhase::WaitSuspend;
        verification.phase_started_ms = now_ms;
        verification.verification_started_ms = now_ms;
        publishVerificationMotion(0, 0);
        requestSystemState(SystemCmdType::RequestStuckSuspend);
    }
}
