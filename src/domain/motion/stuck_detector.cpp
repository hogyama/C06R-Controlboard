#include "stuck_detector.h"

#include <math.h>
#include <string.h>

namespace Domain::Motion {

namespace {

uint32_t saturatedAdd(uint32_t value, uint32_t increment)
{
    return increment > UINT32_MAX - value ? UINT32_MAX : value + increment;
}

uint16_t saturatedU16(float value)
{
    if (!isfinite(value) || value <= 0.0f) return 0;
    return value >= 65535.0f
        ? UINT16_MAX
        : static_cast<uint16_t>(lroundf(value));
}

int32_t median3(int32_t a, int32_t b, int32_t c)
{
    if (a > b) { const int32_t t = a; a = b; b = t; }
    if (b > c) { const int32_t t = b; b = c; c = t; }
    if (a > b) { const int32_t t = a; a = b; b = t; }
    return b;
}

} // namespace

StuckDetector::StuckDetector(const DetectorConfig& config)
    : config_(config)
{
    reset();
}

void StuckDetector::reset(uint32_t timestamp_ms)
{
    command_active_ms_ = 0;
    last_motion_command_ms_ = timestamp_ms;
    wheel_blocked_active_ms_ = 0;
    direction_mismatch_active_ms_ = 0;
    rotation_blocked_active_ms_ = 0;
    slip_gyro_mismatch_active_ms_ = 0;
    acceleration_impact_until_ms_ = 0;
    previous_acceleration_x_g_ = 0.0f;
    have_previous_acceleration_ = false;
    scores_ = {};
    suspend_latched_ = false;
    resetSlipTracking();
}

float StuckDetector::absolute(float value)
{
    return value < 0.0f ? -value : value;
}

uint32_t StuckDetector::updateTimer(
    uint32_t value,
    bool evidence,
    bool healthy,
    uint32_t dt_ms)
{
    if (evidence) return saturatedAdd(value, dt_ms);
    if (healthy) return dt_ms >= value ? 0U : value - dt_ms;
    const uint32_t decay = (dt_ms + 3U) / 4U;
    return decay >= value ? 0U : value - decay;
}

uint16_t StuckDetector::timerScore(uint32_t value, uint32_t threshold_ms)
{
    if (threshold_ms == 0U || value >= threshold_ms) return 1000U;
    return static_cast<uint16_t>(
        (static_cast<uint64_t>(value) * 1000ULL) / threshold_ms);
}

void StuckDetector::resetSlipTracking()
{
    slip_no_ground_active_ms_ = 0;
    slip_no_ground_encoder_mm_ = 0.0f;
    gps_speed_mismatch_count_ = 0;
    gps_point_count_ = 0;
    gps_displacement_mm_ = 0.0f;
    gps_encoder_distance_mm_ = 0.0f;
    memset(gps_points_, 0, sizeof(gps_points_));
}

void StuckDetector::pushGpsPoint(const DetectorSample& sample)
{
    GpsPoint point{};
    point.timestamp_ms = sample.timestamp_ms;
    point.x_mm = sample.gps_x_mm;
    point.y_mm = sample.gps_y_mm;
    point.encoder_mean_mm = static_cast<int32_t>(
        (static_cast<int64_t>(sample.encoder_left_mm) +
         static_cast<int64_t>(sample.encoder_right_mm)) / 2LL);

    if (gps_point_count_ < GPS_POINT_CAPACITY) {
        gps_points_[gps_point_count_++] = point;
    } else {
        for (uint8_t i = 1; i < GPS_POINT_CAPACITY; ++i) {
            gps_points_[i - 1U] = gps_points_[i];
        }
        gps_points_[GPS_POINT_CAPACITY - 1U] = point;
    }

    if (gps_point_count_ < 3U) return;
    const uint8_t last = gps_point_count_ - 1U;
    const int32_t start_x = median3(
        gps_points_[0].x_mm, gps_points_[1].x_mm, gps_points_[2].x_mm);
    const int32_t start_y = median3(
        gps_points_[0].y_mm, gps_points_[1].y_mm, gps_points_[2].y_mm);
    const int32_t end_x = median3(
        gps_points_[last - 2U].x_mm,
        gps_points_[last - 1U].x_mm,
        gps_points_[last].x_mm);
    const int32_t end_y = median3(
        gps_points_[last - 2U].y_mm,
        gps_points_[last - 1U].y_mm,
        gps_points_[last].y_mm);
    const float dx = static_cast<float>(end_x - start_x);
    const float dy = static_cast<float>(end_y - start_y);
    gps_displacement_mm_ = sqrtf(dx * dx + dy * dy);
    gps_encoder_distance_mm_ = absolute(static_cast<float>(
        point.encoder_mean_mm - gps_points_[0].encoder_mean_mm));
}

bool StuckDetector::gpsDistanceMismatch(bool& healthy) const
{
    healthy = false;
    if (gps_point_count_ < config_.gps_minimum_samples) return false;
    const uint8_t last = gps_point_count_ - 1U;
    if (static_cast<uint32_t>(
            gps_points_[last].timestamp_ms - gps_points_[0].timestamp_ms) <
            config_.gps_window_ms ||
        gps_encoder_distance_mm_ < config_.slip_minimum_encoder_mm) {
        return false;
    }
    const float ratio = gps_displacement_mm_ /
        fmaxf(gps_encoder_distance_mm_, 1.0f);
    healthy = ratio >= config_.healthy_distance_ratio;
    return ratio <= config_.slip_distance_ratio;
}

StuckScores StuckDetector::scores() const
{
    return scores_;
}

DetectorDiagnostics StuckDetector::diagnostics(uint32_t timestamp_ms) const
{
    DetectorDiagnostics result{};
    result.gps_window_age_ms = gps_point_count_ > 0U
        ? saturatedU16(static_cast<float>(static_cast<uint32_t>(
            timestamp_ms - gps_points_[0].timestamp_ms)))
        : saturatedU16(static_cast<float>(slip_no_ground_active_ms_));
    result.gps_displacement_mm = saturatedU16(gps_displacement_mm_);
    result.gps_encoder_distance_mm = saturatedU16(
        gps_point_count_ > 0U
            ? gps_encoder_distance_mm_
            : slip_no_ground_encoder_mm_);
    result.gps_sample_count = gps_point_count_;
    result.gps_speed_mismatch_count = gps_speed_mismatch_count_;
    result.wheel_blocked_active_ms = saturatedU16(
        static_cast<float>(wheel_blocked_active_ms_));
    result.wheel_slip_active_ms = saturatedU16(
        static_cast<float>(slip_no_ground_active_ms_));
    result.rotation_blocked_active_ms = saturatedU16(
        static_cast<float>(rotation_blocked_active_ms_));
    return result;
}

void StuckDetector::completeVerification(Reason reason, bool movement_confirmed)
{
    if (reason == Reason::WheelBlocked) {
        wheel_blocked_active_ms_ = movement_confirmed
            ? 0U : config_.wheel_blocked_arm_ms / 4U;
        direction_mismatch_active_ms_ = 0;
        scores_.wheel_blocked = movement_confirmed ? 0U : 250U;
    } else if (reason == Reason::WheelSlip) {
        resetSlipTracking();
        slip_gyro_mismatch_active_ms_ = 0;
        scores_.wheel_slip = 0U;
    } else if (reason == Reason::RotationBlocked) {
        rotation_blocked_active_ms_ = movement_confirmed
            ? 0U : config_.rotation_blocked_arm_ms / 4U;
        scores_.rotation_blocked = movement_confirmed ? 0U : 250U;
    }
    suspend_latched_ = false;
}

Assessment StuckDetector::update(const DetectorSample& sample)
{
    Assessment result{};
    result.timestamp_ms = sample.timestamp_ms;
    result.condition = Condition::Normal;
    result.reason = Reason::None;
    result.encoder_trusted = sample.encoder_available;
    result.gps_trusted = sample.gps_available;
    result.gyro_trusted = sample.gyro_available;

    const uint32_t dt_ms = sample.dt_ms > 200U ? 200U : sample.dt_ms;
    const float dt_s = static_cast<float>(dt_ms) * 0.001f;
    const bool translation_commanded = sample.command_valid &&
        absolute(sample.command_velocity_mm_s) >=
            config_.minimum_translation_command_mm_s;
    const bool rotation_commanded = sample.command_valid &&
        absolute(sample.command_yaw_rate_rad_s) >=
            config_.minimum_rotation_command_rad_s;
    const bool motion_commanded = translation_commanded || rotation_commanded;

    if (!sample.navigation_active) {
        reset(sample.timestamp_ms);
        result.scores = scores_;
        return result;
    }

    if (motion_commanded) {
        if (last_motion_command_ms_ != 0U &&
            static_cast<uint32_t>(sample.timestamp_ms - last_motion_command_ms_) >
                config_.command_gap_reset_ms) {
            command_active_ms_ = 0;
            resetSlipTracking();
        }
        last_motion_command_ms_ = sample.timestamp_ms;
        command_active_ms_ = saturatedAdd(command_active_ms_, dt_ms);
    } else {
        command_active_ms_ = 0;
        wheel_blocked_active_ms_ = 0;
        direction_mismatch_active_ms_ = 0;
        rotation_blocked_active_ms_ = 0;
        slip_gyro_mismatch_active_ms_ = 0;
        resetSlipTracking();
    }
    const bool armed = command_active_ms_ >= config_.command_arm_ms;

    const float left_command = sample.command_velocity_mm_s -
        sample.command_yaw_rate_rad_s * config_.half_track_mm;
    const float right_command = sample.command_velocity_mm_s +
        sample.command_yaw_rate_rad_s * config_.half_track_mm;
    const float rotation_equivalent =
        absolute(sample.command_yaw_rate_rad_s) * config_.half_track_mm;
    const bool translation_dominant = translation_commanded &&
        absolute(sample.command_velocity_mm_s) > rotation_equivalent;
    const bool rotation_dominant = rotation_commanded &&
        rotation_equivalent >= absolute(sample.command_velocity_mm_s);

    const auto wheelState = [&](float command, float velocity,
                                bool& stopped, bool& moving,
                                bool& reversed) {
        const bool commanded = absolute(command) >=
            config_.minimum_wheel_command_mm_s;
        const float stopped_threshold = fmaxf(
            config_.stopped_velocity_mm_s,
            absolute(command) * config_.stopped_ratio);
        const float moving_threshold = fmaxf(
            config_.moving_velocity_mm_s,
            absolute(command) * config_.moving_ratio);
        stopped = commanded && absolute(velocity) < stopped_threshold;
        moving = commanded && velocity * command > 0.0f &&
            absolute(velocity) >= moving_threshold;
        reversed = commanded && velocity * command < 0.0f &&
            absolute(velocity) >= config_.moving_velocity_mm_s;
    };

    bool left_stopped = false, left_moving = false, left_reversed = false;
    bool right_stopped = false, right_moving = false, right_reversed = false;
    if (sample.encoder_available) {
        wheelState(left_command, sample.encoder_left_velocity_mm_s,
                   left_stopped, left_moving, left_reversed);
        wheelState(right_command, sample.encoder_right_velocity_mm_s,
                   right_stopped, right_moving, right_reversed);
        result.evidence_flags |= (left_stopped || right_stopped)
            ? EVIDENCE_ENCODER_NOT_MOVING
            : EVIDENCE_ENCODER_MOVING;
        if (left_stopped != right_stopped) {
            result.evidence_flags |= EVIDENCE_LEFT_RIGHT_MISMATCH;
        }
        if (left_reversed || right_reversed) {
            result.evidence_flags |= EVIDENCE_DIRECTION_MISMATCH;
        }
    } else {
        result.evidence_flags |= EVIDENCE_ENCODER_UNAVAILABLE;
    }

    const bool wheel_blocked_evidence = armed && translation_dominant &&
        sample.encoder_available && (left_stopped || right_stopped);
    const bool wheel_direction_mismatch = armed && translation_dominant &&
        sample.encoder_available && (left_reversed || right_reversed);
    const bool wheel_blocked_healthy = translation_dominant &&
        left_moving && right_moving;
    wheel_blocked_active_ms_ = updateTimer(
        wheel_blocked_active_ms_, wheel_blocked_evidence,
        wheel_blocked_healthy, dt_ms);
    direction_mismatch_active_ms_ = updateTimer(
        direction_mismatch_active_ms_, wheel_direction_mismatch,
        wheel_blocked_healthy, dt_ms);

    const float encoder_velocity = 0.5f *
        (sample.encoder_left_velocity_mm_s +
         sample.encoder_right_velocity_mm_s);
    const float encoder_yaw_rate =
        (sample.encoder_right_velocity_mm_s -
         sample.encoder_left_velocity_mm_s) /
        (2.0f * config_.half_track_mm);
    const float encoder_delta_mm = absolute(encoder_velocity) * dt_s;

    const float stopped_yaw_threshold = fmaxf(
        config_.stopped_yaw_rate_rad_s,
        absolute(sample.command_yaw_rate_rad_s) * config_.stopped_ratio);
    const float moving_yaw_threshold = fmaxf(
        config_.stopped_yaw_rate_rad_s + 0.02f,
        absolute(sample.command_yaw_rate_rad_s) * config_.moving_ratio);
    const bool gyro_stopped = sample.gyro_available &&
        absolute(sample.gyro_yaw_rate_rad_s) < stopped_yaw_threshold;
    const bool gyro_moving = sample.gyro_available &&
        sample.gyro_yaw_rate_rad_s * sample.command_yaw_rate_rad_s > 0.0f &&
        absolute(sample.gyro_yaw_rate_rad_s) >= moving_yaw_threshold;
    const bool gyro_reversed = sample.gyro_available &&
        sample.gyro_yaw_rate_rad_s * sample.command_yaw_rate_rad_s < 0.0f &&
        absolute(sample.gyro_yaw_rate_rad_s) >=
            config_.stopped_yaw_rate_rad_s;
    if (sample.gyro_available) {
        result.evidence_flags |= gyro_stopped
            ? EVIDENCE_GYRO_NOT_ROTATING
            : EVIDENCE_GYRO_ROTATING;
        if (gyro_reversed) result.evidence_flags |= EVIDENCE_DIRECTION_MISMATCH;
    } else {
        result.evidence_flags |= EVIDENCE_GYRO_UNAVAILABLE;
    }

    const float minimum_rotation_wheel_command =
        config_.minimum_rotation_command_rad_s * config_.half_track_mm;
    const bool rotation_left_stopped = sample.encoder_available &&
        absolute(left_command) >= minimum_rotation_wheel_command &&
        absolute(sample.encoder_left_velocity_mm_s) < fmaxf(
            10.0f, absolute(left_command) * config_.stopped_ratio);
    const bool rotation_right_stopped = sample.encoder_available &&
        absolute(right_command) >= minimum_rotation_wheel_command &&
        absolute(sample.encoder_right_velocity_mm_s) < fmaxf(
            10.0f, absolute(right_command) * config_.stopped_ratio);
    const bool rotation_wheels_stopped =
        rotation_left_stopped && rotation_right_stopped;
    const bool rotation_evidence = armed && rotation_dominant &&
        (sample.gyro_available
            ? (gyro_stopped || gyro_reversed)
            : rotation_wheels_stopped);
    const bool rotation_healthy = rotation_dominant && gyro_moving;
    rotation_blocked_active_ms_ = updateTimer(
        rotation_blocked_active_ms_, rotation_evidence,
        rotation_healthy, dt_ms);

    if (sample.acceleration_available) {
        if (have_previous_acceleration_ &&
            absolute(sample.acceleration_x_g - previous_acceleration_x_g_) >=
                config_.acceleration_impact_delta_g) {
            acceleration_impact_until_ms_ = sample.timestamp_ms +
                config_.acceleration_impact_hold_ms;
        }
        previous_acceleration_x_g_ = sample.acceleration_x_g;
        have_previous_acceleration_ = true;
    }
    const bool impact_recent = static_cast<int32_t>(
        acceleration_impact_until_ms_ - sample.timestamp_ms) > 0;
    if (impact_recent) {
        result.evidence_flags |= EVIDENCE_ACCELERATION_IMPACT;
    }

    const bool wheels_spinning = translation_dominant &&
        left_moving && right_moving;
    bool gps_speed_healthy = false;
    bool gps_speed_mismatch = false;
    if (wheels_spinning && sample.gps_updated &&
        sample.gps_velocity_available &&
        sample.gps_speed_accuracy_mm_s <=
            config_.gps_maximum_speed_accuracy_mm_s) {
        const float gps_speed = sqrtf(
            sample.gps_velocity_east_mm_s * sample.gps_velocity_east_mm_s +
            sample.gps_velocity_north_mm_s * sample.gps_velocity_north_mm_s);
        const float wheel_speed = absolute(encoder_velocity);
        const float upper_speed = gps_speed +
            2.0f * sample.gps_speed_accuracy_mm_s;
        const float lower_speed = fmaxf(
            0.0f, gps_speed - 2.0f * sample.gps_speed_accuracy_mm_s);
        gps_speed_mismatch = upper_speed <=
            config_.slip_speed_ratio * wheel_speed;
        gps_speed_healthy = lower_speed >=
            config_.healthy_speed_ratio * wheel_speed;
        if (gps_speed_mismatch) {
            if (gps_speed_mismatch_count_ < UINT8_MAX) {
                ++gps_speed_mismatch_count_;
            }
            result.evidence_flags |= EVIDENCE_GPS_SPEED_MISMATCH;
        } else if (gps_speed_healthy) {
            gps_speed_mismatch_count_ = 0;
            result.evidence_flags |= EVIDENCE_GPS_MOVING;
        } else if (gps_speed_mismatch_count_ > 0U) {
            --gps_speed_mismatch_count_;
        }
    }

    bool gps_distance_healthy = false;
    bool gps_distance_mismatch = false;
    if (wheels_spinning && sample.gps_available && sample.gps_updated &&
        sample.gps_horizontal_accuracy_mm <=
            config_.gps_maximum_accuracy_mm) {
        pushGpsPoint(sample);
        gps_distance_mismatch = gpsDistanceMismatch(gps_distance_healthy);
        if (gps_distance_mismatch) {
            result.evidence_flags |= EVIDENCE_GPS_NOT_MOVING |
                EVIDENCE_ENCODER_GPS_MISMATCH;
        } else if (gps_distance_healthy) {
            result.evidence_flags |= EVIDENCE_GPS_MOVING;
        }
    } else if (!wheels_spinning) {
        resetSlipTracking();
    }

    const bool encoder_turn_expected = wheels_spinning &&
        absolute(encoder_yaw_rate) >= config_.minimum_rotation_command_rad_s;
    const bool slip_gyro_mismatch = encoder_turn_expected &&
        sample.gyro_available &&
        (absolute(sample.gyro_yaw_rate_rad_s) <
             config_.stopped_yaw_rate_rad_s ||
         sample.gyro_yaw_rate_rad_s * encoder_yaw_rate < 0.0f);
    slip_gyro_mismatch_active_ms_ = updateTimer(
        slip_gyro_mismatch_active_ms_, slip_gyro_mismatch,
        encoder_turn_expected && !slip_gyro_mismatch, dt_ms);
    if (slip_gyro_mismatch) {
        result.evidence_flags |= EVIDENCE_ENCODER_GYRO_MISMATCH;
    }

    const bool independent_ground_observation =
        sample.gps_available || sample.gps_velocity_available;
    if (wheels_spinning && !independent_ground_observation) {
        slip_no_ground_active_ms_ = saturatedAdd(
            slip_no_ground_active_ms_, dt_ms);
        slip_no_ground_encoder_mm_ += encoder_delta_mm;
        result.evidence_flags |= EVIDENCE_GPS_UNAVAILABLE;
    } else if (!wheels_spinning || gps_speed_healthy || gps_distance_healthy) {
        slip_no_ground_active_ms_ = 0;
        slip_no_ground_encoder_mm_ = 0.0f;
    }

    const bool slip_from_speed =
        gps_speed_mismatch_count_ >= config_.slip_speed_mismatch_samples;
    const bool slip_from_gyro =
        slip_gyro_mismatch_active_ms_ >= config_.slip_gyro_mismatch_arm_ms;
    const bool slip_without_ground =
        slip_no_ground_active_ms_ >= config_.slip_no_ground_arm_ms &&
        slip_no_ground_encoder_mm_ >= config_.slip_no_ground_encoder_mm;
    const bool slip_after_impact = impact_recent &&
        slip_no_ground_active_ms_ >= config_.slip_after_impact_arm_ms &&
        slip_no_ground_encoder_mm_ >= config_.slip_after_impact_encoder_mm;
    const bool slip_candidate = armed && wheels_spinning &&
        (slip_from_speed || gps_distance_mismatch || slip_from_gyro ||
         slip_without_ground || slip_after_impact);

    scores_.wheel_blocked = timerScore(
        wheel_blocked_active_ms_, config_.wheel_blocked_arm_ms);
    if (direction_mismatch_active_ms_ > 0U) {
        const uint16_t direction_score = timerScore(
            direction_mismatch_active_ms_,
            config_.direction_mismatch_arm_ms);
        if (direction_score > scores_.wheel_blocked) {
            scores_.wheel_blocked = direction_score;
        }
    }
    scores_.rotation_blocked = timerScore(
        rotation_blocked_active_ms_, config_.rotation_blocked_arm_ms);
    uint16_t slip_score = timerScore(
        slip_no_ground_active_ms_, config_.slip_no_ground_arm_ms);
    const uint16_t gyro_score = timerScore(
        slip_gyro_mismatch_active_ms_, config_.slip_gyro_mismatch_arm_ms);
    if (gyro_score > slip_score) slip_score = gyro_score;
    if (gps_speed_mismatch_count_ > 0U) {
        const uint16_t speed_score = static_cast<uint16_t>(fminf(
            1000.0f,
            1000.0f * gps_speed_mismatch_count_ /
                fmaxf(1.0f, config_.slip_speed_mismatch_samples)));
        if (speed_score > slip_score) slip_score = speed_score;
    }
    if (gps_distance_mismatch) slip_score = 1000U;
    if (slip_candidate) slip_score = 1000U;
    scores_.wheel_slip = slip_score;
    result.scores = scores_;

    if (suspend_latched_) {
        result.condition = Condition::Suspected;
        return result;
    }

    if (direction_mismatch_active_ms_ >=
            config_.direction_mismatch_arm_ms ||
        wheel_blocked_active_ms_ >= config_.wheel_blocked_arm_ms) {
        result.reason = Reason::WheelBlocked;
    } else if (rotation_blocked_active_ms_ >=
               config_.rotation_blocked_arm_ms) {
        result.reason = Reason::RotationBlocked;
    } else if (slip_candidate) {
        result.reason = Reason::WheelSlip;
    }

    if (result.reason != Reason::None) {
        result.condition = Condition::Suspected;
        result.suspend_requested = true;
        suspend_latched_ = true;
    }
    return result;
}

} // namespace Domain::Motion
