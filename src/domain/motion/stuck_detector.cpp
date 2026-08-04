#include "stuck_detector.h"

#include <math.h>
#include <string.h>

namespace Domain::Motion {

namespace {

uint16_t saturatingStep(
    uint16_t value,
    uint16_t rate_per_s,
    uint32_t dt_ms,
    uint16_t maximum,
    bool increase)
{
    uint32_t step =
        (static_cast<uint32_t>(rate_per_s) * dt_ms + 999U) / 1000U;
    if (step == 0U && dt_ms != 0U) step = 1U;
    if (increase) {
        const uint32_t sum = static_cast<uint32_t>(value) + step;
        return static_cast<uint16_t>(sum > maximum ? maximum : sum);
    }
    return static_cast<uint16_t>(step >= value ? 0U : value - step);
}

float radiansToDegrees(float radians)
{
    return radians * 57.2957795f;
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
    last_gps_seen_ms_ = 0;
    resetGpsWindow();
    scores_ = {};
    body_tilt_active_ms_ = 0;
    memset(gravity_body_g_, 0, sizeof(gravity_body_g_));
    have_gravity_ = false;
    current_tilt_deg_ = 0.0f;
    suspend_latched_ = false;
}

float StuckDetector::absolute(float value)
{
    return value < 0.0f ? -value : value;
}

void StuckDetector::resetGpsWindow()
{
    gps_window_started_ms_ = 0;
    gps_start_x_mm_ = 0;
    gps_start_y_mm_ = 0;
    gps_max_radius_mm_ = 0.0f;
    gps_encoder_mm_ = 0.0f;
    gps_sample_count_ = 0;
    gps_window_active_ = false;
}

void StuckDetector::updateScore(
    uint16_t& score,
    bool evidence,
    bool healthy,
    uint16_t rise_per_s,
    uint32_t dt_ms)
{
    if (evidence) {
        score = saturatingStep(
            score, rise_per_s, dt_ms, config_.score_maximum, true);
    } else {
        score = saturatingStep(
            score,
            healthy ? config_.healthy_decay_per_s
                    : config_.neutral_decay_per_s,
            dt_ms,
            config_.score_maximum,
            false);
    }
}

uint16_t StuckDetector::scoreForReason(Reason reason) const
{
    switch (reason) {
        case Reason::WheelBlocked: return scores_.wheel_blocked;
        case Reason::WheelSlip: return scores_.wheel_slip;
        case Reason::RotationBlocked: return scores_.rotation_blocked;
        case Reason::BodyTrapped: return scores_.body_trapped;
        default: return 0;
    }
}

StuckScores StuckDetector::scores() const
{
    return scores_;
}

DetectorDiagnostics StuckDetector::diagnostics(uint32_t timestamp_ms) const
{
    const auto saturatedU16 = [](float value) -> uint16_t {
        if (!isfinite(value) || value <= 0.0f) return 0;
        if (value >= 65535.0f) return UINT16_MAX;
        return static_cast<uint16_t>(lroundf(value));
    };

    DetectorDiagnostics result{};
    result.tilt_deg_x10 = saturatedU16(current_tilt_deg_ * 10.0f);
    result.gps_window_age_ms = gps_window_active_
        ? saturatedU16(static_cast<float>(
            static_cast<uint32_t>(timestamp_ms - gps_window_started_ms_)))
        : 0;
    result.gps_max_radius_mm = saturatedU16(gps_max_radius_mm_);
    result.gps_encoder_distance_mm = saturatedU16(gps_encoder_mm_);
    result.gps_sample_count = gps_sample_count_;
    return result;
}

void StuckDetector::completeVerification(
    Reason reason,
    bool movement_confirmed)
{
    uint16_t* score = nullptr;
    switch (reason) {
        case Reason::WheelBlocked: score = &scores_.wheel_blocked; break;
        case Reason::WheelSlip: score = &scores_.wheel_slip; break;
        case Reason::RotationBlocked: score = &scores_.rotation_blocked; break;
        case Reason::BodyTrapped: score = &scores_.body_trapped; break;
        default: break;
    }
    if (score != nullptr) {
        *score = movement_confirmed ? 0U : config_.rearm_score;
    }
    suspend_latched_ = false;
}

Assessment StuckDetector::update(const DetectorSample& sample)
{
    Assessment assessment{};
    assessment.timestamp_ms = sample.timestamp_ms;
    assessment.condition = Condition::Normal;
    assessment.reason = Reason::None;
    assessment.encoder_trusted = sample.encoder_available;
    assessment.gps_trusted = sample.gps_available;
    assessment.gyro_trusted = sample.gyro_available;

    const uint32_t dt_ms = sample.dt_ms > 200U ? 200U : sample.dt_ms;
    const float dt_s = static_cast<float>(dt_ms) * 0.001f;
    const bool translation_commanded =
        sample.command_valid &&
        absolute(sample.command_velocity_mm_s) >=
            config_.minimum_translation_command_mm_s;
    const bool rotation_commanded =
        sample.command_valid &&
        absolute(sample.command_yaw_rate_rad_s) >=
            config_.minimum_rotation_command_rad_s;
    const bool motion_commanded = translation_commanded || rotation_commanded;

    if (!sample.navigation_active) {
        reset(sample.timestamp_ms);
        assessment.scores = scores_;
        return assessment;
    }

    if (motion_commanded) {
        if (last_motion_command_ms_ != 0U &&
            static_cast<uint32_t>(
                sample.timestamp_ms - last_motion_command_ms_) >
                config_.command_gap_reset_ms) {
            command_active_ms_ = 0;
            resetGpsWindow();
        }
        last_motion_command_ms_ = sample.timestamp_ms;
        const uint32_t room = UINT32_MAX - command_active_ms_;
        command_active_ms_ += dt_ms < room ? dt_ms : room;
    } else {
        command_active_ms_ = 0;
        resetGpsWindow();
    }
    const bool armed = command_active_ms_ >= config_.command_arm_ms;

    const float left_command = sample.command_velocity_mm_s -
        sample.command_yaw_rate_rad_s * config_.half_track_mm;
    const float right_command = sample.command_velocity_mm_s +
        sample.command_yaw_rate_rad_s * config_.half_track_mm;
    const float rotation_equivalent_mm_s =
        absolute(sample.command_yaw_rate_rad_s) * config_.half_track_mm;
    const bool translation_dominant =
        translation_commanded &&
        absolute(sample.command_velocity_mm_s) > rotation_equivalent_mm_s;
    const bool rotation_dominant =
        rotation_commanded &&
        rotation_equivalent_mm_s >= absolute(sample.command_velocity_mm_s);

    const bool left_commanded =
        absolute(left_command) >= config_.minimum_wheel_command_mm_s;
    const bool right_commanded =
        absolute(right_command) >= config_.minimum_wheel_command_mm_s;
    const float left_stopped_threshold = fmaxf(
        config_.stopped_velocity_mm_s,
        absolute(left_command) * config_.stopped_ratio);
    const float right_stopped_threshold = fmaxf(
        config_.stopped_velocity_mm_s,
        absolute(right_command) * config_.stopped_ratio);
    const bool left_stopped =
        sample.encoder_available && left_commanded &&
        absolute(sample.encoder_left_velocity_mm_s) < left_stopped_threshold;
    const bool right_stopped =
        sample.encoder_available && right_commanded &&
        absolute(sample.encoder_right_velocity_mm_s) < right_stopped_threshold;
    const bool left_moving =
        sample.encoder_available && left_commanded &&
        sample.encoder_left_velocity_mm_s * left_command > 0.0f &&
        absolute(sample.encoder_left_velocity_mm_s) >=
            absolute(left_command) * config_.moving_ratio;
    const bool right_moving =
        sample.encoder_available && right_commanded &&
        sample.encoder_right_velocity_mm_s * right_command > 0.0f &&
        absolute(sample.encoder_right_velocity_mm_s) >=
            absolute(right_command) * config_.moving_ratio;

    if (sample.encoder_available) {
        assessment.evidence_flags |=
            (left_stopped || right_stopped)
                ? EVIDENCE_ENCODER_NOT_MOVING
                : EVIDENCE_ENCODER_MOVING;
        if (left_stopped != right_stopped) {
            assessment.evidence_flags |= EVIDENCE_LEFT_RIGHT_MISMATCH;
        }
    } else {
        assessment.evidence_flags |= EVIDENCE_ENCODER_UNAVAILABLE;
    }

    const bool wheel_blocked_evidence =
        armed && translation_dominant && sample.encoder_available &&
        (left_stopped || right_stopped);
    const bool wheel_blocked_healthy =
        translation_dominant && left_moving && right_moving;

    const float encoder_velocity = 0.5f *
        (sample.encoder_left_velocity_mm_s +
         sample.encoder_right_velocity_mm_s);
    const float encoder_yaw_rate =
        (sample.encoder_right_velocity_mm_s -
         sample.encoder_left_velocity_mm_s) /
        (2.0f * config_.half_track_mm);
    const float encoder_delta_mm = absolute(encoder_velocity) * dt_s;

    const bool gps_reliable =
        sample.gps_available &&
        sample.gps_horizontal_accuracy_mm <=
            config_.gps_maximum_accuracy_mm;
    if (!sample.gps_available) {
        assessment.evidence_flags |= EVIDENCE_GPS_UNAVAILABLE;
    }

    if (translation_dominant && gps_reliable) {
        last_gps_seen_ms_ = sample.timestamp_ms;
        if (!gps_window_active_ && sample.gps_updated) {
            gps_window_active_ = true;
            gps_window_started_ms_ = sample.timestamp_ms;
            gps_start_x_mm_ = sample.gps_x_mm;
            gps_start_y_mm_ = sample.gps_y_mm;
            gps_sample_count_ = 1;
        } else if (gps_window_active_) {
            if (sample.encoder_available) gps_encoder_mm_ += encoder_delta_mm;
            if (sample.gps_updated) {
                const float dx = static_cast<float>(
                    sample.gps_x_mm - gps_start_x_mm_);
                const float dy = static_cast<float>(
                    sample.gps_y_mm - gps_start_y_mm_);
                const float radius = sqrtf(dx * dx + dy * dy);
                if (radius > gps_max_radius_mm_) gps_max_radius_mm_ = radius;
                if (gps_sample_count_ < UINT8_MAX) ++gps_sample_count_;
            }
        }
    } else if (!translation_dominant ||
               (last_gps_seen_ms_ != 0U &&
                static_cast<uint32_t>(
                    sample.timestamp_ms - last_gps_seen_ms_) > 3000U)) {
        resetGpsWindow();
    }

    const float gps_stationary_radius_mm = fmaxf(
        config_.gps_stationary_radius_mm,
        0.75f * static_cast<float>(sample.gps_horizontal_accuracy_mm));
    const float gps_moving_radius_mm = fmaxf(
        1500.0f,
        1.5f * static_cast<float>(sample.gps_horizontal_accuracy_mm));
    const bool gps_window_ready =
        gps_window_active_ &&
        static_cast<uint32_t>(
            sample.timestamp_ms - gps_window_started_ms_) >=
            config_.gps_window_ms &&
        gps_sample_count_ >= config_.gps_minimum_samples &&
        gps_encoder_mm_ >= config_.slip_minimum_encoder_mm;
    const bool gps_stationary =
        gps_window_ready && gps_max_radius_mm_ <= gps_stationary_radius_mm;
    const bool gps_moving =
        gps_window_active_ && gps_max_radius_mm_ >= gps_moving_radius_mm;
    if (gps_stationary) {
        assessment.evidence_flags |= EVIDENCE_GPS_NOT_MOVING;
    } else if (gps_moving) {
        assessment.evidence_flags |= EVIDENCE_GPS_MOVING;
    }

    const bool wheel_slip_evidence =
        armed && translation_dominant && left_moving && right_moving &&
        gps_stationary;
    const bool wheel_slip_healthy =
        translation_dominant && left_moving && right_moving && gps_moving;

    // Use bounded raw-GPS windows. A historical movement in an old window
    // must not mask a later slip at the new location.
    if (gps_window_ready && !gps_stationary) {
        resetGpsWindow();
    }

    const bool gyro_stopped =
        sample.gyro_available &&
        absolute(sample.gyro_yaw_rate_rad_s) <
            config_.stopped_yaw_rate_rad_s;
    const float gyro_moving_threshold = fmaxf(
        config_.stopped_yaw_rate_rad_s,
        absolute(sample.command_yaw_rate_rad_s) * config_.moving_ratio);
    const bool gyro_moving =
        sample.gyro_available &&
        sample.gyro_yaw_rate_rad_s * sample.command_yaw_rate_rad_s > 0.0f &&
        absolute(sample.gyro_yaw_rate_rad_s) >= gyro_moving_threshold;
    const bool encoder_rotation_expected =
        sample.encoder_available &&
        encoder_yaw_rate * sample.command_yaw_rate_rad_s > 0.0f &&
        absolute(encoder_yaw_rate) >= gyro_moving_threshold;
    const float minimum_rotation_wheel_command =
        config_.minimum_rotation_command_rad_s * config_.half_track_mm;
    const bool rotation_left_stopped =
        sample.encoder_available &&
        absolute(left_command) >= minimum_rotation_wheel_command &&
        absolute(sample.encoder_left_velocity_mm_s) <
            left_stopped_threshold;
    const bool rotation_right_stopped =
        sample.encoder_available &&
        absolute(right_command) >= minimum_rotation_wheel_command &&
        absolute(sample.encoder_right_velocity_mm_s) <
            right_stopped_threshold;
    const bool rotation_blocked_evidence =
        armed && rotation_dominant &&
        ((rotation_left_stopped && rotation_right_stopped) ||
         (encoder_rotation_expected && gyro_stopped));
    const bool rotation_blocked_healthy = rotation_dominant && gyro_moving;
    if (sample.gyro_available) {
        assessment.evidence_flags |= gyro_stopped
            ? EVIDENCE_GYRO_NOT_ROTATING
            : EVIDENCE_GYRO_ROTATING;
        if (encoder_rotation_expected && gyro_stopped) {
            assessment.evidence_flags |= EVIDENCE_ENCODER_GYRO_MISMATCH;
        }
    } else {
        assessment.evidence_flags |= EVIDENCE_GYRO_UNAVAILABLE;
    }

    bool tilt_valid = false;
    float tilt_deg = 0.0f;
    if (sample.acceleration_available) {
        const float norm = sqrtf(
            sample.acceleration_x_g * sample.acceleration_x_g +
            sample.acceleration_y_g * sample.acceleration_y_g +
            sample.acceleration_z_g * sample.acceleration_z_g);
        if (isfinite(norm) && norm >= 0.75f && norm <= 1.25f) {
            if (!have_gravity_) {
                gravity_body_g_[0] = sample.acceleration_x_g;
                gravity_body_g_[1] = sample.acceleration_y_g;
                gravity_body_g_[2] = sample.acceleration_z_g;
                have_gravity_ = true;
            } else {
                const float alpha = config_.gravity_low_pass_alpha;
                gravity_body_g_[0] +=
                    alpha * (sample.acceleration_x_g - gravity_body_g_[0]);
                gravity_body_g_[1] +=
                    alpha * (sample.acceleration_y_g - gravity_body_g_[1]);
                gravity_body_g_[2] +=
                    alpha * (sample.acceleration_z_g - gravity_body_g_[2]);
            }
            const float gravity_norm = sqrtf(
                gravity_body_g_[0] * gravity_body_g_[0] +
                gravity_body_g_[1] * gravity_body_g_[1] +
                gravity_body_g_[2] * gravity_body_g_[2]);
            if (gravity_norm > 0.1f) {
                float cosine = gravity_body_g_[2] / gravity_norm;
                if (cosine > 1.0f) cosine = 1.0f;
                if (cosine < -1.0f) cosine = -1.0f;
                tilt_deg = radiansToDegrees(acosf(cosine));
                tilt_valid = isfinite(tilt_deg);
            }
        }
    }
    current_tilt_deg_ = tilt_valid ? tilt_deg : 0.0f;

    if (motion_commanded && tilt_valid &&
        tilt_deg >= config_.body_tilt_start_deg) {
        const uint32_t room = UINT32_MAX - body_tilt_active_ms_;
        body_tilt_active_ms_ += dt_ms < room ? dt_ms : room;
    } else {
        body_tilt_active_ms_ = 0;
    }
    const bool body_trapped_evidence =
        armed && motion_commanded && tilt_valid &&
        body_tilt_active_ms_ >= config_.body_tilt_arm_ms;
    const bool body_trapped_healthy =
        tilt_valid && tilt_deg <= config_.body_tilt_healthy_deg;

    updateScore(
        scores_.wheel_blocked,
        wheel_blocked_evidence,
        wheel_blocked_healthy,
        config_.wheel_blocked_rise_per_s,
        dt_ms);
    updateScore(
        scores_.wheel_slip,
        wheel_slip_evidence,
        wheel_slip_healthy,
        config_.wheel_slip_rise_per_s,
        dt_ms);
    updateScore(
        scores_.rotation_blocked,
        rotation_blocked_evidence,
        rotation_blocked_healthy,
        config_.rotation_blocked_rise_per_s,
        dt_ms);
    updateScore(
        scores_.body_trapped,
        body_trapped_evidence,
        body_trapped_healthy,
        config_.body_trapped_rise_per_s,
        dt_ms);

    assessment.scores = scores_;
    if (suspend_latched_) {
        assessment.condition = Condition::Suspected;
        return assessment;
    }

    Reason selected = Reason::None;
    uint16_t selected_score = 0;
    const Reason reasons[] = {
        Reason::WheelBlocked,
        Reason::WheelSlip,
        Reason::RotationBlocked,
        Reason::BodyTrapped
    };
    for (Reason reason : reasons) {
        const uint16_t score = scoreForReason(reason);
        if (score >= config_.suspend_score && score > selected_score) {
            selected = reason;
            selected_score = score;
        }
    }

    if (selected != Reason::None) {
        assessment.condition = Condition::Suspected;
        assessment.reason = selected;
        assessment.suspend_requested = true;
        suspend_latched_ = true;
    }
    return assessment;
}

} // namespace Domain::Motion
