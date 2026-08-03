#include "stuck_detector.h"

#include <math.h>
#include <string.h>

namespace Domain::Motion {

namespace {

uint8_t countTrue(bool a, bool b, bool c)
{
    return static_cast<uint8_t>(a) +
        static_cast<uint8_t>(b) +
        static_cast<uint8_t>(c);
}

} // namespace

StuckDetector::StuckDetector(const DetectorConfig& config)
    : config_(config)
{
    reset();
}

void StuckDetector::reset(uint32_t timestamp_ms)
{
    memset(candidate_since_ms_, 0, sizeof(candidate_since_ms_));
    memset(candidate_active_ms_, 0, sizeof(candidate_active_ms_));
    command_active_ms_ = 0;
    last_motion_command_ms_ = timestamp_ms;
    last_gps_seen_ms_ = 0;
    resetGpsWindow();
    resetPathWindow();
    resetOscillationWindow();
    previous_fusion_x_mm_ = 0;
    previous_fusion_y_mm_ = 0;
    have_previous_fusion_position_ = false;
}

float StuckDetector::absolute(float value)
{
    return value < 0.0f ? -value : value;
}

int8_t StuckDetector::signWithDeadband(float value, float deadband)
{
    if (value > deadband) return 1;
    if (value < -deadband) return -1;
    return 0;
}

uint8_t StuckDetector::reasonIndex(Reason reason)
{
    const uint8_t index = static_cast<uint8_t>(reason);
    return index < REASON_COUNT ? index : 0;
}

bool StuckDetector::confirmCandidate(
    Reason reason,
    bool suspected,
    uint32_t required_ms,
    uint32_t now_ms,
    uint32_t active_dt_ms)
{
    const uint8_t index = reasonIndex(reason);
    if (!suspected) {
        candidate_since_ms_[index] = 0;
        candidate_active_ms_[index] = 0;
        return false;
    }
    if (candidate_since_ms_[index] == 0) {
        candidate_since_ms_[index] = now_ms;
    }
    const uint32_t remaining =
        UINT32_MAX - candidate_active_ms_[index];
    candidate_active_ms_[index] +=
        active_dt_ms < remaining ? active_dt_ms : remaining;
    return candidate_active_ms_[index] >= required_ms;
}

void StuckDetector::clearCandidates()
{
    memset(candidate_since_ms_, 0, sizeof(candidate_since_ms_));
    memset(candidate_active_ms_, 0, sizeof(candidate_active_ms_));
}

void StuckDetector::resetGpsWindow()
{
    gps_window_started_ms_ = 0;
    gps_start_x_mm_ = 0;
    gps_start_y_mm_ = 0;
    gps_max_radius_mm_ = 0.0f;
    gps_expected_mm_ = 0.0f;
    gps_encoder_mm_ = 0.0f;
    gps_sample_count_ = 0;
    gps_window_active_ = false;
}

void StuckDetector::resetPathWindow()
{
    path_window_started_ms_ = 0;
    path_revision_ = 0;
    path_start_index_ = 0;
    path_start_goal_distance_mm_ = 0.0f;
    path_commanded_mm_ = 0.0f;
    path_window_active_ = false;
}

void StuckDetector::resetOscillationWindow()
{
    oscillation_window_started_ms_ = 0;
    oscillation_start_x_mm_ = 0;
    oscillation_start_y_mm_ = 0;
    oscillation_max_radius_mm_ = 0.0f;
    oscillation_motion_mm_ = 0.0f;
    oscillation_reversals_ = 0;
    previous_translation_sign_ = 0;
    previous_rotation_sign_ = 0;
    oscillation_window_active_ = false;
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

    const float dt_s =
        static_cast<float>(sample.dt_ms > 200U ? 200U : sample.dt_ms) *
        0.001f;
    const bool translation_commanded =
        sample.command_valid &&
        absolute(sample.command_velocity_mm_s) >=
            config_.minimum_translation_command_mm_s;
    const bool rotation_commanded =
        sample.command_valid &&
        absolute(sample.command_yaw_rate_rad_s) >=
            config_.minimum_rotation_command_rad_s;
    const bool motion_commanded =
        translation_commanded || rotation_commanded;

    if (!sample.navigation_active) {
        reset(sample.timestamp_ms);
        return assessment;
    }

    if (!motion_commanded) {
        if (last_motion_command_ms_ == 0U ||
            static_cast<uint32_t>(
                sample.timestamp_ms - last_motion_command_ms_) >
                config_.command_gap_reset_ms) {
            reset(sample.timestamp_ms);
        }
        return assessment;
    }

    if (last_motion_command_ms_ != 0U &&
        static_cast<uint32_t>(
            sample.timestamp_ms - last_motion_command_ms_) >
            config_.command_gap_reset_ms) {
        reset();
    }
    last_motion_command_ms_ = sample.timestamp_ms;
    const uint32_t active_dt_ms =
        sample.dt_ms > 200U ? 200U : sample.dt_ms;
    const uint32_t command_remaining =
        UINT32_MAX - command_active_ms_;
    command_active_ms_ +=
        active_dt_ms < command_remaining
            ? active_dt_ms
            : command_remaining;
    const bool armed = command_active_ms_ >= config_.command_arm_ms;

    const float encoder_velocity = 0.5f *
        (sample.encoder_left_velocity_mm_s +
         sample.encoder_right_velocity_mm_s);
    const float encoder_yaw_rate =
        (sample.encoder_right_velocity_mm_s -
         sample.encoder_left_velocity_mm_s) /
        (2.0f * config_.half_track_mm);

    if (sample.encoder_available) {
        assessment.evidence_flags |=
            absolute(encoder_velocity) <
                    config_.stopped_velocity_mm_s
                ? EVIDENCE_ENCODER_NOT_MOVING
                : EVIDENCE_ENCODER_MOVING;
    } else {
        assessment.evidence_flags |= EVIDENCE_ENCODER_UNAVAILABLE;
    }
    if (sample.gyro_available) {
        assessment.evidence_flags |=
            absolute(sample.gyro_yaw_rate_rad_s) <
                    config_.stopped_yaw_rate_rad_s
                ? EVIDENCE_GYRO_NOT_ROTATING
                : EVIDENCE_GYRO_ROTATING;
    } else {
        assessment.evidence_flags |= EVIDENCE_GYRO_UNAVAILABLE;
    }
    const bool fusion_translation_available =
        sample.fusion_available && sample.fusion_position_usable;
    const bool fusion_rotation_available =
        sample.fusion_available && sample.fusion_yaw_usable;
    if (fusion_translation_available) {
        assessment.evidence_flags |=
            absolute(sample.fusion_forward_velocity_mm_s) <
                    config_.stopped_velocity_mm_s
                ? EVIDENCE_FUSION_NOT_MOVING
                : EVIDENCE_FUSION_MOVING;
    } else {
        assessment.evidence_flags |= EVIDENCE_FUSION_UNAVAILABLE;
    }
    if (!sample.gps_available) {
        assessment.evidence_flags |= EVIDENCE_GPS_UNAVAILABLE;
    }

    float fusion_delta_mm = 0.0f;
    if (sample.fusion_position_usable) {
        if (have_previous_fusion_position_) {
            const float dx = static_cast<float>(
                sample.fusion_x_mm - previous_fusion_x_mm_);
            const float dy = static_cast<float>(
                sample.fusion_y_mm - previous_fusion_y_mm_);
            fusion_delta_mm = sqrtf(dx * dx + dy * dy);
        }
        previous_fusion_x_mm_ = sample.fusion_x_mm;
        previous_fusion_y_mm_ = sample.fusion_y_mm;
        have_previous_fusion_position_ = true;
    } else {
        have_previous_fusion_position_ = false;
    }

    const float encoder_delta_mm =
        absolute(encoder_velocity) * dt_s;

    const bool gps_reliable_for_stuck =
        sample.gps_available &&
        sample.gps_horizontal_accuracy_mm <=
            config_.gps_maximum_accuracy_mm;

    if (translation_commanded && gps_reliable_for_stuck) {
        last_gps_seen_ms_ = sample.timestamp_ms;
        gps_expected_mm_ +=
            absolute(sample.command_velocity_mm_s) * dt_s;
        if (sample.encoder_available) gps_encoder_mm_ += encoder_delta_mm;
        if (sample.gps_updated) {
            if (!gps_window_active_) {
                gps_window_active_ = true;
                gps_window_started_ms_ = sample.timestamp_ms;
                gps_start_x_mm_ = sample.gps_x_mm;
                gps_start_y_mm_ = sample.gps_y_mm;
                gps_max_radius_mm_ = 0.0f;
                gps_expected_mm_ = 0.0f;
                gps_encoder_mm_ = 0.0f;
                gps_sample_count_ = 1;
            } else {
                const float dx = static_cast<float>(
                    sample.gps_x_mm - gps_start_x_mm_);
                const float dy = static_cast<float>(
                    sample.gps_y_mm - gps_start_y_mm_);
                const float radius = sqrtf(dx * dx + dy * dy);
                if (radius > gps_max_radius_mm_) {
                    gps_max_radius_mm_ = radius;
                }
                if (gps_sample_count_ < UINT8_MAX) gps_sample_count_++;
            }
        }
    } else if (!translation_commanded ||
               (last_gps_seen_ms_ != 0 &&
                static_cast<uint32_t>(
                    sample.timestamp_ms - last_gps_seen_ms_) > 3000U)) {
        resetGpsWindow();
    }

    const float required_gps_expected_mm = fmaxf(
        config_.gps_minimum_expected_mm,
        4.0f * static_cast<float>(
            sample.gps_horizontal_accuracy_mm));
    const bool gps_stationary =
        gps_window_active_ &&
        static_cast<uint32_t>(
            sample.timestamp_ms - gps_window_started_ms_) >=
            config_.gps_window_ms &&
        gps_sample_count_ >= config_.gps_minimum_samples &&
        gps_expected_mm_ >= required_gps_expected_mm &&
        gps_max_radius_mm_ <= config_.gps_stationary_radius_mm;
    if (gps_stationary) {
        assessment.evidence_flags |= EVIDENCE_GPS_NOT_MOVING;
    } else if (gps_window_active_ &&
               gps_max_radius_mm_ >
                   config_.gps_stationary_radius_mm) {
        assessment.evidence_flags |= EVIDENCE_GPS_MOVING;
    }

    if (sample.path_available && translation_commanded) {
        if (!path_window_active_ ||
            sample.path_revision != path_revision_) {
            path_window_active_ = true;
            path_window_started_ms_ = sample.timestamp_ms;
            path_revision_ = sample.path_revision;
            path_start_index_ = sample.path_nearest_index;
            path_start_goal_distance_mm_ =
                sample.path_distance_to_goal_mm;
            path_commanded_mm_ = 0.0f;
        } else {
            path_commanded_mm_ +=
                absolute(sample.command_velocity_mm_s) * dt_s;
        }
    } else {
        resetPathWindow();
    }

    if (sample.fusion_position_usable) {
        if (!oscillation_window_active_) {
            oscillation_window_active_ = true;
            oscillation_window_started_ms_ = sample.timestamp_ms;
            oscillation_start_x_mm_ = sample.fusion_x_mm;
            oscillation_start_y_mm_ = sample.fusion_y_mm;
            oscillation_max_radius_mm_ = 0.0f;
            oscillation_motion_mm_ = 0.0f;
            oscillation_reversals_ = 0;
        }
        const float dx = static_cast<float>(
            sample.fusion_x_mm - oscillation_start_x_mm_);
        const float dy = static_cast<float>(
            sample.fusion_y_mm - oscillation_start_y_mm_);
        const float radius = sqrtf(dx * dx + dy * dy);
        if (radius > oscillation_max_radius_mm_) {
            oscillation_max_radius_mm_ = radius;
        }
        oscillation_motion_mm_ +=
            sample.encoder_available ? encoder_delta_mm : fusion_delta_mm;

        const int8_t translation_sign = signWithDeadband(
            sample.command_velocity_mm_s,
            config_.minimum_translation_command_mm_s);
        const int8_t rotation_sign = signWithDeadband(
            sample.command_yaw_rate_rad_s,
            config_.minimum_rotation_command_rad_s);
        if ((translation_sign != 0 && previous_translation_sign_ != 0 &&
             translation_sign != previous_translation_sign_) ||
            (rotation_sign != 0 && previous_rotation_sign_ != 0 &&
             rotation_sign != previous_rotation_sign_)) {
            if (oscillation_reversals_ < UINT8_MAX) {
                oscillation_reversals_++;
            }
        }
        if (translation_sign != 0) {
            previous_translation_sign_ = translation_sign;
        }
        if (rotation_sign != 0) {
            previous_rotation_sign_ = rotation_sign;
        }
    } else {
        resetOscillationWindow();
    }

    if (!armed) return assessment;

    const float command_abs =
        absolute(sample.command_velocity_mm_s);
    const float stopped_translation_threshold =
        fmaxf(config_.stopped_velocity_mm_s,
              command_abs * config_.stopped_ratio);
    const float moving_translation_threshold =
        fmaxf(config_.stopped_velocity_mm_s,
              command_abs * config_.moving_ratio);

    const bool encoder_translation_stopped =
        sample.encoder_available &&
        absolute(encoder_velocity) < stopped_translation_threshold;
    const bool fusion_translation_stopped =
        fusion_translation_available &&
        absolute(sample.fusion_forward_velocity_mm_s) <
            stopped_translation_threshold;
    const bool encoder_translation_moving =
        sample.encoder_available &&
        absolute(encoder_velocity) > moving_translation_threshold;
    const bool fusion_translation_moving =
        fusion_translation_available &&
        absolute(sample.fusion_forward_velocity_mm_s) >
            moving_translation_threshold;

    // Fusion velocity contains encoder information. Count it as an
    // independent source only when a trusted encoder is unavailable.
    const bool independent_fusion_translation =
        fusion_translation_available && !sample.encoder_available;
    const uint8_t translation_available = countTrue(
        sample.encoder_available,
        independent_fusion_translation,
        gps_stationary);
    const uint8_t translation_stopped = countTrue(
        encoder_translation_stopped,
        fusion_translation_stopped && independent_fusion_translation,
        gps_stationary);
    const bool contradictory_translation =
        encoder_translation_moving || fusion_translation_moving ||
        (assessment.evidence_flags & EVIDENCE_GPS_MOVING) != 0U;
    const bool translation_suspected =
        translation_commanded &&
        !contradictory_translation &&
        translation_stopped >=
            (translation_available >= 2U ? 2U : 1U);
    const uint32_t translation_confirm_ms =
        translation_available >= 2U
            ? config_.blocked_confirm_ms
            : config_.single_source_confirm_ms;

    const float left_command =
        sample.command_velocity_mm_s -
        sample.command_yaw_rate_rad_s * config_.half_track_mm;
    const float right_command =
        sample.command_velocity_mm_s +
        sample.command_yaw_rate_rad_s * config_.half_track_mm;
    const bool left_commanded =
        absolute(left_command) >= config_.minimum_wheel_command_mm_s;
    const bool right_commanded =
        absolute(right_command) >= config_.minimum_wheel_command_mm_s;
    const bool left_stopped = sample.encoder_available && left_commanded &&
        absolute(sample.encoder_left_velocity_mm_s) <
            fmaxf(config_.stopped_velocity_mm_s,
                  absolute(left_command) * config_.stopped_ratio);
    const bool right_stopped = sample.encoder_available && right_commanded &&
        absolute(sample.encoder_right_velocity_mm_s) <
            fmaxf(config_.stopped_velocity_mm_s,
                  absolute(right_command) * config_.stopped_ratio);
    const bool left_moving = sample.encoder_available && left_commanded &&
        absolute(sample.encoder_left_velocity_mm_s) >
            absolute(left_command) * config_.moving_ratio;
    const bool right_moving = sample.encoder_available && right_commanded &&
        absolute(sample.encoder_right_velocity_mm_s) >
            absolute(right_command) * config_.moving_ratio;

    const bool encoder_rotation_stopped =
        sample.encoder_available &&
        absolute(encoder_yaw_rate) <
            config_.stopped_yaw_rate_rad_s;
    const bool gyro_rotation_stopped =
        sample.gyro_available &&
        absolute(sample.gyro_yaw_rate_rad_s) <
            config_.stopped_yaw_rate_rad_s;
    const bool fusion_rotation_stopped =
        fusion_rotation_available &&
        absolute(sample.fusion_yaw_rate_rad_s) <
            config_.stopped_yaw_rate_rad_s;
    const float commanded_rotation_abs =
        absolute(sample.command_yaw_rate_rad_s);
    const float moving_rotation_threshold = fmaxf(
        config_.stopped_yaw_rate_rad_s,
        commanded_rotation_abs * config_.moving_ratio);
    const bool encoder_rotation_moving =
        sample.encoder_available &&
        absolute(encoder_yaw_rate) > moving_rotation_threshold;
    const bool gyro_rotation_moving =
        sample.gyro_available &&
        absolute(sample.gyro_yaw_rate_rad_s) >
            moving_rotation_threshold;
    const bool fusion_rotation_moving =
        fusion_rotation_available &&
        absolute(sample.fusion_yaw_rate_rad_s) >
            moving_rotation_threshold;
    const bool independent_fusion_rotation =
        fusion_rotation_available &&
        !sample.encoder_available &&
        !sample.gyro_available;
    const uint8_t rotation_available = countTrue(
        sample.encoder_available,
        sample.gyro_available,
        independent_fusion_rotation);
    const uint8_t rotation_stopped = countTrue(
        encoder_rotation_stopped,
        gyro_rotation_stopped,
        fusion_rotation_stopped && independent_fusion_rotation);
    const bool contradictory_rotation =
        encoder_rotation_moving ||
        gyro_rotation_moving ||
        fusion_rotation_moving;
    const bool rotation_suspected =
        rotation_commanded &&
        !contradictory_rotation &&
        rotation_stopped >=
            (rotation_available >= 2U ? 2U : 1U);

    const bool slip_suspected =
        translation_commanded &&
        sample.encoder_available &&
        gps_stationary &&
        gps_encoder_mm_ >= config_.slip_minimum_encoder_mm;

    const bool path_window_ready =
        path_window_active_ &&
        static_cast<uint32_t>(
            sample.timestamp_ms - path_window_started_ms_) >=
            config_.path_window_ms &&
        path_commanded_mm_ >= config_.path_minimum_commanded_mm;
    const bool path_index_stalled =
        sample.path_nearest_index <
        static_cast<uint16_t>(
            path_start_index_ + config_.path_minimum_index_advance);
    const bool goal_not_improving =
        path_start_goal_distance_mm_ -
            sample.path_distance_to_goal_mm <
        config_.path_minimum_goal_improvement_mm;
    const bool path_suspected =
        path_window_ready &&
        path_index_stalled &&
        goal_not_improving &&
        (encoder_translation_moving || fusion_translation_moving);
    if (path_suspected) {
        assessment.evidence_flags |= EVIDENCE_PATH_NOT_PROGRESSING;
    }

    const bool oscillation_suspected =
        oscillation_window_active_ &&
        static_cast<uint32_t>(
            sample.timestamp_ms - oscillation_window_started_ms_) >=
            config_.oscillation_window_ms &&
        oscillation_reversals_ >=
            config_.oscillation_minimum_reversals &&
        oscillation_motion_mm_ >=
            config_.oscillation_minimum_motion_mm &&
        oscillation_max_radius_mm_ <=
            config_.oscillation_maximum_radius_mm;
    if (oscillation_suspected) {
        assessment.evidence_flags |= EVIDENCE_COMMAND_OSCILLATING;
    }

    if (gps_window_active_ && !gps_stationary &&
        sample.gps_updated &&
        static_cast<uint32_t>(
            sample.timestamp_ms - gps_window_started_ms_) >=
            config_.gps_window_ms) {
        gps_window_started_ms_ = sample.timestamp_ms;
        gps_start_x_mm_ = sample.gps_x_mm;
        gps_start_y_mm_ = sample.gps_y_mm;
        gps_max_radius_mm_ = 0.0f;
        gps_expected_mm_ = 0.0f;
        gps_encoder_mm_ = 0.0f;
        gps_sample_count_ = 1;
    }
    if (path_window_ready && !path_suspected) {
        resetPathWindow();
    }
    if (oscillation_window_active_ && !oscillation_suspected &&
        static_cast<uint32_t>(
            sample.timestamp_ms - oscillation_window_started_ms_) >=
            config_.oscillation_window_ms) {
        resetOscillationWindow();
    }

    Reason reason = Reason::None;
    if (confirmCandidate(
            Reason::LeftWheelBlocked,
            left_stopped && right_moving,
            config_.wheel_blocked_confirm_ms,
            sample.timestamp_ms,
            active_dt_ms)) {
        reason = Reason::LeftWheelBlocked;
    } else if (confirmCandidate(
                   Reason::RightWheelBlocked,
                   right_stopped && left_moving,
                   config_.wheel_blocked_confirm_ms,
                   sample.timestamp_ms,
                   active_dt_ms)) {
        reason = Reason::RightWheelBlocked;
    } else if (confirmCandidate(
                   Reason::RotationBlocked,
                   rotation_suspected,
                   rotation_available >= 2U
                       ? config_.rotation_confirm_ms
                       : config_.single_source_confirm_ms,
                   sample.timestamp_ms,
                   active_dt_ms)) {
        reason = Reason::RotationBlocked;
    } else if (confirmCandidate(
                   Reason::WheelSlip,
                   slip_suspected,
                   config_.blocked_confirm_ms,
                   sample.timestamp_ms,
                   active_dt_ms)) {
        reason = Reason::WheelSlip;
    } else if (confirmCandidate(
                   Reason::TranslationBlocked,
                   translation_suspected,
                   translation_confirm_ms,
                   sample.timestamp_ms,
                   active_dt_ms)) {
        reason =
            !sample.encoder_available && gps_stationary
                ? Reason::GpsNoProgress
                : Reason::TranslationBlocked;
    } else if (confirmCandidate(
                   Reason::Oscillation,
                   oscillation_suspected,
                   1000U,
                   sample.timestamp_ms,
                   active_dt_ms)) {
        reason = Reason::Oscillation;
    } else if (confirmCandidate(
                   Reason::PathNoProgress,
                   path_suspected,
                   1000U,
                   sample.timestamp_ms,
                   active_dt_ms)) {
        reason = Reason::PathNoProgress;
    }

    const bool translation_unobservable =
        translation_commanded &&
        !sample.encoder_available &&
        !fusion_translation_available &&
        !sample.gps_available;
    const bool rotation_unobservable =
        rotation_commanded &&
        !sample.encoder_available &&
        !fusion_rotation_available &&
        !sample.gyro_available;
    const bool motion_unobservable =
        translation_unobservable || rotation_unobservable;
    if (reason == Reason::None &&
        confirmCandidate(
            Reason::MotionUnobservable,
            motion_unobservable,
            config_.unobservable_confirm_ms,
            sample.timestamp_ms,
            active_dt_ms)) {
        reason = Reason::MotionUnobservable;
        assessment.condition = Condition::SensorFault;
    }

    if (reason != Reason::None) {
        assessment.reason = reason;
        assessment.suspected_since_ms =
            candidate_since_ms_[reasonIndex(reason)];
        if (assessment.condition != Condition::SensorFault) {
            assessment.condition = Condition::Stuck;
        }
        return assessment;
    }

    for (uint8_t i = 1; i < REASON_COUNT; ++i) {
        if (candidate_since_ms_[i] != 0) {
            assessment.condition = Condition::Suspected;
            assessment.reason = static_cast<Reason>(i);
            assessment.suspected_since_ms = candidate_since_ms_[i];
            break;
        }
    }
    return assessment;
}

} // namespace Domain::Motion
