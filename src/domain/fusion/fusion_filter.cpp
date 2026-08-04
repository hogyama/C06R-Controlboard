#include "fusion_filter.h"

#include <math.h>
#include <string.h>

namespace Domain::Fusion {

namespace {

constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 2.0f * PI_F;
constexpr float MINIMUM_VARIANCE = 1.0e-9f;

float square(float value)
{
    return value * value;
}

float clampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

float norm3(const float value[3])
{
    return sqrtf(
        value[0] * value[0] +
        value[1] * value[1] +
        value[2] * value[2]);
}

float dot3(const float first[3], const float second[3])
{
    return first[0] * second[0] +
        first[1] * second[1] +
        first[2] * second[2];
}

void cross3(const float first[3], const float second[3], float result[3])
{
    result[0] = first[1] * second[2] - first[2] * second[1];
    result[1] = first[2] * second[0] - first[0] * second[2];
    result[2] = first[0] * second[1] - first[1] * second[0];
}

bool normalize3(float value[3])
{
    const float norm = norm3(value);
    if (!isfinite(norm) || norm < 1.0e-6f) return false;
    value[0] /= norm;
    value[1] /= norm;
    value[2] /= norm;
    return true;
}

uint8_t incrementSaturated(uint8_t value)
{
    return value == UINT8_MAX ? value : static_cast<uint8_t>(value + 1U);
}

bool invertMatrix(
    const float input[2][2],
    uint8_t dimension,
    float inverse[2][2])
{
    memset(inverse, 0, sizeof(float) * 4U);
    if (dimension == 1U) {
        if (!isfinite(input[0][0]) || input[0][0] <= MINIMUM_VARIANCE) {
            return false;
        }
        inverse[0][0] = 1.0f / input[0][0];
        return true;
    }
    if (dimension != 2U) return false;

    const float determinant =
        input[0][0] * input[1][1] -
        input[0][1] * input[1][0];
    if (!isfinite(determinant) || fabsf(determinant) <= MINIMUM_VARIANCE) {
        return false;
    }
    const float reciprocal = 1.0f / determinant;
    inverse[0][0] = input[1][1] * reciprocal;
    inverse[0][1] = -input[0][1] * reciprocal;
    inverse[1][0] = -input[1][0] * reciprocal;
    inverse[1][1] = input[0][0] * reciprocal;
    return true;
}

} // namespace

Filter::Filter(const Config& config)
    : config_(config)
{
    reset();
}

void Filter::reset()
{
    memset(state_, 0, sizeof(state_));
    memset(covariance_, 0, sizeof(covariance_));

    initialized_ = false;
    yaw_reference_usable_ = false;

    last_state_timestamp_ms_ = 0;
    last_predict_timestamp_ms_ = 0;
    last_gps_timestamp_ms_ = 0;
    last_gps_position_used_timestamp_ms_ = 0;
    last_yaw_aiding_timestamp_ms_ = 0;
    last_magnetic_timestamp_ms_ = 0;
    last_encoder_used_timestamp_ms_ = 0;

    have_previous_encoder_ = false;
    previous_encoder_ = {};
    gyro_interval_z_integral_rad_ = 0.0f;
    gyro_interval_duration_s_ = 0.0f;
    latest_gyro_z_rad_s_ = 0.0f;

    // 加速度をまだ得ていない間は、機体が水平という初期値を使う。
    gravity_body_unit_[0] = 0.0f;
    gravity_body_unit_[1] = 0.0f;
    gravity_body_unit_[2] = 1.0f;
    have_gravity_direction_ = false;
    have_previous_acceleration_ = false;
    memset(
        previous_acceleration_body_g_,
        0,
        sizeof(previous_acceleration_body_g_));
    previous_acceleration_timestamp_ms_ = 0;

    have_magnetic_reference_strength_ = false;
    magnetic_reference_strength_uT_ = 0.0f;

    gps_health_ = {};
    magnetic_health_ = {};
    encoder_health_ = {};

    gps_resnap_sample_count_ = 0;
    gps_resnap_anchor_x_mm_ = 0.0f;
    gps_resnap_anchor_y_mm_ = 0.0f;
    gps_resnap_sum_x_mm_ = 0.0f;
    gps_resnap_sum_y_mm_ = 0.0f;

    cycle_status_flags_ = STATUS_NONE;
}

void Filter::initialize(
    int32_t x_mm,
    int32_t y_mm,
    float yaw_rad,
    bool yaw_usable,
    uint32_t timestamp_ms,
    uint32_t position_std_mm)
{
    reset();

    state_[POSITION_X] = static_cast<float>(x_mm);
    state_[POSITION_Y] = static_cast<float>(y_mm);
    state_[YAW] = normalizePi(yaw_rad);

    const float position_std = clampFloat(
        static_cast<float>(position_std_mm > 0U
            ? position_std_mm
            : config_.minimum_gps_position_std_mm),
        static_cast<float>(config_.minimum_gps_position_std_mm),
        static_cast<float>(config_.maximum_gps_position_std_mm));

    covariance_[POSITION_X][POSITION_X] = square(position_std);
    covariance_[POSITION_Y][POSITION_Y] = square(position_std);
    covariance_[YAW][YAW] = square(yaw_usable ? 0.5f : 3.0f);
    covariance_[FORWARD_VELOCITY][FORWARD_VELOCITY] = square(1000.0f);
    covariance_[GYRO_BIAS_Z][GYRO_BIAS_Z] = square(0.20f);

    initialized_ = true;
    yaw_reference_usable_ = yaw_usable;
    last_state_timestamp_ms_ = timestamp_ms;
    last_predict_timestamp_ms_ = timestamp_ms;
    last_gps_position_used_timestamp_ms_ = timestamp_ms;
    if (yaw_usable) last_yaw_aiding_timestamp_ms_ = timestamp_ms;

    cycle_status_flags_ = STATUS_INITIALIZED | STATUS_POSITION_USABLE;
    if (yaw_usable) cycle_status_flags_ |= STATUS_YAW_USABLE;
}

bool Filter::initialized() const
{
    return initialized_;
}

void Filter::beginCycle()
{
    cycle_status_flags_ = STATUS_NONE;
}

float Filter::normalizePi(float angle)
{
    angle = fmodf(angle + PI_F, TWO_PI_F);
    if (angle < 0.0f) angle += TWO_PI_F;
    return angle - PI_F;
}

float Filter::normalizeTwoPi(float angle)
{
    angle = fmodf(angle, TWO_PI_F);
    if (angle < 0.0f) angle += TWO_PI_F;
    return angle;
}

int32_t Filter::signedTimeDifference(uint32_t first, uint32_t second)
{
    return static_cast<int32_t>(first - second);
}

bool Filter::predict(const ImuObservation& observation)
{
    if (!initialized_ || observation.timestamp_ms == 0U ||
        !isfinite(observation.accel_x_g) ||
        !isfinite(observation.accel_y_g) ||
        !isfinite(observation.accel_z_g) ||
        !isfinite(observation.gyro_x_rad_s) ||
        !isfinite(observation.gyro_y_rad_s) ||
        !isfinite(observation.gyro_z_rad_s)) {
        cycle_status_flags_ |= STATUS_IMU_REJECTED;
        return false;
    }

    const int32_t dt_ms_signed = signedTimeDifference(
        observation.timestamp_ms,
        last_predict_timestamp_ms_);
    if (dt_ms_signed <= 0) {
        cycle_status_flags_ |= STATUS_IMU_REJECTED;
        return false;
    }

    const uint32_t dt_ms = static_cast<uint32_t>(dt_ms_signed);
    if (dt_ms > config_.maximum_prediction_gap_ms) {
        // 長時間欠落を古い1サンプルで埋めず、時刻だけを再同期する。
        inflateCovariance(4.0f, 4.0f);
        last_predict_timestamp_ms_ = observation.timestamp_ms;
        last_state_timestamp_ms_ = observation.timestamp_ms;
        gyro_interval_z_integral_rad_ = 0.0f;
        gyro_interval_duration_s_ = 0.0f;
        latest_gyro_z_rad_s_ = observation.gyro_z_rad_s;
        cycle_status_flags_ |= STATUS_DEGRADED | STATUS_IMU_REJECTED;
        return false;
    }

    const uint32_t maximum_step_ms =
        config_.maximum_prediction_step_ms > 0U
            ? config_.maximum_prediction_step_ms : 1U;
    const uint32_t step_count =
        (dt_ms + maximum_step_ms - 1U) / maximum_step_ms;
    const float step_dt_s =
        static_cast<float>(dt_ms) * 0.001f /
        static_cast<float>(step_count);
    for (uint32_t step = 0; step < step_count; ++step) {
        propagateStep(observation.gyro_z_rad_s, step_dt_s);
    }

    // 車輪が来ない間は位置を動かさないが、確信度だけは時間とともに下げる。
    const int32_t encoder_age_ms = signedTimeDifference(
        observation.timestamp_ms,
        last_encoder_used_timestamp_ms_);
    if (last_encoder_used_timestamp_ms_ == 0U ||
        encoder_age_ms < 0 ||
        static_cast<uint32_t>(encoder_age_ms) > config_.encoder_stale_ms) {
        const float variance =
            square(config_.missing_encoder_position_noise_mm_sqrt_s) *
            static_cast<float>(dt_ms) * 0.001f;
        covariance_[POSITION_X][POSITION_X] += variance;
        covariance_[POSITION_Y][POSITION_Y] += variance;
    }

    latest_gyro_z_rad_s_ = observation.gyro_z_rad_s;
    gyro_interval_z_integral_rad_ +=
        observation.gyro_z_rad_s *
        static_cast<float>(dt_ms) * 0.001f;
    gyro_interval_duration_s_ +=
        static_cast<float>(dt_ms) * 0.001f;

    last_predict_timestamp_ms_ = observation.timestamp_ms;
    last_state_timestamp_ms_ = observation.timestamp_ms;
    updateGravityDirection(observation);
    stabilizeCovariance();
    cycle_status_flags_ |= STATUS_IMU_USED;
    return true;
}

void Filter::propagateStep(float gyro_z_rad_s, float dt_s)
{
    state_[YAW] = normalizePi(
        state_[YAW] +
        (gyro_z_rad_s - state_[GYRO_BIAS_Z]) * dt_s);

    float transition[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        transition[state][state] = 1.0f;
    }
    transition[YAW][GYRO_BIAS_Z] = -dt_s;

    float intermediate[STATE_COUNT][STATE_COUNT] = {};
    float predicted[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                intermediate[row][column] +=
                    transition[row][k] * covariance_[k][column];
            }
        }
    }
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                predicted[row][column] +=
                    intermediate[row][k] * transition[column][k];
            }
        }
    }

    predicted[YAW][YAW] += square(config_.gyro_noise_rad_s) * dt_s;
    predicted[GYRO_BIAS_Z][GYRO_BIAS_Z] +=
        square(config_.gyro_bias_walk_rad_s2) * dt_s;
    predicted[FORWARD_VELOCITY][FORWARD_VELOCITY] +=
        square(config_.velocity_process_noise_mm_s2) * dt_s;
    memcpy(covariance_, predicted, sizeof(covariance_));
    stabilizeCovariance();
}

void Filter::updateGravityDirection(const ImuObservation& observation)
{
    const float acceleration[3] = {
        observation.accel_x_g,
        observation.accel_y_g,
        observation.accel_z_g
    };
    const float acceleration_norm = norm3(acceleration);

    float jerk_g_s = 0.0f;
    bool jerk_valid = false;
    if (have_previous_acceleration_) {
        const int32_t dt_ms = signedTimeDifference(
            observation.timestamp_ms,
            previous_acceleration_timestamp_ms_);
        if (dt_ms > 0 && dt_ms <= 500) {
            const float difference[3] = {
                acceleration[0] - previous_acceleration_body_g_[0],
                acceleration[1] - previous_acceleration_body_g_[1],
                acceleration[2] - previous_acceleration_body_g_[2]
            };
            jerk_g_s = norm3(difference) /
                (static_cast<float>(dt_ms) * 0.001f);
            jerk_valid = true;
        }
    }

    memcpy(
        previous_acceleration_body_g_,
        acceleration,
        sizeof(previous_acceleration_body_g_));
    previous_acceleration_timestamp_ms_ = observation.timestamp_ms;
    have_previous_acceleration_ = true;

    if (!isfinite(acceleration_norm) || acceleration_norm < 0.1f ||
        fabsf(acceleration_norm - 1.0f) > config_.gravity_norm_tolerance_g ||
        (jerk_valid && jerk_g_s > config_.gravity_jerk_limit_g_s)) {
        return;
    }

    float measured[3] = {
        acceleration[0] / acceleration_norm,
        acceleration[1] / acceleration_norm,
        acceleration[2] / acceleration_norm
    };
    if (!have_gravity_direction_) {
        memcpy(gravity_body_unit_, measured, sizeof(gravity_body_unit_));
        have_gravity_direction_ = true;
        return;
    }

    const float gain = clampFloat(
        config_.gravity_direction_filter_gain,
        0.0f,
        1.0f);
    for (uint8_t axis = 0; axis < 3; ++axis) {
        gravity_body_unit_[axis] =
            (1.0f - gain) * gravity_body_unit_[axis] +
            gain * measured[axis];
    }
    normalize3(gravity_body_unit_);
}

bool Filter::updateEncoder(const EncoderObservation& observation)
{
    if (!initialized_ || observation.timestamp_ms == 0U ||
        config_.track_width_mm <= 0.0f) {
        cycle_status_flags_ |= STATUS_ENCODER_REJECTED;
        return false;
    }

    if (!have_previous_encoder_) {
        previous_encoder_ = observation;
        have_previous_encoder_ = true;
        gyro_interval_z_integral_rad_ = 0.0f;
        gyro_interval_duration_s_ = 0.0f;
        return false;
    }

    const int32_t dt_ms_signed = signedTimeDifference(
        observation.timestamp_ms,
        previous_encoder_.timestamp_ms);
    if (dt_ms_signed <= 0) {
        cycle_status_flags_ |= STATUS_ENCODER_REJECTED;
        noteRejected(encoder_health_, config_);
        return false;
    }

    const uint32_t dt_ms = static_cast<uint32_t>(dt_ms_signed);
    const float delta_left = static_cast<float>(
        static_cast<int64_t>(observation.left_mm) -
        static_cast<int64_t>(previous_encoder_.left_mm));
    const float delta_right = static_cast<float>(
        static_cast<int64_t>(observation.right_mm) -
        static_cast<int64_t>(previous_encoder_.right_mm));
    previous_encoder_ = observation;

    auto finishInterval = [&]() {
        gyro_interval_z_integral_rad_ = 0.0f;
        gyro_interval_duration_s_ = 0.0f;
    };

    if (dt_ms > config_.maximum_encoder_interval_ms) {
        finishInterval();
        cycle_status_flags_ |= STATUS_DEGRADED | STATUS_ENCODER_REJECTED;
        noteRejected(encoder_health_, config_);
        return false;
    }

    const float dt_s = static_cast<float>(dt_ms) * 0.001f;
    const float left_velocity = delta_left / dt_s;
    const float right_velocity = delta_right / dt_s;
    if (!isfinite(left_velocity) || !isfinite(right_velocity) ||
        fabsf(left_velocity) > config_.maximum_encoder_speed_mm_s ||
        fabsf(right_velocity) > config_.maximum_encoder_speed_mm_s) {
        finishInterval();
        cycle_status_flags_ |= STATUS_DEGRADED | STATUS_ENCODER_REJECTED;
        noteRejected(encoder_health_, config_);
        return false;
    }

    bool interval_aligned = false;
    if (gyro_interval_duration_s_ > 0.0f) {
        const uint32_t gyro_duration_ms = static_cast<uint32_t>(
            lroundf(gyro_interval_duration_s_ * 1000.0f));
        const uint32_t difference = gyro_duration_ms > dt_ms
            ? gyro_duration_ms - dt_ms
            : dt_ms - gyro_duration_ms;
        interval_aligned = difference <=
            config_.encoder_imu_alignment_tolerance_ms;
    }

    const float delta_forward = 0.5f * (delta_left + delta_right);
    const float delta_encoder_yaw =
        (delta_right - delta_left) / config_.track_width_mm;
    const float delta_gyro_yaw =
        gyro_interval_z_integral_rad_ -
        state_[GYRO_BIAS_Z] * gyro_interval_duration_s_;
    // ジャイロが区間全体を覆うときは差分だけを補正する。
    // 区間内にジャイロが1件もない場合だけ、車輪Yawをフォールバックにする。
    // 中途半端なジャイロ区間では二重加算を避けるためYawを足さない。
    const bool gyro_interval_missing = gyro_interval_duration_s_ <= 0.0f;
    const float yaw_blend = interval_aligned
        ? clampFloat(config_.encoder_yaw_blend, 0.0f, 1.0f)
        : (gyro_interval_missing ? 1.0f : 0.0f);
    const float yaw_correction = yaw_blend * normalizePi(
        delta_encoder_yaw - delta_gyro_yaw);
    state_[YAW] = normalizePi(state_[YAW] + yaw_correction);

    const float fused_delta_yaw = delta_gyro_yaw + yaw_correction;
    const float midpoint_yaw = state_[YAW] - 0.5f * fused_delta_yaw;
    const float yaw_disagreement = interval_aligned
        ? fabsf(normalizePi(delta_encoder_yaw - delta_gyro_yaw))
        : 0.0f;
    const float slip_threshold = fmaxf(
        0.10f,
        3.0f * config_.encoder_yaw_rate_std_rad_s * dt_s);
    const bool slip_likely =
        interval_aligned && yaw_disagreement > slip_threshold;

    propagateWheelMotion(
        delta_left,
        delta_right,
        delta_forward,
        midpoint_yaw,
        yaw_blend,
        slip_likely,
        dt_s);

    state_[POSITION_X] += delta_forward * cosf(midpoint_yaw);
    state_[POSITION_Y] += delta_forward * sinf(midpoint_yaw);
    state_[FORWARD_VELOCITY] = 0.5f * (left_velocity + right_velocity);

    bool yaw_rate_rejected = false;
    if (interval_aligned) {
        const float measured_yaw_rate =
            (right_velocity - left_velocity) / config_.track_width_mm;
        const float average_raw_gyro_z =
            gyro_interval_z_integral_rad_ / gyro_interval_duration_s_;
        float residual[MAX_MEASUREMENT_DIMENSION] = {};
        float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT] = {};
        float variance[MAX_MEASUREMENT_DIMENSION] = {};
        residual[0] = measured_yaw_rate -
            (average_raw_gyro_z - state_[GYRO_BIAS_Z]);
        jacobian[0][GYRO_BIAS_Z] = -1.0f;
        variance[0] = square(config_.encoder_yaw_rate_std_rad_s);
        yaw_rate_rejected = measurementUpdate(
            residual,
            jacobian,
            variance,
            1,
            config_.nis_soft_1d,
            config_.nis_hard_1d) == UpdateResult::Rejected;
    }

    finishInterval();
    noteAccepted(encoder_health_, config_);
    last_encoder_used_timestamp_ms_ = observation.timestamp_ms;
    cycle_status_flags_ |= STATUS_ENCODER_USED;
    if (yaw_rate_rejected) cycle_status_flags_ |= STATUS_ENCODER_REJECTED;
    return true;
}

void Filter::propagateWheelMotion(
    float delta_left_mm,
    float delta_right_mm,
    float delta_forward_mm,
    float midpoint_yaw_rad,
    float yaw_blend,
    bool slip_likely,
    float dt_s)
{
    float transition[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        transition[state][state] = 1.0f;
    }
    // 速度は今回の左右輪差分で置き換えるため、古い速度分散を持ち越さない。
    for (uint8_t column = 0; column < STATE_COUNT; ++column) {
        transition[FORWARD_VELOCITY][column] = 0.0f;
    }
    transition[POSITION_X][YAW] =
        -delta_forward_mm * sinf(midpoint_yaw_rad);
    transition[POSITION_Y][YAW] =
        delta_forward_mm * cosf(midpoint_yaw_rad);
    transition[YAW][GYRO_BIAS_Z] +=
        yaw_blend * gyro_interval_duration_s_;

    float intermediate[STATE_COUNT][STATE_COUNT] = {};
    float propagated[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                intermediate[row][column] +=
                    transition[row][k] * covariance_[k][column];
            }
        }
    }
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                propagated[row][column] +=
                    intermediate[row][k] * transition[column][k];
            }
        }
    }

    float wheel_variance[2] = {
        fmaxf(config_.wheel_noise_mm2_per_mm, 0.0f) * fabsf(delta_left_mm) +
            square(config_.wheel_relative_noise * delta_left_mm),
        fmaxf(config_.wheel_noise_mm2_per_mm, 0.0f) * fabsf(delta_right_mm) +
            square(config_.wheel_relative_noise * delta_right_mm)
    };
    if (slip_likely) {
        const float multiplier = fmaxf(
            config_.encoder_slip_noise_multiplier,
            1.0f);
        wheel_variance[0] *= multiplier;
        wheel_variance[1] *= multiplier;
    }

    const float half_yaw_gain =
        0.5f * yaw_blend / config_.track_width_mm;
    const float full_yaw_gain =
        yaw_blend / config_.track_width_mm;
    float wheel_jacobian[STATE_COUNT][2] = {};
    wheel_jacobian[POSITION_X][0] =
        0.5f * cosf(midpoint_yaw_rad) +
        delta_forward_mm * sinf(midpoint_yaw_rad) * half_yaw_gain;
    wheel_jacobian[POSITION_X][1] =
        0.5f * cosf(midpoint_yaw_rad) -
        delta_forward_mm * sinf(midpoint_yaw_rad) * half_yaw_gain;
    wheel_jacobian[POSITION_Y][0] =
        0.5f * sinf(midpoint_yaw_rad) -
        delta_forward_mm * cosf(midpoint_yaw_rad) * half_yaw_gain;
    wheel_jacobian[POSITION_Y][1] =
        0.5f * sinf(midpoint_yaw_rad) +
        delta_forward_mm * cosf(midpoint_yaw_rad) * half_yaw_gain;
    wheel_jacobian[YAW][0] = -full_yaw_gain;
    wheel_jacobian[YAW][1] = full_yaw_gain;
    wheel_jacobian[FORWARD_VELOCITY][0] = 0.5f / dt_s;
    wheel_jacobian[FORWARD_VELOCITY][1] = 0.5f / dt_s;

    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t wheel = 0; wheel < 2; ++wheel) {
                propagated[row][column] +=
                    wheel_jacobian[row][wheel] *
                    wheel_variance[wheel] *
                    wheel_jacobian[column][wheel];
            }
        }
    }
    memcpy(covariance_, propagated, sizeof(covariance_));
    stabilizeCovariance();
}

bool Filter::updateGps(const GpsUpdate& observation)
{
    if (!initialized_ || observation.timestamp_ms == 0U) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        return false;
    }
    if (observation.timestamp_ms == last_gps_timestamp_ms_) return false;
    if (last_gps_timestamp_ms_ != 0U &&
        signedTimeDifference(
            observation.timestamp_ms,
            last_gps_timestamp_ms_) <= 0) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        noteRejected(gps_health_, config_);
        return false;
    }
    last_gps_timestamp_ms_ = observation.timestamp_ms;

    if (!observation.fix_ok || observation.fix_type < 2U ||
        observation.horizontal_accuracy_mm == 0U ||
        observation.horizontal_accuracy_mm >
            config_.maximum_gps_position_std_mm) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        noteRejected(gps_health_, config_);
        return false;
    }

    const int32_t age_ms = signedTimeDifference(
        last_state_timestamp_ms_,
        observation.timestamp_ms);
    if (age_ms > static_cast<int32_t>(config_.maximum_delayed_gps_ms) ||
        age_ms < -static_cast<int32_t>(config_.maximum_future_measurement_ms)) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        noteRejected(gps_health_, config_);
        return false;
    }

    // NAV-PVT速度があるときだけ、遅延位置を現在時刻へ投影する。
    const float projection_dt_s =
        observation.velocity_valid && age_ms > 0
            ? static_cast<float>(age_ms) * 0.001f
            : 0.0f;
    const float gps_x = static_cast<float>(observation.x_mm) +
        static_cast<float>(observation.velocity_east_mm_s) * projection_dt_s;
    const float gps_y = static_cast<float>(observation.y_mm) +
        static_cast<float>(observation.velocity_north_mm_s) * projection_dt_s;
    if (!isfinite(gps_x) || !isfinite(gps_y)) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        noteRejected(gps_health_, config_);
        return false;
    }

    const float position_std = clampFloat(
        static_cast<float>(observation.horizontal_accuracy_mm),
        static_cast<float>(config_.minimum_gps_position_std_mm),
        static_cast<float>(config_.maximum_gps_position_std_mm));
    const float position_residual = hypotf(
        gps_x - state_[POSITION_X],
        gps_y - state_[POSITION_Y]);

    if (position_residual > static_cast<float>(config_.gps_resnap_distance_mm)) {
        if (observation.horizontal_accuracy_mm >
            config_.gps_resnap_maximum_accuracy_mm) {
            gps_resnap_sample_count_ = 0;
            cycle_status_flags_ |= STATUS_GPS_REJECTED;
            noteRejected(gps_health_, config_);
            return false;
        }

        const float spread = hypotf(
            gps_x - gps_resnap_anchor_x_mm_,
            gps_y - gps_resnap_anchor_y_mm_);
        if (gps_resnap_sample_count_ == 0U ||
            spread > static_cast<float>(config_.gps_resnap_stable_radius_mm)) {
            gps_resnap_sample_count_ = 1;
            gps_resnap_anchor_x_mm_ = gps_x;
            gps_resnap_anchor_y_mm_ = gps_y;
            gps_resnap_sum_x_mm_ = gps_x;
            gps_resnap_sum_y_mm_ = gps_y;
        } else {
            gps_resnap_sample_count_++;
            gps_resnap_sum_x_mm_ += gps_x;
            gps_resnap_sum_y_mm_ += gps_y;
        }

        if (gps_resnap_sample_count_ < config_.gps_resnap_required_samples) {
            cycle_status_flags_ |= STATUS_GPS_REJECTED;
            return false;
        }

        state_[POSITION_X] =
            gps_resnap_sum_x_mm_ / gps_resnap_sample_count_;
        state_[POSITION_Y] =
            gps_resnap_sum_y_mm_ / gps_resnap_sample_count_;
        resetStateCovariance(POSITION_X, square(position_std));
        resetStateCovariance(POSITION_Y, square(position_std));

        if (observation.velocity_valid) {
            const float speed = hypotf(
                static_cast<float>(observation.velocity_east_mm_s),
                static_cast<float>(observation.velocity_north_mm_s));
            state_[FORWARD_VELOCITY] =
                state_[FORWARD_VELOCITY] < 0.0f ? -speed : speed;
            const float speed_std = clampFloat(
                static_cast<float>(observation.speed_accuracy_mm_s),
                static_cast<float>(config_.minimum_gps_speed_std_mm_s),
                static_cast<float>(config_.maximum_gps_speed_std_mm_s));
            resetStateCovariance(
                FORWARD_VELOCITY,
                square(speed_std));
        }

        gps_resnap_sample_count_ = 0;
        last_gps_position_used_timestamp_ms_ = last_state_timestamp_ms_;
        noteAccepted(gps_health_, config_);
        cycle_status_flags_ |= STATUS_GPS_USED;
        return true;
    }
    gps_resnap_sample_count_ = 0;

    float residual[MAX_MEASUREMENT_DIMENSION] = {};
    float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT] = {};
    float variance[MAX_MEASUREMENT_DIMENSION] = {};
    residual[0] = gps_x - state_[POSITION_X];
    residual[1] = gps_y - state_[POSITION_Y];
    jacobian[0][POSITION_X] = 1.0f;
    jacobian[1][POSITION_Y] = 1.0f;
    variance[0] = square(position_std);
    variance[1] = square(position_std);
    const UpdateResult position_result = measurementUpdate(
        residual,
        jacobian,
        variance,
        2,
        config_.nis_soft_2d,
        config_.nis_hard_2d);

    UpdateResult speed_result = UpdateResult::Rejected;
    UpdateResult course_result = UpdateResult::Rejected;
    bool course_attempted = false;
    const float predicted_forward_velocity = state_[FORWARD_VELOCITY];
    const float measured_east =
        static_cast<float>(observation.velocity_east_mm_s);
    const float measured_north =
        static_cast<float>(observation.velocity_north_mm_s);
    const float measured_speed = hypotf(measured_east, measured_north);
    const float speed_std = clampFloat(
        static_cast<float>(observation.speed_accuracy_mm_s),
        static_cast<float>(config_.minimum_gps_speed_std_mm_s),
        static_cast<float>(config_.maximum_gps_speed_std_mm_s));
    const bool velocity_usable =
        observation.velocity_valid &&
        measured_speed >= static_cast<float>(config_.minimum_gps_velocity_mm_s) &&
        observation.speed_accuracy_mm_s <= config_.maximum_gps_speed_std_mm_s;

    if (velocity_usable) {
        memset(residual, 0, sizeof(residual));
        memset(jacobian, 0, sizeof(jacobian));
        memset(variance, 0, sizeof(variance));
        const float signed_speed =
            state_[FORWARD_VELOCITY] < 0.0f
                ? -measured_speed : measured_speed;
        residual[0] = signed_speed - state_[FORWARD_VELOCITY];
        jacobian[0][FORWARD_VELOCITY] = 1.0f;
        variance[0] = square(speed_std);
        speed_result = measurementUpdate(
            residual,
            jacobian,
            variance,
            1,
            config_.nis_soft_1d,
            config_.nis_hard_1d);

        // courseは進行方向なので、後退中は機首Yawとして使用しない。
        if (predicted_forward_velocity > 0.0f) {
            const float course_std = clampFloat(
                speed_std / measured_speed,
                config_.minimum_gps_course_std_rad,
                config_.maximum_gps_course_std_rad);
            if (isfinite(course_std)) {
                course_attempted = true;
                const float measured_course = atan2f(measured_north, measured_east);
                memset(residual, 0, sizeof(residual));
                memset(jacobian, 0, sizeof(jacobian));
                memset(variance, 0, sizeof(variance));
                residual[0] = normalizePi(measured_course - state_[YAW]);
                jacobian[0][YAW] = 1.0f;
                variance[0] = square(course_std);
                course_result = measurementUpdate(
                    residual,
                    jacobian,
                    variance,
                    1,
                    config_.nis_soft_1d,
                    config_.nis_hard_1d);
            }
        }
    }

    const bool position_used = position_result != UpdateResult::Rejected;
    const bool speed_used = speed_result != UpdateResult::Rejected;
    const bool course_used = course_result != UpdateResult::Rejected;
    if (position_used || speed_used || course_used) {
        noteAccepted(gps_health_, config_);
        if (position_used) {
            last_gps_position_used_timestamp_ms_ = last_state_timestamp_ms_;
        }
        if (course_used) {
            yaw_reference_usable_ = true;
            last_yaw_aiding_timestamp_ms_ = last_state_timestamp_ms_;
        }
        if (!position_used || (course_attempted && !course_used)) {
            cycle_status_flags_ |= STATUS_GPS_REJECTED;
        }
        cycle_status_flags_ |= STATUS_GPS_USED;
        return true;
    }

    noteRejected(gps_health_, config_);
    cycle_status_flags_ |= STATUS_GPS_REJECTED;
    return false;
}

bool Filter::updateMagnetic(const MagneticObservation& observation)
{
    if (!initialized_ || observation.timestamp_ms == 0U ||
        !isfinite(observation.x_uT) ||
        !isfinite(observation.y_uT) ||
        !isfinite(observation.z_uT)) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        return false;
    }
    if (observation.timestamp_ms == last_magnetic_timestamp_ms_) return false;
    if (last_magnetic_timestamp_ms_ != 0U &&
        signedTimeDifference(
            observation.timestamp_ms,
            last_magnetic_timestamp_ms_) <= 0) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        noteRejected(magnetic_health_, config_);
        return false;
    }
    last_magnetic_timestamp_ms_ = observation.timestamp_ms;

    const float magnetic[3] = {
        observation.x_uT,
        observation.y_uT,
        observation.z_uT
    };
    const float total_strength = norm3(magnetic);
    if (!isfinite(total_strength) ||
        total_strength < config_.magnetic_min_total_uT ||
        total_strength > config_.magnetic_max_total_uT) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        noteRejected(magnetic_health_, config_);
        return false;
    }

    if (fabsf(state_[FORWARD_VELOCITY]) >
            config_.magnetic_moving_max_speed_mm_s ||
        fabsf(yawRate()) > config_.magnetic_moving_max_yaw_rate_rad_s) {
        return false;
    }
    const bool stationary =
        fabsf(state_[FORWARD_VELOCITY]) <=
            config_.magnetic_stationary_max_speed_mm_s &&
        fabsf(yawRate()) <=
            config_.magnetic_stationary_max_yaw_rate_rad_s;
    if (stationary && observation.motor_command_active) return false;

    if (have_magnetic_reference_strength_) {
        const float relative_error = fabsf(
            total_strength - magnetic_reference_strength_uT_) /
            magnetic_reference_strength_uT_;
        if (relative_error > config_.magnetic_strength_relative_tolerance) {
            cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
            noteRejected(magnetic_health_, config_);
            return false;
        }
    }

    // 重力方向へ直交する前・左基底を作り、3軸地磁気を水平面へ射影する。
    float gravity[3] = {
        gravity_body_unit_[0],
        gravity_body_unit_[1],
        gravity_body_unit_[2]
    };
    if (!normalize3(gravity)) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        return false;
    }
    const float body_forward[3] = {1.0f, 0.0f, 0.0f};
    const float forward_vertical = dot3(body_forward, gravity);
    float horizontal_forward[3] = {
        body_forward[0] - forward_vertical * gravity[0],
        body_forward[1] - forward_vertical * gravity[1],
        body_forward[2] - forward_vertical * gravity[2]
    };
    if (!normalize3(horizontal_forward)) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        return false;
    }
    float horizontal_left[3] = {};
    cross3(gravity, horizontal_forward, horizontal_left);
    if (!normalize3(horizontal_left)) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        return false;
    }

    const float magnetic_vertical = dot3(magnetic, gravity);
    const float horizontal_magnetic[3] = {
        magnetic[0] - magnetic_vertical * gravity[0],
        magnetic[1] - magnetic_vertical * gravity[1],
        magnetic[2] - magnetic_vertical * gravity[2]
    };
    const float magnetic_forward = dot3(horizontal_magnetic, horizontal_forward);
    const float magnetic_left = dot3(horizontal_magnetic, horizontal_left);
    const float horizontal_strength = hypotf(magnetic_forward, magnetic_left);
    if (!isfinite(horizontal_strength) ||
        horizontal_strength < config_.magnetic_min_horizontal_uT) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        noteRejected(magnetic_health_, config_);
        return false;
    }

    const float expected_field_azimuth =
        0.5f * PI_F - config_.magnetic_declination_rad;
    const float field_relative_to_vehicle =
        atan2f(magnetic_left, magnetic_forward);
    const float measured_yaw = normalizePi(
        expected_field_azimuth - field_relative_to_vehicle);

    float residual[MAX_MEASUREMENT_DIMENSION] = {};
    float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT] = {};
    float variance[MAX_MEASUREMENT_DIMENSION] = {};
    residual[0] = normalizePi(measured_yaw - state_[YAW]);
    jacobian[0][YAW] = 1.0f;
    variance[0] = square(stationary
        ? config_.magnetic_stationary_yaw_std_rad
        : config_.magnetic_moving_yaw_std_rad);
    const UpdateResult result = measurementUpdate(
        residual,
        jacobian,
        variance,
        1,
        config_.nis_soft_1d,
        config_.nis_hard_1d);
    if (result == UpdateResult::Rejected) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        noteRejected(magnetic_health_, config_);
        return false;
    }

    if (!have_magnetic_reference_strength_) {
        magnetic_reference_strength_uT_ = total_strength;
        have_magnetic_reference_strength_ = true;
    } else {
        magnetic_reference_strength_uT_ =
            0.98f * magnetic_reference_strength_uT_ +
            0.02f * total_strength;
    }
    noteAccepted(magnetic_health_, config_);
    yaw_reference_usable_ = true;
    last_yaw_aiding_timestamp_ms_ = observation.timestamp_ms;
    cycle_status_flags_ |= STATUS_MAGNETIC_USED;
    return true;
}

Filter::UpdateResult Filter::measurementUpdate(
    const float residual[MAX_MEASUREMENT_DIMENSION],
    const float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT],
    const float variance[MAX_MEASUREMENT_DIMENSION],
    uint8_t dimension,
    float soft_gate,
    float hard_gate)
{
    if (dimension == 0U || dimension > MAX_MEASUREMENT_DIMENSION ||
        soft_gate <= 0.0f || hard_gate <= soft_gate) {
        return UpdateResult::Rejected;
    }

    float effective_variance[MAX_MEASUREMENT_DIMENSION] = {};
    for (uint8_t i = 0; i < dimension; ++i) {
        if (!isfinite(residual[i]) || !isfinite(variance[i]) ||
            variance[i] <= 0.0f) {
            return UpdateResult::Rejected;
        }
        effective_variance[i] = variance[i];
    }

    float projected[STATE_COUNT][MAX_MEASUREMENT_DIMENSION] = {};
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        for (uint8_t measurement = 0; measurement < dimension; ++measurement) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                projected[state][measurement] +=
                    covariance_[state][k] * jacobian[measurement][k];
            }
        }
    }

    auto calculateInnovation = [&] (
        float innovation[2][2],
        float inverse[2][2],
        float& nis) -> bool {
        memset(innovation, 0, sizeof(float) * 4U);
        for (uint8_t row = 0; row < dimension; ++row) {
            for (uint8_t column = 0; column < dimension; ++column) {
                for (uint8_t state = 0; state < STATE_COUNT; ++state) {
                    innovation[row][column] +=
                        jacobian[row][state] * projected[state][column];
                }
            }
            innovation[row][row] += effective_variance[row];
        }
        if (!invertMatrix(innovation, dimension, inverse)) return false;
        nis = 0.0f;
        for (uint8_t row = 0; row < dimension; ++row) {
            for (uint8_t column = 0; column < dimension; ++column) {
                nis += residual[row] * inverse[row][column] * residual[column];
            }
        }
        return isfinite(nis);
    };

    float innovation[2][2] = {};
    float innovation_inverse[2][2] = {};
    float nis = 0.0f;
    if (!calculateInnovation(innovation, innovation_inverse, nis) ||
        nis > hard_gate) {
        return UpdateResult::Rejected;
    }

    UpdateResult result = UpdateResult::Accepted;
    if (nis > soft_gate) {
        const float scale = nis / soft_gate;
        for (uint8_t i = 0; i < dimension; ++i) {
            effective_variance[i] *= scale;
        }
        if (!calculateInnovation(innovation, innovation_inverse, nis)) {
            return UpdateResult::Rejected;
        }
        result = UpdateResult::SoftAccepted;
    }

    float gain[STATE_COUNT][MAX_MEASUREMENT_DIMENSION] = {};
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        for (uint8_t measurement = 0; measurement < dimension; ++measurement) {
            for (uint8_t k = 0; k < dimension; ++k) {
                gain[state][measurement] +=
                    projected[state][k] * innovation_inverse[k][measurement];
            }
        }
    }

    float correction[STATE_COUNT] = {};
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        for (uint8_t measurement = 0; measurement < dimension; ++measurement) {
            correction[state] += gain[state][measurement] * residual[measurement];
        }
    }

    // Joseph形式で単精度でも対称性と半正定値性を保つ。
    float identity_minus_kh[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        identity_minus_kh[row][row] = 1.0f;
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t measurement = 0; measurement < dimension; ++measurement) {
                identity_minus_kh[row][column] -=
                    gain[row][measurement] * jacobian[measurement][column];
            }
        }
    }

    float intermediate[STATE_COUNT][STATE_COUNT] = {};
    float updated[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                intermediate[row][column] +=
                    identity_minus_kh[row][k] * covariance_[k][column];
            }
        }
    }
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                updated[row][column] +=
                    intermediate[row][k] * identity_minus_kh[column][k];
            }
            for (uint8_t measurement = 0; measurement < dimension; ++measurement) {
                updated[row][column] +=
                    gain[row][measurement] *
                    effective_variance[measurement] *
                    gain[column][measurement];
            }
        }
    }

    memcpy(covariance_, updated, sizeof(covariance_));
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        state_[state] += correction[state];
    }
    state_[YAW] = normalizePi(state_[YAW]);
    stabilizeCovariance();
    return result;
}

void Filter::resetStateCovariance(uint8_t state_index, float variance)
{
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        covariance_[state_index][state] = 0.0f;
        covariance_[state][state_index] = 0.0f;
    }
    covariance_[state_index][state_index] = fmaxf(variance, MINIMUM_VARIANCE);
}

void Filter::stabilizeCovariance()
{
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = static_cast<uint8_t>(row + 1U);
             column < STATE_COUNT;
             ++column) {
            float symmetric = 0.5f * (
                covariance_[row][column] + covariance_[column][row]);
            if (!isfinite(symmetric)) symmetric = 0.0f;
            covariance_[row][column] = symmetric;
            covariance_[column][row] = symmetric;
        }
        if (!isfinite(covariance_[row][row]) ||
            covariance_[row][row] < MINIMUM_VARIANCE) {
            covariance_[row][row] = MINIMUM_VARIANCE;
        }
    }
}

void Filter::inflateCovariance(float position_factor, float yaw_factor)
{
    position_factor = fmaxf(position_factor, 1.0f);
    yaw_factor = fmaxf(yaw_factor, 1.0f);
    covariance_[POSITION_X][POSITION_X] *= position_factor;
    covariance_[POSITION_Y][POSITION_Y] *= position_factor;
    covariance_[FORWARD_VELOCITY][FORWARD_VELOCITY] *= position_factor;
    covariance_[YAW][YAW] *= yaw_factor;
    covariance_[GYRO_BIAS_Z][GYRO_BIAS_Z] *= yaw_factor;
    stabilizeCovariance();
}

void Filter::noteAccepted(HealthTracker& tracker, const Config& config)
{
    tracker.rejection_count = 0;
    if (!tracker.failed) {
        tracker.acceptance_count = 0;
        return;
    }
    tracker.acceptance_count = incrementSaturated(tracker.acceptance_count);
    if (tracker.acceptance_count >= config.health_recovery_accepts) {
        tracker.failed = false;
        tracker.acceptance_count = 0;
    }
}

void Filter::noteRejected(HealthTracker& tracker, const Config& config)
{
    tracker.acceptance_count = 0;
    tracker.rejection_count = incrementSaturated(tracker.rejection_count);
    if (tracker.rejection_count >= config.health_failure_rejections) {
        tracker.failed = true;
    }
}

SensorHealth Filter::sensorHealthAt(
    uint32_t now_ms,
    uint32_t last_timestamp_ms,
    uint32_t stale_ms,
    uint32_t failed_ms)
{
    if (last_timestamp_ms == 0U) return SensorHealth::Failed;
    const uint32_t age_ms = static_cast<uint32_t>(now_ms - last_timestamp_ms);
    if (age_ms <= stale_ms) return SensorHealth::Fresh;
    if (age_ms <= failed_ms) return SensorHealth::Stale;
    return SensorHealth::Failed;
}

bool Filter::positionUsableAt(uint32_t timestamp_ms) const
{
    (void)timestamp_ms;
    if (!initialized_ || last_gps_position_used_timestamp_ms_ == 0U) {
        return false;
    }
    const float position_std = sqrtf(fmaxf(
        covariance_[POSITION_X][POSITION_X],
        covariance_[POSITION_Y][POSITION_Y]));
    return isfinite(position_std) &&
        position_std <= static_cast<float>(config_.maximum_usable_position_std_mm);
}

bool Filter::yawUsableAt(uint32_t timestamp_ms) const
{
    if (!initialized_ || !yaw_reference_usable_) return false;

    const int32_t imu_age = signedTimeDifference(
        timestamp_ms,
        last_predict_timestamp_ms_);
    if (imu_age < 0 ||
        static_cast<uint32_t>(imu_age) > config_.maximum_imu_age_ms) {
        return false;
    }
    const int32_t aiding_age = signedTimeDifference(
        timestamp_ms,
        last_yaw_aiding_timestamp_ms_);
    if (last_yaw_aiding_timestamp_ms_ == 0U || aiding_age < 0 ||
        static_cast<uint32_t>(aiding_age) > config_.maximum_yaw_aiding_age_ms) {
        return false;
    }
    const float yaw_std = sqrtf(fmaxf(covariance_[YAW][YAW], MINIMUM_VARIANCE));
    return isfinite(yaw_std) && yaw_std <= config_.maximum_usable_yaw_std_rad;
}

float Filter::yawRate() const
{
    return latest_gyro_z_rad_s_ - state_[GYRO_BIAS_Z];
}

Output Filter::output(uint32_t timestamp_ms) const
{
    Output result{};
    result.timestamp_ms = last_state_timestamp_ms_;
    result.x_mm = static_cast<int32_t>(lroundf(state_[POSITION_X]));
    result.y_mm = static_cast<int32_t>(lroundf(state_[POSITION_Y]));
    result.yaw_rad = normalizeTwoPi(state_[YAW]);
    result.forward_velocity_mm_s = state_[FORWARD_VELOCITY];
    result.yaw_rate_rad_s = yawRate();
    result.position_std_mm = static_cast<uint32_t>(lroundf(sqrtf(fmaxf(
        covariance_[POSITION_X][POSITION_X],
        covariance_[POSITION_Y][POSITION_Y]))));
    result.yaw_std_rad = sqrtf(fmaxf(covariance_[YAW][YAW], MINIMUM_VARIANCE));

    result.status_flags = cycle_status_flags_;
    if (initialized_) result.status_flags |= STATUS_INITIALIZED;
    if (positionUsableAt(timestamp_ms)) result.status_flags |= STATUS_POSITION_USABLE;
    if (yawUsableAt(timestamp_ms)) result.status_flags |= STATUS_YAW_USABLE;
    if (gps_health_.failed) result.status_flags |= STATUS_GPS_UNHEALTHY;
    if (magnetic_health_.failed) result.status_flags |= STATUS_MAGNETIC_UNHEALTHY;
    if (encoder_health_.failed) result.status_flags |= STATUS_ENCODER_UNHEALTHY;
    if (result.x_mm < 0 || result.y_mm < 0 ||
        result.x_mm > config_.field_size_x_mm ||
        result.y_mm > config_.field_size_y_mm) {
        result.status_flags |= STATUS_OUTSIDE_FIELD;
    }

    result.gps_health = sensorHealthAt(
        timestamp_ms,
        last_gps_timestamp_ms_,
        config_.gps_stale_ms,
        config_.gps_failed_ms);
    result.encoder_health = sensorHealthAt(
        timestamp_ms,
        previous_encoder_.timestamp_ms,
        config_.encoder_stale_ms,
        config_.encoder_failed_ms);
    result.imu_health = sensorHealthAt(
        timestamp_ms,
        last_predict_timestamp_ms_,
        config_.imu_stale_ms,
        config_.imu_failed_ms);
    result.magnetic_health = sensorHealthAt(
        timestamp_ms,
        last_magnetic_timestamp_ms_,
        config_.magnetic_stale_ms,
        config_.magnetic_failed_ms);
    if (gps_health_.failed) result.gps_health = SensorHealth::Failed;
    if (encoder_health_.failed) result.encoder_health = SensorHealth::Failed;
    if (magnetic_health_.failed) result.magnetic_health = SensorHealth::Failed;

    const bool position_usable = positionUsableAt(timestamp_ms);
    const bool yaw_usable = yawUsableAt(timestamp_ms);
    if (!position_usable || !yaw_usable ||
        result.gps_health == SensorHealth::Failed ||
        result.encoder_health == SensorHealth::Failed ||
        result.imu_health == SensorHealth::Failed ||
        result.magnetic_health == SensorHealth::Failed) {
        result.status_flags |= STATUS_DEGRADED;
    }

    result.anomaly_flags = ANOMALY_NONE;
    result.anomaly_since_ms = 0;
    if (!initialized_) {
        result.quality = Quality::Uninitialized;
    } else if (!position_usable || !yaw_usable ||
               !isfinite(result.yaw_std_rad)) {
        result.quality = Quality::Failed;
    } else if (result.position_std_mm > 8000U ||
               result.yaw_std_rad > 0.52f) {
        result.quality = Quality::Unreliable;
    } else if (result.position_std_mm > 5000U ||
               result.yaw_std_rad > 0.35f ||
               result.gps_health == SensorHealth::Failed ||
               result.encoder_health == SensorHealth::Failed ||
               result.imu_health == SensorHealth::Failed ||
               result.magnetic_health == SensorHealth::Failed) {
        result.quality = Quality::Degraded;
    } else {
        result.quality = Quality::Normal;
    }
    return result;
}

} // namespace Domain::Fusion
