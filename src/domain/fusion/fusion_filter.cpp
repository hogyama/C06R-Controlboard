#include "fusion_filter.h"

#include <math.h>
#include <string.h>

namespace Domain::Fusion {

namespace {

constexpr float PI_F = 3.14159265358979323846f;
constexpr float TWO_PI_F = 2.0f * PI_F;
constexpr float MINIMUM_VARIANCE = 1.0e-9f;
constexpr float MINIMUM_QUATERNION_NORM = 1.0e-12f;

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

uint8_t incrementSaturated(uint8_t value)
{
    return value == UINT8_MAX ? value : static_cast<uint8_t>(value + 1U);
}

float vectorNorm3(const float value[3])
{
    return sqrtf(
        value[0] * value[0] +
        value[1] * value[1] +
        value[2] * value[2]);
}

void skewMatrix(const float vector[3], float result[3][3])
{
    result[0][0] = 0.0f;
    result[0][1] = -vector[2];
    result[0][2] = vector[1];
    result[1][0] = vector[2];
    result[1][1] = 0.0f;
    result[1][2] = -vector[0];
    result[2][0] = -vector[1];
    result[2][1] = vector[0];
    result[2][2] = 0.0f;
}

bool invertSmallMatrix(
    const float input[3][3],
    uint8_t dimension,
    float inverse[3][3])
{
    if (dimension == 0 || dimension > 3) return false;

    float augmented[3][6] = {};
    for (uint8_t row = 0; row < dimension; ++row) {
        for (uint8_t column = 0; column < dimension; ++column) {
            augmented[row][column] = input[row][column];
        }
        augmented[row][dimension + row] = 1.0f;
    }

    for (uint8_t pivot_column = 0;
         pivot_column < dimension;
         ++pivot_column) {
        uint8_t pivot_row = pivot_column;
        float pivot_magnitude = fabsf(
            augmented[pivot_row][pivot_column]);
        for (uint8_t row = static_cast<uint8_t>(pivot_column + 1U);
             row < dimension;
             ++row) {
            const float candidate = fabsf(
                augmented[row][pivot_column]);
            if (candidate > pivot_magnitude) {
                pivot_magnitude = candidate;
                pivot_row = row;
            }
        }

        if (!isfinite(pivot_magnitude) ||
            pivot_magnitude <= MINIMUM_VARIANCE) {
            return false;
        }

        if (pivot_row != pivot_column) {
            for (uint8_t column = 0;
                 column < static_cast<uint8_t>(2U * dimension);
                 ++column) {
                const float temporary =
                    augmented[pivot_column][column];
                augmented[pivot_column][column] =
                    augmented[pivot_row][column];
                augmented[pivot_row][column] = temporary;
            }
        }

        const float pivot = augmented[pivot_column][pivot_column];
        for (uint8_t column = 0;
             column < static_cast<uint8_t>(2U * dimension);
             ++column) {
            augmented[pivot_column][column] /= pivot;
        }

        for (uint8_t row = 0; row < dimension; ++row) {
            if (row == pivot_column) continue;
            const float factor = augmented[row][pivot_column];
            for (uint8_t column = 0;
                 column < static_cast<uint8_t>(2U * dimension);
                 ++column) {
                augmented[row][column] -=
                    factor * augmented[pivot_column][column];
            }
        }
    }

    memset(inverse, 0, sizeof(float) * 9U);
    for (uint8_t row = 0; row < dimension; ++row) {
        for (uint8_t column = 0; column < dimension; ++column) {
            inverse[row][column] =
                augmented[row][dimension + column];
        }
    }
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
    memset(position_world_mm_, 0, sizeof(position_world_mm_));
    memset(velocity_world_mm_s_, 0, sizeof(velocity_world_mm_s_));
    memset(gyro_bias_rad_s_, 0, sizeof(gyro_bias_rad_s_));
    memset(
        acceleration_bias_mm_s2_,
        0,
        sizeof(acceleration_bias_mm_s2_));
    memset(covariance_, 0, sizeof(covariance_));
    memset(latest_gyro_body_rad_s_, 0, sizeof(latest_gyro_body_rad_s_));
    memset(
        previous_acceleration_body_g_,
        0,
        sizeof(previous_acceleration_body_g_));

    quaternion_world_from_body_[0] = 1.0f;
    quaternion_world_from_body_[1] = 0.0f;
    quaternion_world_from_body_[2] = 0.0f;
    quaternion_world_from_body_[3] = 0.0f;

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
    gyro_interval_start_timestamp_ms_ = 0;

    have_previous_acceleration_ = false;
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

    position_world_mm_[0] = static_cast<float>(x_mm);
    position_world_mm_[1] = static_cast<float>(y_mm);
    quaternionFromYaw(yaw_rad, quaternion_world_from_body_);

    const float position_std = clampFloat(
        static_cast<float>(
            position_std_mm > 0
                ? position_std_mm
                : config_.minimum_gps_position_std_mm),
        static_cast<float>(config_.minimum_gps_position_std_mm),
        static_cast<float>(config_.maximum_gps_position_std_mm));

    covariance_[POSITION_X][POSITION_X] = square(position_std);
    covariance_[POSITION_Y][POSITION_Y] = square(position_std);
    covariance_[VELOCITY_X][VELOCITY_X] = square(1000.0f);
    covariance_[VELOCITY_Y][VELOCITY_Y] = square(1000.0f);
    covariance_[ATTITUDE_X][ATTITUDE_X] = square(0.5f);
    covariance_[ATTITUDE_Y][ATTITUDE_Y] = square(0.5f);
    covariance_[ATTITUDE_Z][ATTITUDE_Z] =
        square(yaw_usable ? 0.5f : 3.0f);
    covariance_[GYRO_BIAS_X][GYRO_BIAS_X] = square(0.20f);
    covariance_[GYRO_BIAS_Y][GYRO_BIAS_Y] = square(0.20f);
    covariance_[GYRO_BIAS_Z][GYRO_BIAS_Z] = square(0.20f);
    covariance_[ACCELERATION_BIAS_X][ACCELERATION_BIAS_X] =
        square(1000.0f);
    covariance_[ACCELERATION_BIAS_Y][ACCELERATION_BIAS_Y] =
        square(1000.0f);
    covariance_[ACCELERATION_BIAS_Z][ACCELERATION_BIAS_Z] =
        square(1000.0f);

    initialized_ = true;
    yaw_reference_usable_ = yaw_usable;
    last_state_timestamp_ms_ = timestamp_ms;
    last_predict_timestamp_ms_ = timestamp_ms;
    last_gps_position_used_timestamp_ms_ = timestamp_ms;
    if (yaw_usable) last_yaw_aiding_timestamp_ms_ = timestamp_ms;

    cycle_status_flags_ =
        STATUS_INITIALIZED | STATUS_POSITION_USABLE;
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

void Filter::normalizeQuaternion(float quaternion[4])
{
    const float norm = sqrtf(
        quaternion[0] * quaternion[0] +
        quaternion[1] * quaternion[1] +
        quaternion[2] * quaternion[2] +
        quaternion[3] * quaternion[3]);
    if (!isfinite(norm) || norm <= MINIMUM_QUATERNION_NORM) {
        quaternion[0] = 1.0f;
        quaternion[1] = 0.0f;
        quaternion[2] = 0.0f;
        quaternion[3] = 0.0f;
        return;
    }
    const float inverse_norm = 1.0f / norm;
    for (uint8_t i = 0; i < 4; ++i) {
        quaternion[i] *= inverse_norm;
    }
}

void Filter::quaternionFromYaw(float yaw_rad, float quaternion[4])
{
    const float half_yaw = 0.5f * normalizePi(yaw_rad);
    quaternion[0] = cosf(half_yaw);
    quaternion[1] = 0.0f;
    quaternion[2] = 0.0f;
    quaternion[3] = sinf(half_yaw);
    normalizeQuaternion(quaternion);
}

void Filter::deltaQuaternion(
    const float rotation_vector_rad[3],
    float quaternion[4])
{
    const float angle = vectorNorm3(rotation_vector_rad);
    if (angle < 1.0e-6f) {
        quaternion[0] = 1.0f;
        quaternion[1] = 0.5f * rotation_vector_rad[0];
        quaternion[2] = 0.5f * rotation_vector_rad[1];
        quaternion[3] = 0.5f * rotation_vector_rad[2];
        normalizeQuaternion(quaternion);
        return;
    }

    const float half_angle = 0.5f * angle;
    const float scale = sinf(half_angle) / angle;
    quaternion[0] = cosf(half_angle);
    quaternion[1] = rotation_vector_rad[0] * scale;
    quaternion[2] = rotation_vector_rad[1] * scale;
    quaternion[3] = rotation_vector_rad[2] * scale;
}

void Filter::multiplyQuaternion(
    const float left[4],
    const float right[4],
    float result[4])
{
    const float temporary[4] = {
        left[0] * right[0] - left[1] * right[1] -
            left[2] * right[2] - left[3] * right[3],
        left[0] * right[1] + left[1] * right[0] +
            left[2] * right[3] - left[3] * right[2],
        left[0] * right[2] - left[1] * right[3] +
            left[2] * right[0] + left[3] * right[1],
        left[0] * right[3] + left[1] * right[2] -
            left[2] * right[1] + left[3] * right[0]
    };
    memcpy(result, temporary, sizeof(temporary));
}

void Filter::rotationMatrix(
    const float quaternion[4],
    float matrix[3][3])
{
    const float w = quaternion[0];
    const float x = quaternion[1];
    const float y = quaternion[2];
    const float z = quaternion[3];

    matrix[0][0] = 1.0f - 2.0f * (y * y + z * z);
    matrix[0][1] = 2.0f * (x * y - w * z);
    matrix[0][2] = 2.0f * (x * z + w * y);
    matrix[1][0] = 2.0f * (x * y + w * z);
    matrix[1][1] = 1.0f - 2.0f * (x * x + z * z);
    matrix[1][2] = 2.0f * (y * z - w * x);
    matrix[2][0] = 2.0f * (x * z - w * y);
    matrix[2][1] = 2.0f * (y * z + w * x);
    matrix[2][2] = 1.0f - 2.0f * (x * x + y * y);
}

void Filter::rotateBodyToWorld(
    const float quaternion[4],
    const float body[3],
    float world[3])
{
    float rotation[3][3] = {};
    rotationMatrix(quaternion, rotation);
    for (uint8_t row = 0; row < 3; ++row) {
        world[row] = 0.0f;
        for (uint8_t column = 0; column < 3; ++column) {
            world[row] += rotation[row][column] * body[column];
        }
    }
}

void Filter::rotateWorldToBody(
    const float quaternion[4],
    const float world[3],
    float body[3])
{
    float rotation[3][3] = {};
    rotationMatrix(quaternion, rotation);
    for (uint8_t body_axis = 0; body_axis < 3; ++body_axis) {
        body[body_axis] = 0.0f;
        for (uint8_t world_axis = 0; world_axis < 3; ++world_axis) {
            body[body_axis] +=
                rotation[world_axis][body_axis] * world[world_axis];
        }
    }
}

float Filter::yawFromQuaternion(const float quaternion[4])
{
    float rotation[3][3] = {};
    rotationMatrix(quaternion, rotation);
    return atan2f(rotation[1][0], rotation[0][0]);
}

bool Filter::predict(const ImuObservation& observation)
{
    if (!initialized_ || observation.timestamp_ms == 0 ||
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
        // A very long gap cannot safely be filled using one stale IMU sample.
        // Rebase time and make the loss of information explicit.
        inflateCovariance(4.0f, 4.0f);
        last_predict_timestamp_ms_ = observation.timestamp_ms;
        last_state_timestamp_ms_ = observation.timestamp_ms;
        gyro_interval_z_integral_rad_ = 0.0f;
        gyro_interval_duration_s_ = 0.0f;
        gyro_interval_start_timestamp_ms_ = observation.timestamp_ms;
        cycle_status_flags_ |= STATUS_DEGRADED | STATUS_IMU_REJECTED;
        return false;
    }

    const float acceleration_body_mm_s2[3] = {
        observation.accel_x_g * config_.gravity_mm_s2,
        observation.accel_y_g * config_.gravity_mm_s2,
        observation.accel_z_g * config_.gravity_mm_s2
    };
    const float gyro_body_rad_s[3] = {
        observation.gyro_x_rad_s,
        observation.gyro_y_rad_s,
        observation.gyro_z_rad_s
    };

    const uint32_t maximum_step_ms =
        config_.maximum_prediction_step_ms > 0
            ? config_.maximum_prediction_step_ms
            : 1U;
    const uint32_t step_count =
        (dt_ms + maximum_step_ms - 1U) / maximum_step_ms;
    const float step_dt_s =
        static_cast<float>(dt_ms) * 0.001f /
        static_cast<float>(step_count);

    for (uint32_t step = 0; step < step_count; ++step) {
        propagateStep(
            acceleration_body_mm_s2,
            gyro_body_rad_s,
            step_dt_s);
    }

    memcpy(
        latest_gyro_body_rad_s_,
        gyro_body_rad_s,
        sizeof(latest_gyro_body_rad_s_));
    gyro_interval_z_integral_rad_ +=
        observation.gyro_z_rad_s *
        static_cast<float>(dt_ms) * 0.001f;
    gyro_interval_duration_s_ +=
        static_cast<float>(dt_ms) * 0.001f;

    last_predict_timestamp_ms_ = observation.timestamp_ms;
    last_state_timestamp_ms_ = observation.timestamp_ms;

    updateGravityDirection(observation, acceleration_body_mm_s2);
    cycle_status_flags_ |= STATUS_IMU_USED;
    return true;
}

void Filter::propagateStep(
    const float acceleration_body_mm_s2[3],
    const float gyro_body_rad_s[3],
    float dt_s)
{
    const float corrected_acceleration_body[3] = {
        acceleration_body_mm_s2[0] - acceleration_bias_mm_s2_[0],
        acceleration_body_mm_s2[1] - acceleration_bias_mm_s2_[1],
        acceleration_body_mm_s2[2] - acceleration_bias_mm_s2_[2]
    };
    const float corrected_gyro_body[3] = {
        gyro_body_rad_s[0] - gyro_bias_rad_s_[0],
        gyro_body_rad_s[1] - gyro_bias_rad_s_[1],
        gyro_body_rad_s[2] - gyro_bias_rad_s_[2]
    };

    // 地上走行では僅かな傾きの重力成分が巨大な位置ドリフトになる。
    // 加速度は姿勢・異常判定だけに使い、位置と速度は積分しない。

    const float full_rotation_vector[3] = {
        corrected_gyro_body[0] * dt_s,
        corrected_gyro_body[1] * dt_s,
        corrected_gyro_body[2] * dt_s
    };
    float full_delta_quaternion[4] = {};
    deltaQuaternion(full_rotation_vector, full_delta_quaternion);
    float propagated_quaternion[4] = {};
    multiplyQuaternion(
        quaternion_world_from_body_,
        full_delta_quaternion,
        propagated_quaternion);
    memcpy(
        quaternion_world_from_body_,
        propagated_quaternion,
        sizeof(quaternion_world_from_body_));
    normalizeQuaternion(quaternion_world_from_body_);

    float transition[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t i = 0; i < STATE_COUNT; ++i) {
        transition[i][i] = 1.0f;
    }

    float gyro_skew[3][3] = {};
    skewMatrix(corrected_gyro_body, gyro_skew);
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t column = 0; column < 3; ++column) {
            transition[ATTITUDE_X + row][ATTITUDE_X + column] -=
                gyro_skew[row][column] * dt_s;
        }
        transition[ATTITUDE_X + row][GYRO_BIAS_X + row] = -dt_s;
    }

    float intermediate[STATE_COUNT][STATE_COUNT] = {};
    float predicted_covariance[STATE_COUNT][STATE_COUNT] = {};
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
                predicted_covariance[row][column] +=
                    intermediate[row][k] * transition[column][k];
            }
        }
    }

    for (uint8_t axis = 0; axis < 3; ++axis) {
        predicted_covariance[ATTITUDE_X + axis][ATTITUDE_X + axis] +=
            square(config_.gyro_noise_rad_s) * dt_s;
        predicted_covariance[GYRO_BIAS_X + axis][GYRO_BIAS_X + axis] +=
            square(config_.gyro_bias_walk_rad_s2) * dt_s;
        predicted_covariance[ACCELERATION_BIAS_X + axis]
                            [ACCELERATION_BIAS_X + axis] +=
            square(config_.acceleration_bias_walk_mm_s3) * dt_s;
    }

    memcpy(covariance_, predicted_covariance, sizeof(covariance_));
    stabilizeCovariance();
}

void Filter::updateGravityDirection(
    const ImuObservation& observation,
    const float acceleration_body_mm_s2[3])
{
    const float acceleration_body_g[3] = {
        observation.accel_x_g,
        observation.accel_y_g,
        observation.accel_z_g
    };
    const float acceleration_norm_g = vectorNorm3(acceleration_body_g);

    float jerk_g_s = 0.0f;
    bool jerk_valid = false;
    if (have_previous_acceleration_) {
        const int32_t dt_ms = signedTimeDifference(
            observation.timestamp_ms,
            previous_acceleration_timestamp_ms_);
        if (dt_ms > 0 && dt_ms <= 500) {
            const float difference[3] = {
                acceleration_body_g[0] -
                    previous_acceleration_body_g_[0],
                acceleration_body_g[1] -
                    previous_acceleration_body_g_[1],
                acceleration_body_g[2] -
                    previous_acceleration_body_g_[2]
            };
            jerk_g_s =
                vectorNorm3(difference) /
                (static_cast<float>(dt_ms) * 0.001f);
            jerk_valid = true;
        }
    }

    memcpy(
        previous_acceleration_body_g_,
        acceleration_body_g,
        sizeof(previous_acceleration_body_g_));
    previous_acceleration_timestamp_ms_ = observation.timestamp_ms;
    have_previous_acceleration_ = true;

    if (!isfinite(acceleration_norm_g) ||
        fabsf(acceleration_norm_g - 1.0f) >
            config_.gravity_norm_tolerance_g ||
        (jerk_valid && jerk_g_s > config_.gravity_jerk_limit_g_s)) {
        return;
    }

    const float corrected_acceleration[3] = {
        acceleration_body_mm_s2[0] - acceleration_bias_mm_s2_[0],
        acceleration_body_mm_s2[1] - acceleration_bias_mm_s2_[1],
        acceleration_body_mm_s2[2] - acceleration_bias_mm_s2_[2]
    };
    const float corrected_norm = vectorNorm3(corrected_acceleration);
    if (!isfinite(corrected_norm) || corrected_norm < 100.0f) return;

    const float measured_direction[3] = {
        corrected_acceleration[0] / corrected_norm,
        corrected_acceleration[1] / corrected_norm,
        corrected_acceleration[2] / corrected_norm
    };
    const float world_up[3] = {0.0f, 0.0f, 1.0f};
    float predicted_direction[3] = {};
    rotateWorldToBody(
        quaternion_world_from_body_,
        world_up,
        predicted_direction);

    float residual[MAX_MEASUREMENT_DIMENSION] = {};
    float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT] = {};
    float variance[MAX_MEASUREMENT_DIMENSION] = {};
    float direction_skew[3][3] = {};
    skewMatrix(predicted_direction, direction_skew);
    for (uint8_t row = 0; row < 3; ++row) {
        residual[row] =
            measured_direction[row] - predicted_direction[row];
        variance[row] = square(config_.gravity_direction_std);
        for (uint8_t column = 0; column < 3; ++column) {
            jacobian[row][ATTITUDE_X + column] =
                direction_skew[row][column];
        }
    }

    measurementUpdate(
        residual,
        jacobian,
        variance,
        3,
        config_.nis_soft_3d,
        config_.nis_hard_3d);
}

Filter::UpdateResult Filter::measurementUpdate(
    const float residual[MAX_MEASUREMENT_DIMENSION],
    const float observation_jacobian
        [MAX_MEASUREMENT_DIMENSION][STATE_COUNT],
    const float measurement_variance[MAX_MEASUREMENT_DIMENSION],
    uint8_t measurement_dimension,
    float soft_gate,
    float hard_gate)
{
    if (measurement_dimension == 0 ||
        measurement_dimension > MAX_MEASUREMENT_DIMENSION ||
        soft_gate <= 0.0f ||
        hard_gate <= soft_gate) {
        return UpdateResult::Rejected;
    }

    float effective_variance[MAX_MEASUREMENT_DIMENSION] = {};
    for (uint8_t i = 0; i < measurement_dimension; ++i) {
        if (!isfinite(residual[i]) ||
            !isfinite(measurement_variance[i]) ||
            measurement_variance[i] <= 0.0f) {
            return UpdateResult::Rejected;
        }
        effective_variance[i] = measurement_variance[i];
    }

    float projected[STATE_COUNT][MAX_MEASUREMENT_DIMENSION] = {};
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        for (uint8_t measurement = 0;
             measurement < measurement_dimension;
             ++measurement) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                projected[state][measurement] +=
                    covariance_[state][k] *
                    observation_jacobian[measurement][k];
            }
        }
    }

    auto calculateInnovation = [&](
        const float variances[MAX_MEASUREMENT_DIMENSION],
        float innovation[3][3],
        float innovation_inverse[3][3],
        float& nis) -> bool {
        memset(innovation, 0, sizeof(float) * 9U);
        for (uint8_t row = 0; row < measurement_dimension; ++row) {
            for (uint8_t column = 0;
                 column < measurement_dimension;
                 ++column) {
                for (uint8_t state = 0; state < STATE_COUNT; ++state) {
                    innovation[row][column] +=
                        observation_jacobian[row][state] *
                        projected[state][column];
                }
            }
            innovation[row][row] += variances[row];
        }

        if (!invertSmallMatrix(
                innovation,
                measurement_dimension,
                innovation_inverse)) {
            return false;
        }

        nis = 0.0f;
        for (uint8_t row = 0; row < measurement_dimension; ++row) {
            for (uint8_t column = 0;
                 column < measurement_dimension;
                 ++column) {
                nis += residual[row] *
                    innovation_inverse[row][column] *
                    residual[column];
            }
        }
        return isfinite(nis);
    };

    float innovation[3][3] = {};
    float innovation_inverse[3][3] = {};
    float nis = 0.0f;
    if (!calculateInnovation(
            effective_variance,
            innovation,
            innovation_inverse,
            nis) ||
        nis > hard_gate) {
        return UpdateResult::Rejected;
    }

    UpdateResult result = UpdateResult::Accepted;
    if (nis > soft_gate) {
        const float variance_scale = clampFloat(
            nis / soft_gate,
            1.0f,
            hard_gate / soft_gate);
        for (uint8_t i = 0; i < measurement_dimension; ++i) {
            effective_variance[i] *= variance_scale;
        }
        if (!calculateInnovation(
                effective_variance,
                innovation,
                innovation_inverse,
                nis)) {
            return UpdateResult::Rejected;
        }
        result = UpdateResult::SoftAccepted;
    }

    float gain[STATE_COUNT][MAX_MEASUREMENT_DIMENSION] = {};
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        for (uint8_t measurement = 0;
             measurement < measurement_dimension;
             ++measurement) {
            for (uint8_t k = 0; k < measurement_dimension; ++k) {
                gain[state][measurement] +=
                    projected[state][k] *
                    innovation_inverse[k][measurement];
            }
        }
    }

    float error_state[STATE_COUNT] = {};
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        for (uint8_t measurement = 0;
             measurement < measurement_dimension;
             ++measurement) {
            error_state[state] +=
                gain[state][measurement] * residual[measurement];
        }
    }

    // Joseph form preserves symmetry and positive semi-definiteness better
    // than P = P - KHP on a single-precision embedded target.
    float identity_minus_kh[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        identity_minus_kh[row][row] = 1.0f;
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t measurement = 0;
                 measurement < measurement_dimension;
                 ++measurement) {
                identity_minus_kh[row][column] -=
                    gain[row][measurement] *
                    observation_jacobian[measurement][column];
            }
        }
    }

    float intermediate[STATE_COUNT][STATE_COUNT] = {};
    float updated_covariance[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                intermediate[row][column] +=
                    identity_minus_kh[row][k] *
                    covariance_[k][column];
            }
        }
    }
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                updated_covariance[row][column] +=
                    intermediate[row][k] *
                    identity_minus_kh[column][k];
            }
            for (uint8_t measurement = 0;
                 measurement < measurement_dimension;
                 ++measurement) {
                updated_covariance[row][column] +=
                    gain[row][measurement] *
                    effective_variance[measurement] *
                    gain[column][measurement];
            }
        }
    }

    memcpy(covariance_, updated_covariance, sizeof(covariance_));
    injectError(error_state);
    const float attitude_error[3] = {
        error_state[ATTITUDE_X],
        error_state[ATTITUDE_Y],
        error_state[ATTITUDE_Z]
    };
    applyAttitudeResetJacobian(attitude_error);
    stabilizeCovariance();
    return result;
}

void Filter::injectError(const float error_state[STATE_COUNT])
{
    position_world_mm_[0] += error_state[POSITION_X];
    position_world_mm_[1] += error_state[POSITION_Y];
    velocity_world_mm_s_[0] += error_state[VELOCITY_X];
    velocity_world_mm_s_[1] += error_state[VELOCITY_Y];

    const float attitude_error[3] = {
        error_state[ATTITUDE_X],
        error_state[ATTITUDE_Y],
        error_state[ATTITUDE_Z]
    };
    float correction_quaternion[4] = {};
    deltaQuaternion(attitude_error, correction_quaternion);
    float corrected_quaternion[4] = {};
    multiplyQuaternion(
        quaternion_world_from_body_,
        correction_quaternion,
        corrected_quaternion);
    memcpy(
        quaternion_world_from_body_,
        corrected_quaternion,
        sizeof(quaternion_world_from_body_));
    normalizeQuaternion(quaternion_world_from_body_);

    for (uint8_t axis = 0; axis < 3; ++axis) {
        gyro_bias_rad_s_[axis] +=
            error_state[GYRO_BIAS_X + axis];
        acceleration_bias_mm_s2_[axis] +=
            error_state[ACCELERATION_BIAS_X + axis];
    }
}

void Filter::applyAttitudeResetJacobian(
    const float attitude_error[3])
{
    float reset_jacobian[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t i = 0; i < STATE_COUNT; ++i) {
        reset_jacobian[i][i] = 1.0f;
    }

    float error_skew[3][3] = {};
    skewMatrix(attitude_error, error_skew);
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t column = 0; column < 3; ++column) {
            reset_jacobian[ATTITUDE_X + row][ATTITUDE_X + column] -=
                0.5f * error_skew[row][column];
        }
    }

    float intermediate[STATE_COUNT][STATE_COUNT] = {};
    float reset_covariance[STATE_COUNT][STATE_COUNT] = {};
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                intermediate[row][column] +=
                    reset_jacobian[row][k] * covariance_[k][column];
            }
        }
    }
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                reset_covariance[row][column] +=
                    intermediate[row][k] * reset_jacobian[column][k];
            }
        }
    }
    memcpy(covariance_, reset_covariance, sizeof(covariance_));
}

bool Filter::updateEncoder(const EncoderObservation& observation)
{
    if (!initialized_ || observation.timestamp_ms == 0 ||
        config_.track_width_mm <= 0.0f) {
        cycle_status_flags_ |= STATUS_ENCODER_REJECTED;
        return false;
    }

    if (!have_previous_encoder_) {
        previous_encoder_ = observation;
        have_previous_encoder_ = true;
        gyro_interval_z_integral_rad_ = 0.0f;
        gyro_interval_duration_s_ = 0.0f;
        gyro_interval_start_timestamp_ms_ = observation.timestamp_ms;
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
    const int64_t delta_left =
        static_cast<int64_t>(observation.left_mm) -
        static_cast<int64_t>(previous_encoder_.left_mm);
    const int64_t delta_right =
        static_cast<int64_t>(observation.right_mm) -
        static_cast<int64_t>(previous_encoder_.right_mm);

    previous_encoder_ = observation;

    auto finishInterval = [&]() {
        gyro_interval_z_integral_rad_ = 0.0f;
        gyro_interval_duration_s_ = 0.0f;
        gyro_interval_start_timestamp_ms_ = observation.timestamp_ms;
    };

    if (dt_ms > config_.maximum_encoder_interval_ms) {
        finishInterval();
        cycle_status_flags_ |=
            STATUS_DEGRADED | STATUS_ENCODER_REJECTED;
        noteRejected(encoder_health_, config_);
        return false;
    }

    const float dt_s = static_cast<float>(dt_ms) * 0.001f;
    const float left_velocity =
        static_cast<float>(delta_left) / dt_s;
    const float right_velocity =
        static_cast<float>(delta_right) / dt_s;
    if (!isfinite(left_velocity) || !isfinite(right_velocity) ||
        fabsf(left_velocity) > config_.maximum_encoder_speed_mm_s ||
        fabsf(right_velocity) > config_.maximum_encoder_speed_mm_s) {
        finishInterval();
        cycle_status_flags_ |=
            STATUS_DEGRADED | STATUS_ENCODER_REJECTED;
        noteRejected(encoder_health_, config_);
        return false;
    }

    const float measured_forward_velocity =
        0.5f * (left_velocity + right_velocity);
    const float measured_yaw_rate =
        (right_velocity - left_velocity) / config_.track_width_mm;

    bool yaw_interval_aligned = false;
    if (gyro_interval_duration_s_ > 0.0f) {
        const uint32_t gyro_duration_ms = static_cast<uint32_t>(
            lroundf(gyro_interval_duration_s_ * 1000.0f));
        const uint32_t duration_difference =
            gyro_duration_ms > dt_ms
                ? gyro_duration_ms - dt_ms
                : dt_ms - gyro_duration_ms;
        yaw_interval_aligned =
            duration_difference <=
            config_.encoder_imu_alignment_tolerance_ms;
    }

    // 位置は左右エンコーダの実移動量から直接更新する。
    // ジャイロで得た現在方位と車輪差の中間方位を用いる。
    const float delta_forward_mm =
        0.5f * (static_cast<float>(delta_left) +
                static_cast<float>(delta_right));
    const float delta_encoder_yaw =
        (static_cast<float>(delta_right) -
         static_cast<float>(delta_left)) / config_.track_width_mm;
    const float gyro_delta_yaw =
        gyro_interval_z_integral_rad_ -
        gyro_bias_rad_s_[2] * gyro_interval_duration_s_;
    const float encoder_yaw_correction =
        yaw_interval_aligned
            ? 0.20f * normalizePi(
                delta_encoder_yaw - gyro_delta_yaw)
            : delta_encoder_yaw;
    const float yaw_correction_vector[3] = {
        0.0f, 0.0f, encoder_yaw_correction
    };
    float yaw_correction_quaternion[4] = {};
    deltaQuaternion(
        yaw_correction_vector,
        yaw_correction_quaternion);
    float corrected_quaternion[4] = {};
    multiplyQuaternion(
        quaternion_world_from_body_,
        yaw_correction_quaternion,
        corrected_quaternion);
    memcpy(
        quaternion_world_from_body_,
        corrected_quaternion,
        sizeof(quaternion_world_from_body_));
    normalizeQuaternion(quaternion_world_from_body_);

    const float current_yaw =
        yawFromQuaternion(quaternion_world_from_body_);
    const float midpoint_yaw =
        current_yaw - 0.5f * delta_encoder_yaw;
    position_world_mm_[0] += delta_forward_mm * cosf(midpoint_yaw);
    position_world_mm_[1] += delta_forward_mm * sinf(midpoint_yaw);
    velocity_world_mm_s_[0] =
        measured_forward_velocity * cosf(current_yaw);
    velocity_world_mm_s_[1] =
        measured_forward_velocity * sinf(current_yaw);

    // GPSがない間は、走行距離に比例して位置・方位の不確かさを増やす。
    const float encoder_position_noise_mm =
        20.0f + 0.05f * fabsf(delta_forward_mm);
    covariance_[POSITION_X][POSITION_X] +=
        square(encoder_position_noise_mm);
    covariance_[POSITION_Y][POSITION_Y] +=
        square(encoder_position_noise_mm);
    covariance_[ATTITUDE_Z][ATTITUDE_Z] +=
        square(0.01f + 0.10f * fabsf(delta_encoder_yaw));

    float rotation[3][3] = {};
    rotationMatrix(quaternion_world_from_body_, rotation);
    const float world_velocity[3] = {
        velocity_world_mm_s_[0],
        velocity_world_mm_s_[1],
        0.0f
    };
    float body_velocity[3] = {};
    rotateWorldToBody(
        quaternion_world_from_body_,
        world_velocity,
        body_velocity);

    float residual[MAX_MEASUREMENT_DIMENSION] = {};
    float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT] = {};
    float variance[MAX_MEASUREMENT_DIMENSION] = {};

    residual[0] = measured_forward_velocity - body_velocity[0];
    jacobian[0][VELOCITY_X] = rotation[0][0];
    jacobian[0][VELOCITY_Y] = rotation[1][0];
    jacobian[0][ATTITUDE_Y] = -body_velocity[2];
    jacobian[0][ATTITUDE_Z] = body_velocity[1];
    variance[0] = square(config_.encoder_velocity_std_mm_s);
    const UpdateResult forward_result = measurementUpdate(
        residual,
        jacobian,
        variance,
        1,
        config_.nis_soft_1d,
        config_.nis_hard_1d);

    UpdateResult yaw_result = UpdateResult::Rejected;
    if (yaw_interval_aligned) {
        memset(residual, 0, sizeof(residual));
        memset(jacobian, 0, sizeof(jacobian));
        memset(variance, 0, sizeof(variance));
        const float average_raw_gyro_z =
            gyro_interval_z_integral_rad_ /
            gyro_interval_duration_s_;
        const float predicted_yaw_rate =
            average_raw_gyro_z - gyro_bias_rad_s_[2];
        residual[0] = measured_yaw_rate - predicted_yaw_rate;
        jacobian[0][GYRO_BIAS_Z] = -1.0f;
        variance[0] = square(config_.encoder_yaw_rate_std_rad_s);
        yaw_result = measurementUpdate(
            residual,
            jacobian,
            variance,
            1,
            config_.nis_soft_1d,
            config_.nis_hard_1d);
    }

    finishInterval();

    // Non-holonomic lateral-velocity constraint.  Do not apply it when the
    // encoder/gyro turn-rate comparison indicates likely slip.
    UpdateResult lateral_result = UpdateResult::Rejected;
    if (yaw_result != UpdateResult::Rejected) {
        const float updated_world_velocity[3] = {
            velocity_world_mm_s_[0],
            velocity_world_mm_s_[1],
            0.0f
        };
        rotateWorldToBody(
            quaternion_world_from_body_,
            updated_world_velocity,
            body_velocity);
        rotationMatrix(quaternion_world_from_body_, rotation);

        memset(residual, 0, sizeof(residual));
        memset(jacobian, 0, sizeof(jacobian));
        memset(variance, 0, sizeof(variance));
        residual[0] = -body_velocity[1];
        jacobian[0][VELOCITY_X] = rotation[0][1];
        jacobian[0][VELOCITY_Y] = rotation[1][1];
        jacobian[0][ATTITUDE_X] = body_velocity[2];
        jacobian[0][ATTITUDE_Z] = -body_velocity[0];
        variance[0] =
            square(config_.encoder_lateral_velocity_std_mm_s);
        lateral_result = measurementUpdate(
            residual,
            jacobian,
            variance,
            1,
            config_.nis_soft_1d,
            config_.nis_hard_1d);
    }

    const bool forward_used =
        forward_result != UpdateResult::Rejected;
    const bool yaw_used =
        yaw_result != UpdateResult::Rejected;
    const bool lateral_used =
        lateral_result != UpdateResult::Rejected;
    const bool used = forward_used || yaw_used || lateral_used;

    if (used) {
        noteAccepted(encoder_health_, config_);
        last_encoder_used_timestamp_ms_ = observation.timestamp_ms;
        cycle_status_flags_ |= STATUS_ENCODER_USED;
        if (!yaw_used && yaw_interval_aligned) {
            cycle_status_flags_ |= STATUS_ENCODER_REJECTED;
        }
        return true;
    }

    noteRejected(encoder_health_, config_);
    cycle_status_flags_ |= STATUS_ENCODER_REJECTED;
    return false;
}

bool Filter::updateGps(const GpsUpdate& observation)
{
    if (!initialized_ || observation.timestamp_ms == 0) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        return false;
    }

    if (observation.timestamp_ms == last_gps_timestamp_ms_) {
        return false;
    }
    if (last_gps_timestamp_ms_ != 0 &&
        signedTimeDifference(
            observation.timestamp_ms,
            last_gps_timestamp_ms_) <= 0) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        noteRejected(gps_health_, config_);
        return false;
    }
    last_gps_timestamp_ms_ = observation.timestamp_ms;

    if (!observation.fix_ok || observation.fix_type < 2U) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        noteRejected(gps_health_, config_);
        return false;
    }

    // 大きなhAccを上限値へ丸めて使うと、数十mのGPS飛びを過信する。
    // 最大許容精度を超える位置観測はFusionへ入れない。
    if (observation.horizontal_accuracy_mm == 0U ||
        observation.horizontal_accuracy_mm >
            config_.maximum_gps_position_std_mm) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        noteRejected(gps_health_, config_);
        return false;
    }

    const int32_t measurement_age_ms = signedTimeDifference(
        last_state_timestamp_ms_,
        observation.timestamp_ms);
    if (measurement_age_ms >
            static_cast<int32_t>(config_.maximum_delayed_gps_ms) ||
        measurement_age_ms <
            -static_cast<int32_t>(
                config_.maximum_future_measurement_ms)) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        noteRejected(gps_health_, config_);
        return false;
    }

    // Bring a delayed position measurement to the current filter epoch using
    // the receiver velocity.  This avoids applying an old position directly
    // to the current state when full rollback/replay is unavailable.
    const float projection_dt_s =
        measurement_age_ms > 0
            ? static_cast<float>(measurement_age_ms) * 0.001f
            : 0.0f;
    const float gps_x =
        static_cast<float>(observation.x_mm) +
        static_cast<float>(observation.velocity_east_mm_s) *
            projection_dt_s;
    const float gps_y =
        static_cast<float>(observation.y_mm) +
        static_cast<float>(observation.velocity_north_mm_s) *
            projection_dt_s;

    // フィールド外でもGPS観測自体は有効である。地図境界はNavigationが
    // 扱い、Localizationでは非有限値だけを棄却する。
    if (!isfinite(gps_x) || !isfinite(gps_y)) {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        noteRejected(gps_health_, config_);
        return false;
    }

    const float position_std = clampFloat(
        static_cast<float>(observation.horizontal_accuracy_mm),
        static_cast<float>(config_.minimum_gps_position_std_mm),
        static_cast<float>(config_.maximum_gps_position_std_mm));

    const float position_residual_mm = hypotf(
        gps_x - position_world_mm_[0],
        gps_y - position_world_mm_[1]);
    if (position_residual_mm >
        static_cast<float>(config_.gps_resnap_distance_mm)) {
        if (observation.horizontal_accuracy_mm >
            config_.gps_resnap_maximum_accuracy_mm) {
            gps_resnap_sample_count_ = 0;
            cycle_status_flags_ |= STATUS_GPS_REJECTED;
            noteRejected(gps_health_, config_);
            return false;
        }
        const float recovery_spread_mm = hypotf(
            gps_x - gps_resnap_anchor_x_mm_,
            gps_y - gps_resnap_anchor_y_mm_);
        if (gps_resnap_sample_count_ == 0U ||
            recovery_spread_mm >
                static_cast<float>(
                    config_.gps_resnap_stable_radius_mm)) {
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

        if (gps_resnap_sample_count_ <
            config_.gps_resnap_required_samples) {
            cycle_status_flags_ |= STATUS_GPS_REJECTED;
            return false;
        }

        // 大誤差が3回安定して続いた場合だけ、位置・速度・共分散を
        // 一体で再初期化する。一発のGPS飛びではスナップしない。
        position_world_mm_[0] =
            gps_resnap_sum_x_mm_ / gps_resnap_sample_count_;
        position_world_mm_[1] =
            gps_resnap_sum_y_mm_ / gps_resnap_sample_count_;
        velocity_world_mm_s_[0] =
            static_cast<float>(observation.velocity_east_mm_s);
        velocity_world_mm_s_[1] =
            static_cast<float>(observation.velocity_north_mm_s);
        covariance_[POSITION_X][POSITION_X] =
            square(position_std);
        covariance_[POSITION_Y][POSITION_Y] =
            square(position_std);
        covariance_[VELOCITY_X][VELOCITY_X] =
            square(fmaxf(
                static_cast<float>(
                    observation.speed_accuracy_mm_s),
                100.0f));
        covariance_[VELOCITY_Y][VELOCITY_Y] =
            covariance_[VELOCITY_X][VELOCITY_X];
        gps_resnap_sample_count_ = 0;
        noteAccepted(gps_health_, config_);
        last_gps_position_used_timestamp_ms_ =
            last_state_timestamp_ms_;
        cycle_status_flags_ |= STATUS_GPS_USED;
        return true;
    }
    gps_resnap_sample_count_ = 0;

    float residual[MAX_MEASUREMENT_DIMENSION] = {};
    float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT] = {};
    float variance[MAX_MEASUREMENT_DIMENSION] = {};
    residual[0] = gps_x - position_world_mm_[0];
    residual[1] = gps_y - position_world_mm_[1];
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

    UpdateResult velocity_result = UpdateResult::Rejected;
    UpdateResult course_result = UpdateResult::Rejected;
    bool course_attempted = false;
    const float measured_velocity_east =
        static_cast<float>(observation.velocity_east_mm_s);
    const float measured_velocity_north =
        static_cast<float>(observation.velocity_north_mm_s);
    const float measured_speed = hypotf(
        measured_velocity_east,
        measured_velocity_north);
    const float speed_std = clampFloat(
        static_cast<float>(observation.speed_accuracy_mm_s),
        static_cast<float>(config_.minimum_gps_speed_std_mm_s),
        static_cast<float>(config_.maximum_gps_speed_std_mm_s));

    if (measured_speed >=
            static_cast<float>(config_.minimum_gps_velocity_mm_s) &&
        observation.speed_accuracy_mm_s <=
            config_.maximum_gps_speed_std_mm_s) {
        memset(residual, 0, sizeof(residual));
        memset(jacobian, 0, sizeof(jacobian));
        memset(variance, 0, sizeof(variance));
        residual[0] =
            measured_velocity_east - velocity_world_mm_s_[0];
        residual[1] =
            measured_velocity_north - velocity_world_mm_s_[1];
        jacobian[0][VELOCITY_X] = 1.0f;
        jacobian[1][VELOCITY_Y] = 1.0f;
        variance[0] = square(speed_std);
        variance[1] = square(speed_std);
        velocity_result = measurementUpdate(
            residual,
            jacobian,
            variance,
            2,
            config_.nis_soft_2d,
            config_.nis_hard_2d);

        // GNSS course is an absolute true-north yaw observation while moving.
        // Its small-angle uncertainty is approximately speed_std / speed.
        // Do not use it at low speed, where course becomes ill-conditioned.
        const float course_std = clampFloat(
            speed_std / measured_speed,
            config_.minimum_gps_course_std_rad,
            config_.maximum_gps_course_std_rad);
        if (isfinite(course_std) &&
            config_.maximum_gps_course_std_rad > 0.0f) {
            course_attempted = true;
            const float measured_course =
                atan2f(
                    measured_velocity_north,
                    measured_velocity_east);
            const float world_up[3] = {0.0f, 0.0f, 1.0f};
            float world_up_body[3] = {};
            rotateWorldToBody(
                quaternion_world_from_body_,
                world_up,
                world_up_body);

            memset(residual, 0, sizeof(residual));
            memset(jacobian, 0, sizeof(jacobian));
            memset(variance, 0, sizeof(variance));
            residual[0] = normalizePi(
                measured_course -
                yawFromQuaternion(quaternion_world_from_body_));
            jacobian[0][ATTITUDE_X] = world_up_body[0];
            jacobian[0][ATTITUDE_Y] = world_up_body[1];
            jacobian[0][ATTITUDE_Z] = world_up_body[2];
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

    const bool position_used =
        position_result != UpdateResult::Rejected;
    const bool velocity_used =
        velocity_result != UpdateResult::Rejected;
    const bool course_used =
        course_result != UpdateResult::Rejected;

    if (position_used || velocity_used || course_used) {
        noteAccepted(gps_health_, config_);
        if (position_used) {
            last_gps_position_used_timestamp_ms_ =
                last_state_timestamp_ms_;
        }
        if (course_used) {
            yaw_reference_usable_ = true;
            last_yaw_aiding_timestamp_ms_ =
                last_state_timestamp_ms_;
        }
        if (!position_used ||
            (course_attempted && !course_used)) {
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
    if (!initialized_ || observation.timestamp_ms == 0 ||
        !isfinite(observation.x_uT) ||
        !isfinite(observation.y_uT) ||
        !isfinite(observation.z_uT)) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        return false;
    }

    if (observation.timestamp_ms == last_magnetic_timestamp_ms_) {
        return false;
    }
    if (last_magnetic_timestamp_ms_ != 0 &&
        signedTimeDifference(
            observation.timestamp_ms,
            last_magnetic_timestamp_ms_) <= 0) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        noteRejected(magnetic_health_, config_);
        return false;
    }
    last_magnetic_timestamp_ms_ = observation.timestamp_ms;

    const float magnetic_body[3] = {
        observation.x_uT,
        observation.y_uT,
        observation.z_uT
    };
    const float total_strength = vectorNorm3(magnetic_body);
    if (!isfinite(total_strength) ||
        total_strength < config_.magnetic_min_total_uT ||
        total_strength > config_.magnetic_max_total_uT) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        noteRejected(magnetic_health_, config_);
        return false;
    }

    // 走行中はモーター磁界の影響を受けやすい。ジャイロとGPS進行方位を
    // 優先し、地磁気は停止・低速時のドリフト補正に限定する。
    if (fabsf(forwardVelocity()) >
            config_.magnetic_moving_max_speed_mm_s ||
        fabsf(yawRate()) >
            config_.magnetic_moving_max_yaw_rate_rad_s) {
        return false;
    }

    const bool physically_stationary =
        fabsf(forwardVelocity()) <=
            config_.magnetic_stationary_max_speed_mm_s &&
        fabsf(yawRate()) <=
            config_.magnetic_stationary_max_yaw_rate_rad_s;
    // 指令中なのに動いていない場合は、スタック中のモーター磁界を
    // 「停止中の高信頼な地磁気」と誤認しない。
    if (physically_stationary && observation.motor_command_active) {
        return false;
    }
    const float magnetic_yaw_std_rad = physically_stationary
        ? config_.magnetic_stationary_yaw_std_rad
        : config_.magnetic_moving_yaw_std_rad;

    if (have_magnetic_reference_strength_) {
        const float relative_error = fabsf(
            total_strength - magnetic_reference_strength_uT_) /
            magnetic_reference_strength_uT_;
        if (relative_error >
            config_.magnetic_strength_relative_tolerance) {
            cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
            noteRejected(magnetic_health_, config_);
            return false;
        }
    }

    float magnetic_world[3] = {};
    rotateBodyToWorld(
        quaternion_world_from_body_,
        magnetic_body,
        magnetic_world);
    const float horizontal_strength =
        hypotf(magnetic_world[0], magnetic_world[1]);
    if (!isfinite(horizontal_strength) ||
        horizontal_strength < config_.magnetic_min_horizontal_uT) {
        cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
        noteRejected(magnetic_health_, config_);
        return false;
    }

    // East-positive declination means magnetic north is clockwise from true
    // north in the east/north plane.
    const float expected_field_azimuth =
        0.5f * PI_F - config_.magnetic_declination_rad;
    const float measured_field_azimuth =
        atan2f(magnetic_world[1], magnetic_world[0]);

    const float world_up[3] = {0.0f, 0.0f, 1.0f};
    float world_up_body[3] = {};
    rotateWorldToBody(
        quaternion_world_from_body_,
        world_up,
        world_up_body);

    float residual[MAX_MEASUREMENT_DIMENSION] = {};
    float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT] = {};
    float variance[MAX_MEASUREMENT_DIMENSION] = {};
    residual[0] = normalizePi(
        expected_field_azimuth - measured_field_azimuth);
    jacobian[0][ATTITUDE_X] = world_up_body[0];
    jacobian[0][ATTITUDE_Y] = world_up_body[1];
    jacobian[0][ATTITUDE_Z] = world_up_body[2];
    variance[0] = square(magnetic_yaw_std_rad);

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

void Filter::noteAccepted(
    HealthTracker& tracker,
    const Config& config)
{
    tracker.rejection_count = 0;
    if (!tracker.failed) {
        tracker.acceptance_count = 0;
        return;
    }

    tracker.acceptance_count =
        incrementSaturated(tracker.acceptance_count);
    if (tracker.acceptance_count >= config.health_recovery_accepts) {
        tracker.failed = false;
        tracker.acceptance_count = 0;
    }
}

void Filter::noteRejected(
    HealthTracker& tracker,
    const Config& config)
{
    tracker.acceptance_count = 0;
    tracker.rejection_count =
        incrementSaturated(tracker.rejection_count);
    if (tracker.rejection_count >= config.health_failure_rejections) {
        tracker.failed = true;
    }
}

void Filter::stabilizeCovariance()
{
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = static_cast<uint8_t>(row + 1U);
             column < STATE_COUNT;
             ++column) {
            float symmetric = 0.5f * (
                covariance_[row][column] +
                covariance_[column][row]);
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

void Filter::inflateCovariance(
    float position_factor,
    float attitude_factor)
{
    position_factor = fmaxf(position_factor, 1.0f);
    attitude_factor = fmaxf(attitude_factor, 1.0f);
    covariance_[POSITION_X][POSITION_X] *= position_factor;
    covariance_[POSITION_Y][POSITION_Y] *= position_factor;
    covariance_[VELOCITY_X][VELOCITY_X] *= position_factor;
    covariance_[VELOCITY_Y][VELOCITY_Y] *= position_factor;
    covariance_[ATTITUDE_X][ATTITUDE_X] *= attitude_factor;
    covariance_[ATTITUDE_Y][ATTITUDE_Y] *= attitude_factor;
    covariance_[ATTITUDE_Z][ATTITUDE_Z] *= attitude_factor;
    stabilizeCovariance();
}

float Filter::forwardVelocity() const
{
    const float world_velocity[3] = {
        velocity_world_mm_s_[0],
        velocity_world_mm_s_[1],
        0.0f
    };
    float body_velocity[3] = {};
    rotateWorldToBody(
        quaternion_world_from_body_,
        world_velocity,
        body_velocity);
    return body_velocity[0];
}

float Filter::yawRate() const
{
    const float corrected_gyro_body[3] = {
        latest_gyro_body_rad_s_[0] - gyro_bias_rad_s_[0],
        latest_gyro_body_rad_s_[1] - gyro_bias_rad_s_[1],
        latest_gyro_body_rad_s_[2] - gyro_bias_rad_s_[2]
    };
    float angular_velocity_world[3] = {};
    rotateBodyToWorld(
        quaternion_world_from_body_,
        corrected_gyro_body,
        angular_velocity_world);
    return angular_velocity_world[2];
}

float Filter::yawVariance() const
{
    const float world_up[3] = {0.0f, 0.0f, 1.0f};
    float world_up_body[3] = {};
    rotateWorldToBody(
        quaternion_world_from_body_,
        world_up,
        world_up_body);

    float variance = 0.0f;
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t column = 0; column < 3; ++column) {
            variance +=
                world_up_body[row] *
                covariance_[ATTITUDE_X + row][ATTITUDE_X + column] *
                world_up_body[column];
        }
    }
    return fmaxf(variance, MINIMUM_VARIANCE);
}

bool Filter::positionUsableAt(uint32_t timestamp_ms) const
{
    if (!initialized_ ||
        last_gps_position_used_timestamp_ms_ == 0) {
        return false;
    }
    (void)timestamp_ms;
    const float position_std = sqrtf(fmaxf(
        covariance_[POSITION_X][POSITION_X],
        covariance_[POSITION_Y][POSITION_Y]));
    return isfinite(position_std) &&
        position_std <=
            static_cast<float>(config_.maximum_usable_position_std_mm);
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
    if (last_yaw_aiding_timestamp_ms_ == 0 ||
        aiding_age < 0 ||
        static_cast<uint32_t>(aiding_age) >
            config_.maximum_yaw_aiding_age_ms) {
        return false;
    }

    const float yaw_std = sqrtf(yawVariance());
    return isfinite(yaw_std) &&
        yaw_std <= config_.maximum_usable_yaw_std_rad;
}

SensorHealth Filter::sensorHealthAt(
    uint32_t now_ms,
    uint32_t last_timestamp_ms,
    uint32_t stale_ms,
    uint32_t failed_ms)
{
    if (last_timestamp_ms == 0U) return SensorHealth::Failed;
    const uint32_t age_ms =
        static_cast<uint32_t>(now_ms - last_timestamp_ms);
    if (age_ms <= stale_ms) return SensorHealth::Fresh;
    if (age_ms <= failed_ms) return SensorHealth::Stale;
    return SensorHealth::Failed;
}

Output Filter::output(uint32_t timestamp_ms) const
{
    Output result{};
    result.timestamp_ms = last_state_timestamp_ms_;
    result.x_mm = static_cast<int32_t>(
        lroundf(position_world_mm_[0]));
    result.y_mm = static_cast<int32_t>(
        lroundf(position_world_mm_[1]));
    result.yaw_rad = normalizeTwoPi(
        yawFromQuaternion(quaternion_world_from_body_));
    result.forward_velocity_mm_s = forwardVelocity();
    result.yaw_rate_rad_s = yawRate();
    result.position_std_mm = static_cast<uint32_t>(lroundf(sqrtf(fmaxf(
        covariance_[POSITION_X][POSITION_X],
        covariance_[POSITION_Y][POSITION_Y]))));
    result.yaw_std_rad = sqrtf(yawVariance());

    result.status_flags = cycle_status_flags_;
    if (initialized_) result.status_flags |= STATUS_INITIALIZED;
    if (positionUsableAt(timestamp_ms)) {
        result.status_flags |= STATUS_POSITION_USABLE;
    }
    if (yawUsableAt(timestamp_ms)) {
        result.status_flags |= STATUS_YAW_USABLE;
    }
    if (gps_health_.failed) {
        result.status_flags |= STATUS_GPS_UNHEALTHY;
    }
    if (magnetic_health_.failed) {
        result.status_flags |= STATUS_MAGNETIC_UNHEALTHY;
    }
    if (encoder_health_.failed) {
        result.status_flags |= STATUS_ENCODER_UNHEALTHY;
    }
    if (result.x_mm < 0 || result.y_mm < 0 ||
        result.x_mm > config_.field_size_x_mm ||
        result.y_mm > config_.field_size_y_mm) {
        result.status_flags |= STATUS_OUTSIDE_FIELD;
    }
    if (!positionUsableAt(timestamp_ms) ||
        !yawUsableAt(timestamp_ms) ||
        gps_health_.failed ||
        magnetic_health_.failed ||
        encoder_health_.failed) {
        result.status_flags |= STATUS_DEGRADED;
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
    if (gps_health_.failed) {
        result.gps_health = SensorHealth::Failed;
    }
    if (encoder_health_.failed) {
        result.encoder_health = SensorHealth::Failed;
    }
    if (magnetic_health_.failed) {
        result.magnetic_health = SensorHealth::Failed;
    }
    result.anomaly_flags = ANOMALY_NONE;
    result.anomaly_since_ms = 0;

    const bool position_usable =
        (result.status_flags & STATUS_POSITION_USABLE) != 0U;
    const bool yaw_usable =
        (result.status_flags & STATUS_YAW_USABLE) != 0U;
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
