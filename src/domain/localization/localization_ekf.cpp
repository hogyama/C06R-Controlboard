#include "localization_ekf.h"

#include <math.h>
#include <string.h>

namespace Domain::Localization {
namespace {

constexpr float MINIMUM_VARIANCE = 1.0e-9f;
constexpr float PI = 3.14159265358979323846f;

float square(float value)
{
    return value * value;
}

bool finiteVector(const float* values, uint8_t count)
{
    for (uint8_t i = 0; i < count; ++i) {
        if (!isfinite(values[i])) return false;
    }
    return true;
}

} // namespace

Ekf5::Ekf5(const Config& config) : config_(config)
{
    reset();
}

void Ekf5::reset()
{
    memset(state_, 0, sizeof(state_));
    memset(covariance_, 0, sizeof(covariance_));
    timestamp_us_ = 0;
    last_mahalanobis_ = 0.0f;
    initialized_ = false;
    yaw_initialized_ = false;
    covariance_valid_ = true;
}

void Ekf5::initialize(
    float px_m,
    float py_m,
    float theta_rad,
    float velocity_m_s,
    float gyro_bias_rad_s,
    uint64_t timestamp_us,
    bool yaw_initialized)
{
    reset();
    state_[POSITION_X] = px_m;
    state_[POSITION_Y] = py_m;
    state_[YAW] = wrapPi(theta_rad);
    state_[FORWARD_VELOCITY] = velocity_m_s;
    state_[GYRO_BIAS_Z] = gyro_bias_rad_s;
    covariance_[POSITION_X][POSITION_X] =
        square(config_.initial_position_std_m);
    covariance_[POSITION_Y][POSITION_Y] =
        square(config_.initial_position_std_m);
    covariance_[YAW][YAW] = square(config_.initial_yaw_std_rad);
    covariance_[FORWARD_VELOCITY][FORWARD_VELOCITY] =
        square(config_.initial_velocity_std_m_s);
    covariance_[GYRO_BIAS_Z][GYRO_BIAS_Z] =
        square(config_.initial_bias_std_rad_s);
    timestamp_us_ = timestamp_us;
    initialized_ = timestamp_us != 0U && finiteVector(state_, STATE_COUNT);
    yaw_initialized_ = initialized_ && yaw_initialized;
}

float Ekf5::wrapPi(float angle_rad)
{
    constexpr float TWO_PI = 2.0f * PI;
    angle_rad = fmodf(
        angle_rad + PI, TWO_PI);
    if (angle_rad < 0.0f) angle_rad += TWO_PI;
    return angle_rad - PI;
}

bool Ekf5::predict(const GyroPreintegration& integration)
{
    if (!initialized_ || !integration.valid ||
        integration.end_timestamp_us <= timestamp_us_ ||
        integration.end_timestamp_us <= integration.start_timestamp_us ||
        !isfinite(integration.integrated_z_rad)) {
        return false;
    }

    const uint64_t effective_start_us =
        integration.start_timestamp_us > timestamp_us_
            ? integration.start_timestamp_us : timestamp_us_;
    if (integration.end_timestamp_us <= effective_start_us) return false;
    const float dt = static_cast<float>(
        integration.end_timestamp_us - effective_start_us) * 1.0e-6f;
    if (!(dt > 0.0f) || dt > 2.0f) return false;

    const float interval_scale =
        static_cast<float>(integration.end_timestamp_us - effective_start_us) /
        static_cast<float>(
            integration.end_timestamp_us - integration.start_timestamp_us);
    const float integrated_raw = integration.integrated_z_rad * interval_scale;
    const float delta_theta = integrated_raw - state_[GYRO_BIAS_Z] * dt;
    const float midpoint_yaw = state_[YAW] + 0.5f * delta_theta;
    const float distance = state_[FORWARD_VELOCITY] * dt;
    const float cosine = cosf(midpoint_yaw);
    const float sine = sinf(midpoint_yaw);

    float jacobian[STATE_COUNT][STATE_COUNT]{};
    for (uint8_t i = 0; i < STATE_COUNT; ++i) jacobian[i][i] = 1.0f;
    jacobian[POSITION_X][YAW] = -distance * sine;
    jacobian[POSITION_X][FORWARD_VELOCITY] = dt * cosine;
    jacobian[POSITION_X][GYRO_BIAS_Z] = 0.5f * distance * dt * sine;
    jacobian[POSITION_Y][YAW] = distance * cosine;
    jacobian[POSITION_Y][FORWARD_VELOCITY] = dt * sine;
    jacobian[POSITION_Y][GYRO_BIAS_Z] = -0.5f * distance * dt * cosine;
    jacobian[YAW][GYRO_BIAS_Z] = -dt;

    float temporary[STATE_COUNT][STATE_COUNT]{};
    float predicted[STATE_COUNT][STATE_COUNT]{};
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                temporary[row][column] +=
                    jacobian[row][k] * covariance_[k][column];
            }
        }
    }
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                predicted[row][column] +=
                    temporary[row][k] * jacobian[column][k];
            }
        }
    }

    // Noise inputs are distance increment, angle increment, velocity RW,
    // and gyro-bias RW. This keeps Q symmetric by construction.
    float noise_map[STATE_COUNT][4]{};
    noise_map[POSITION_X][0] = cosine;
    noise_map[POSITION_Y][0] = sine;
    noise_map[POSITION_X][1] = -0.5f * distance * sine;
    noise_map[POSITION_Y][1] = 0.5f * distance * cosine;
    noise_map[YAW][1] = 1.0f;
    noise_map[FORWARD_VELOCITY][2] = 1.0f;
    noise_map[GYRO_BIAS_Z][3] = 1.0f;
    const float noise_variance[4] = {
        fmaxf(config_.distance_noise_m2_s, 0.0f) * dt,
        fmaxf(config_.angle_noise_rad2_s, 0.0f) * dt,
        fmaxf(config_.velocity_noise_m2_s3, 0.0f) * dt,
        fmaxf(config_.gyro_bias_noise_rad2_s3, 0.0f) * dt
    };
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t n = 0; n < 4; ++n) {
                predicted[row][column] +=
                    noise_map[row][n] * noise_variance[n] *
                    noise_map[column][n];
            }
        }
    }

    state_[POSITION_X] += distance * cosine;
    state_[POSITION_Y] += distance * sine;
    state_[YAW] = wrapPi(state_[YAW] + delta_theta);
    memcpy(covariance_, predicted, sizeof(covariance_));
    timestamp_us_ = integration.end_timestamp_us;
    stabilizeCovariance();
    return covariance_valid_;
}

bool Ekf5::updateEncoderVelocity(
    const EncoderVelocityObservation& observation)
{
    if (!observation.valid || !isfinite(observation.velocity_m_s)) return false;
    const float residual[2] = {
        observation.velocity_m_s - state_[FORWARD_VELOCITY], 0.0f};
    float jacobian[2][STATE_COUNT]{};
    jacobian[0][FORWARD_VELOCITY] = 1.0f;
    const float variance[2] = {
        fmaxf(observation.variance_m2_s2, MINIMUM_VARIANCE), 0.0f};
    return measurementUpdate(
        residual, jacobian, variance, 1, config_.mahalanobis_gate_1d);
}

bool Ekf5::updateMagneticHeading(
    const MagneticHeadingObservation& observation)
{
    if (!observation.valid || !isfinite(observation.heading_rad)) return false;
    const float residual[2] = {
        wrapPi(observation.heading_rad - state_[YAW]), 0.0f};
    float jacobian[2][STATE_COUNT]{};
    jacobian[0][YAW] = 1.0f;
    const float variance[2] = {
        fmaxf(observation.variance_rad2, MINIMUM_VARIANCE), 0.0f};
    const bool accepted = measurementUpdate(
        residual, jacobian, variance, 1, config_.mahalanobis_gate_1d);
    if (accepted) yaw_initialized_ = true;
    return accepted;
}

bool Ekf5::updateGpsPosition(const GpsObservation& observation)
{
    if (!observation.position_valid || !isfinite(observation.east_m) ||
        !isfinite(observation.north_m)) return false;
    const float residual[2] = {
        observation.east_m - state_[POSITION_X],
        observation.north_m - state_[POSITION_Y]};
    float jacobian[2][STATE_COUNT]{};
    jacobian[0][POSITION_X] = 1.0f;
    jacobian[1][POSITION_Y] = 1.0f;
    const float standard_deviation = fmaxf(
        observation.horizontal_accuracy_m,
        config_.gps_position_noise_floor_m);
    const float variance[2] = {
        square(standard_deviation), square(standard_deviation)};
    return measurementUpdate(
        residual, jacobian, variance, 2, config_.mahalanobis_gate_2d);
}

bool Ekf5::updateGpsVelocity(const GpsObservation& observation)
{
    if (!observation.velocity_valid ||
        !isfinite(observation.velocity_east_m_s) ||
        !isfinite(observation.velocity_north_m_s)) return false;
    const float yaw = state_[YAW];
    const float velocity = state_[FORWARD_VELOCITY];
    const float cosine = cosf(yaw);
    const float sine = sinf(yaw);
    const float residual[2] = {
        observation.velocity_east_m_s - velocity * cosine,
        observation.velocity_north_m_s - velocity * sine};
    float jacobian[2][STATE_COUNT]{};
    jacobian[0][YAW] = -velocity * sine;
    jacobian[0][FORWARD_VELOCITY] = cosine;
    jacobian[1][YAW] = velocity * cosine;
    jacobian[1][FORWARD_VELOCITY] = sine;
    const float standard_deviation = fmaxf(
        observation.speed_accuracy_m_s,
        config_.gps_velocity_noise_floor_m_s);
    const float variance[2] = {
        square(standard_deviation), square(standard_deviation)};
    const bool accepted = measurementUpdate(
        residual, jacobian, variance, 2, config_.mahalanobis_gate_2d);
    if (accepted && hypotf(
            observation.velocity_east_m_s,
            observation.velocity_north_m_s) >=
            config_.gps_minimum_course_speed_m_s) {
        yaw_initialized_ = true;
    }
    return accepted;
}

bool Ekf5::updateZeroVelocity(float variance_m2_s2)
{
    EncoderVelocityObservation observation{};
    observation.velocity_m_s = 0.0f;
    observation.variance_m2_s2 = fmaxf(variance_m2_s2, MINIMUM_VARIANCE);
    observation.valid = true;
    return updateEncoderVelocity(observation);
}

bool Ekf5::updateZaru(
    float measured_gyro_z_rad_s,
    float variance_rad2_s2)
{
    if (!isfinite(measured_gyro_z_rad_s)) return false;
    const float residual[2] = {
        measured_gyro_z_rad_s - state_[GYRO_BIAS_Z], 0.0f};
    float jacobian[2][STATE_COUNT]{};
    jacobian[0][GYRO_BIAS_Z] = 1.0f;
    const float variance[2] = {
        fmaxf(variance_rad2_s2, MINIMUM_VARIANCE), 0.0f};
    return measurementUpdate(
        residual, jacobian, variance, 1, config_.mahalanobis_gate_1d);
}

bool Ekf5::setGyroBias(float bias_rad_s, float variance_rad2_s2)
{
    if (!initialized_ || !isfinite(bias_rad_s) ||
        !isfinite(variance_rad2_s2)) return false;
    state_[GYRO_BIAS_Z] = bias_rad_s;
    for (uint8_t i = 0; i < STATE_COUNT; ++i) {
        covariance_[GYRO_BIAS_Z][i] = 0.0f;
        covariance_[i][GYRO_BIAS_Z] = 0.0f;
    }
    covariance_[GYRO_BIAS_Z][GYRO_BIAS_Z] =
        fmaxf(variance_rad2_s2, MINIMUM_VARIANCE);
    stabilizeCovariance();
    return covariance_valid_;
}

Ekf5::Snapshot Ekf5::snapshot() const
{
    Snapshot result{};
    memcpy(result.state, state_, sizeof(state_));
    memcpy(result.covariance, covariance_, sizeof(covariance_));
    result.timestamp_us = timestamp_us_;
    result.initialized = initialized_;
    result.yaw_initialized = yaw_initialized_;
    return result;
}

bool Ekf5::restore(const Snapshot& snapshot)
{
    if (!snapshot.initialized || snapshot.timestamp_us == 0U ||
        !finiteVector(snapshot.state, STATE_COUNT)) return false;
    memcpy(state_, snapshot.state, sizeof(state_));
    memcpy(covariance_, snapshot.covariance, sizeof(covariance_));
    timestamp_us_ = snapshot.timestamp_us;
    initialized_ = snapshot.initialized;
    yaw_initialized_ = snapshot.yaw_initialized;
    stabilizeCovariance();
    return covariance_valid_;
}

bool Ekf5::measurementUpdate(
    const float residual[MAX_MEASUREMENT_DIMENSION],
    const float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT],
    const float variance[MAX_MEASUREMENT_DIMENSION],
    uint8_t dimension,
    float gate)
{
    if (!initialized_ || dimension == 0 || dimension > 2 ||
        !finiteVector(residual, dimension)) return false;

    float innovation_covariance[2][2]{};
    for (uint8_t row = 0; row < dimension; ++row) {
        for (uint8_t column = 0; column < dimension; ++column) {
            for (uint8_t i = 0; i < STATE_COUNT; ++i) {
                for (uint8_t j = 0; j < STATE_COUNT; ++j) {
                    innovation_covariance[row][column] +=
                        jacobian[row][i] * covariance_[i][j] *
                        jacobian[column][j];
                }
            }
            if (row == column) {
                innovation_covariance[row][column] += variance[row];
            }
        }
    }

    float inverse[2][2]{};
    if (dimension == 1) {
        if (!(innovation_covariance[0][0] > MINIMUM_VARIANCE)) return false;
        inverse[0][0] = 1.0f / innovation_covariance[0][0];
    } else {
        const float determinant =
            innovation_covariance[0][0] * innovation_covariance[1][1] -
            innovation_covariance[0][1] * innovation_covariance[1][0];
        if (!(determinant > MINIMUM_VARIANCE) || !isfinite(determinant)) {
            return false;
        }
        inverse[0][0] = innovation_covariance[1][1] / determinant;
        inverse[0][1] = -innovation_covariance[0][1] / determinant;
        inverse[1][0] = -innovation_covariance[1][0] / determinant;
        inverse[1][1] = innovation_covariance[0][0] / determinant;
    }

    last_mahalanobis_ = 0.0f;
    for (uint8_t row = 0; row < dimension; ++row) {
        for (uint8_t column = 0; column < dimension; ++column) {
            last_mahalanobis_ +=
                residual[row] * inverse[row][column] * residual[column];
        }
    }
    if (!isfinite(last_mahalanobis_) || last_mahalanobis_ > gate) return false;

    float gain[STATE_COUNT][2]{};
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        for (uint8_t measurement = 0; measurement < dimension; ++measurement) {
            for (uint8_t i = 0; i < STATE_COUNT; ++i) {
                for (uint8_t j = 0; j < dimension; ++j) {
                    gain[state][measurement] +=
                        covariance_[state][i] * jacobian[j][i] *
                        inverse[j][measurement];
                }
            }
        }
    }
    for (uint8_t state = 0; state < STATE_COUNT; ++state) {
        for (uint8_t measurement = 0; measurement < dimension; ++measurement) {
            state_[state] += gain[state][measurement] * residual[measurement];
        }
    }
    state_[YAW] = wrapPi(state_[YAW]);

    float identity_minus_kh[STATE_COUNT][STATE_COUNT]{};
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        identity_minus_kh[row][row] = 1.0f;
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t measurement = 0;
                 measurement < dimension;
                 ++measurement) {
                identity_minus_kh[row][column] -=
                    gain[row][measurement] * jacobian[measurement][column];
            }
        }
    }

    float temporary[STATE_COUNT][STATE_COUNT]{};
    float updated[STATE_COUNT][STATE_COUNT]{};
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                temporary[row][column] +=
                    identity_minus_kh[row][k] * covariance_[k][column];
            }
        }
    }
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = 0; column < STATE_COUNT; ++column) {
            for (uint8_t k = 0; k < STATE_COUNT; ++k) {
                updated[row][column] +=
                    temporary[row][k] * identity_minus_kh[column][k];
            }
            for (uint8_t measurement = 0;
                 measurement < dimension;
                 ++measurement) {
                updated[row][column] +=
                    gain[row][measurement] * variance[measurement] *
                    gain[column][measurement];
            }
        }
    }
    memcpy(covariance_, updated, sizeof(covariance_));
    stabilizeCovariance();
    return covariance_valid_;
}

void Ekf5::stabilizeCovariance()
{
    covariance_valid_ = finiteVector(state_, STATE_COUNT);
    for (uint8_t row = 0; row < STATE_COUNT; ++row) {
        for (uint8_t column = row; column < STATE_COUNT; ++column) {
            const float symmetric = 0.5f *
                (covariance_[row][column] + covariance_[column][row]);
            covariance_[row][column] = symmetric;
            covariance_[column][row] = symmetric;
            if (!isfinite(symmetric)) covariance_valid_ = false;
        }
        if (covariance_[row][row] < MINIMUM_VARIANCE) {
            covariance_[row][row] = MINIMUM_VARIANCE;
        }
    }
}

} // namespace Domain::Localization
