#include "localization_preprocessor.h"

#include "localization_ekf.h"

#include <math.h>

namespace Domain::Localization {
namespace {

float square(float value)
{
    return value * value;
}

bool finiteGyroscope(const Sensor::GyroscopeData& sample)
{
    return sample.metadata.valid && sample.metadata.timestamp_us != 0U &&
        isfinite(sample.x_rad_s) && isfinite(sample.y_rad_s) &&
        isfinite(sample.z_rad_s);
}

} // namespace

void GyroIntervalIntegrator::reset(uint64_t start_timestamp_us)
{
    previous_ = {};
    interval_start_us_ = start_timestamp_us;
    integrated_z_rad_ = 0.0f;
    sample_count_ = 0;
    have_previous_ = false;
}

bool GyroIntervalIntegrator::push(const Sensor::GyroscopeData& sample)
{
    if (!finiteGyroscope(sample)) {
        ++rejected_samples_;
        return false;
    }
    if (have_previous_ &&
        (sample.metadata.timestamp_us <= previous_.metadata.timestamp_us ||
         sample.metadata.source != previous_.metadata.source)) {
        ++rejected_samples_;
        previous_ = sample;
        interval_start_us_ = sample.metadata.timestamp_us;
        integrated_z_rad_ = 0.0f;
        sample_count_ = 1;
        return false;
    }
    if (!have_previous_) {
        previous_ = sample;
        have_previous_ = true;
        if (interval_start_us_ == 0U) {
            interval_start_us_ = sample.metadata.timestamp_us;
        }
        sample_count_ = 1;
        return true;
    }

    const float dt = static_cast<float>(
        sample.metadata.timestamp_us - previous_.metadata.timestamp_us) *
        1.0e-6f;
    integrated_z_rad_ +=
        0.5f * (previous_.z_rad_s + sample.z_rad_s) * dt;
    previous_ = sample;
    if (sample_count_ != UINT16_MAX) ++sample_count_;
    return true;
}

GyroPreintegration GyroIntervalIntegrator::finish(uint64_t end_timestamp_us)
{
    GyroPreintegration result{};
    result.start_timestamp_us = interval_start_us_;
    result.end_timestamp_us = end_timestamp_us;
    result.integrated_z_rad = integrated_z_rad_;
    result.sample_count = sample_count_;
    if (have_previous_) {
        result.latest_z_rad_s = previous_.z_rad_s;
        result.source = previous_.metadata.source;
        if (end_timestamp_us > previous_.metadata.timestamp_us) {
            const uint64_t tail_us =
                end_timestamp_us - previous_.metadata.timestamp_us;
            // Hold the last sample only over a short scheduling gap.
            if (tail_us <= 20000U) {
                result.integrated_z_rad +=
                    previous_.z_rad_s * static_cast<float>(tail_us) * 1.0e-6f;
            }
        }
    }
    result.valid = have_previous_ && sample_count_ >= 2U &&
        result.start_timestamp_us != 0U &&
        end_timestamp_us > result.start_timestamp_us;

    integrated_z_rad_ = 0.0f;
    sample_count_ = have_previous_ ? 1U : 0U;
    interval_start_us_ = end_timestamp_us;
    if (have_previous_ && end_timestamp_us > previous_.metadata.timestamp_us &&
        end_timestamp_us - previous_.metadata.timestamp_us <= 20000U) {
        previous_.metadata.timestamp_us = end_timestamp_us;
    }
    return result;
}

Preprocessor::Preprocessor(const Config& config) : config_(config)
{
    reset();
}

void Preprocessor::reset()
{
    previous_encoder_ = {};
    have_previous_encoder_ = false;
    latest_encoder_speed_m_s_ = 0.0f;
    acceleration_body_m_s2_[0] = 0.0f;
    acceleration_body_m_s2_[1] = 0.0f;
    acceleration_body_m_s2_[2] = 0.0f;
    acceleration_norm_m_s2_ = 0.0f;
    for (float& value : acceleration_norm_window_) value = 0.0f;
    acceleration_window_count_ = 0;
    acceleration_window_index_ = 0;
    acceleration_variance_ = 0.0f;
    acceleration_timestamp_us_ = 0;
    stationary_since_us_ = 0;
    previous_magnetic_heading_rad_ = 0.0f;
    have_previous_magnetic_heading_ = false;
}

void Preprocessor::setMagneticCalibration(
    const MagneticCalibration& calibration)
{
    magnetic_calibration_ = calibration;
}

EncoderVelocityObservation Preprocessor::processEncoder(
    const EncoderObservation& observation)
{
    EncoderVelocityObservation result{};
    result.metadata = observation.metadata;
    if (!observation.metadata.valid || !observation.left_valid ||
        !observation.right_valid || observation.metadata.timestamp_us == 0U) {
        return result;
    }
    if (!have_previous_encoder_) {
        previous_encoder_ = observation;
        have_previous_encoder_ = true;
        return result;
    }
    const uint32_t expected_sequence =
        previous_encoder_.sequence == UINT32_MAX
            ? 1U : previous_encoder_.sequence + 1U;
    if (observation.metadata.timestamp_us <=
            previous_encoder_.metadata.timestamp_us ||
        observation.sequence != expected_sequence) {
        previous_encoder_ = observation;
        return result;
    }

    const uint64_t dt_us = observation.metadata.timestamp_us -
        previous_encoder_.metadata.timestamp_us;
    const int64_t delta_left_mm =
        static_cast<int64_t>(observation.left_cumulative_mm) -
        previous_encoder_.left_cumulative_mm;
    const int64_t delta_right_mm =
        static_cast<int64_t>(observation.right_cumulative_mm) -
        previous_encoder_.right_cumulative_mm;
    previous_encoder_ = observation;
    if (dt_us < config_.encoder_minimum_interval_us ||
        dt_us > config_.encoder_maximum_interval_us) return result;

    const float inverse_dt = 1000.0f / static_cast<float>(dt_us);
    result.left_velocity_m_s = delta_left_mm * inverse_dt;
    result.right_velocity_m_s = delta_right_mm * inverse_dt;
    result.velocity_m_s =
        0.5f * (result.left_velocity_m_s + result.right_velocity_m_s);
    result.interval_us = dt_us;
    result.delta_distance_m =
        0.5f * static_cast<float>(delta_left_mm + delta_right_mm) * 0.001f;
    result.delta_yaw_rad = config_.encoder_track_width_m > 0.0f
        ? static_cast<float>(delta_right_mm - delta_left_mm) * 0.001f /
            config_.encoder_track_width_m
        : 0.0f;
    result.yaw_rate_rad_s =
        result.delta_yaw_rad * 1000000.0f / static_cast<float>(dt_us);
    result.yaw_variance_rad2 =
        fmaxf(config_.encoder_yaw_noise_rad2_s, 0.0f) *
        static_cast<float>(dt_us) * 1.0e-6f;
    result.variance_m2_s2 = square(config_.encoder_velocity_std_m_s);
    if (!isfinite(result.velocity_m_s) ||
        fabsf(result.left_velocity_m_s) > config_.encoder_maximum_speed_m_s ||
        fabsf(result.right_velocity_m_s) > config_.encoder_maximum_speed_m_s ||
        fabsf(result.left_velocity_m_s - result.right_velocity_m_s) >
            config_.encoder_maximum_left_right_difference_m_s) {
        return result;
    }
    latest_encoder_speed_m_s_ = result.velocity_m_s;
    result.valid = true;
    return result;
}

void Preprocessor::updateAcceleration(
    const Sensor::AccelerometerData& acceleration_body)
{
    if (!acceleration_body.metadata.valid ||
        !isfinite(acceleration_body.x_m_s2) ||
        !isfinite(acceleration_body.y_m_s2) ||
        !isfinite(acceleration_body.z_m_s2)) return;
    acceleration_body_m_s2_[0] = acceleration_body.x_m_s2;
    acceleration_body_m_s2_[1] = acceleration_body.y_m_s2;
    acceleration_body_m_s2_[2] = acceleration_body.z_m_s2;
    acceleration_norm_m_s2_ = hypotf(
        hypotf(acceleration_body.x_m_s2, acceleration_body.y_m_s2),
        acceleration_body.z_m_s2);
    acceleration_timestamp_us_ = acceleration_body.metadata.timestamp_us;
    acceleration_norm_window_[acceleration_window_index_] =
        acceleration_norm_m_s2_;
    acceleration_window_index_ = static_cast<uint8_t>(
        (acceleration_window_index_ + 1U) % ACCELERATION_WINDOW);
    if (acceleration_window_count_ < ACCELERATION_WINDOW) {
        ++acceleration_window_count_;
    }
    float mean = 0.0f;
    for (uint8_t i = 0; i < acceleration_window_count_; ++i) {
        mean += acceleration_norm_window_[i];
    }
    mean /= acceleration_window_count_;
    acceleration_variance_ = 0.0f;
    for (uint8_t i = 0; i < acceleration_window_count_; ++i) {
        acceleration_variance_ += square(acceleration_norm_window_[i] - mean);
    }
    acceleration_variance_ /= acceleration_window_count_;
}

MagneticDiagnostic Preprocessor::magneticDiagnostic(
    const Sensor::MagneticData& magnetic_body)
    const
{
    MagneticDiagnostic result{};
    if (!magnetic_body.metadata.valid || !isfinite(magnetic_body.x_uT) ||
        !isfinite(magnetic_body.y_uT) || !isfinite(magnetic_body.z_uT)) {
        return result;
    }

    const bool apply_board_calibration =
        magnetic_body.metadata.source == Sensor::Source::BoardI2c &&
        magnetic_calibration_.valid;
    result.calibration_applied = apply_board_calibration;
    const float raw[3] = {
        magnetic_body.x_uT - (apply_board_calibration
            ? magnetic_calibration_.hard_iron_uT[0] : 0.0f),
        magnetic_body.y_uT - (apply_board_calibration
            ? magnetic_calibration_.hard_iron_uT[1] : 0.0f),
        magnetic_body.z_uT - (apply_board_calibration
            ? magnetic_calibration_.hard_iron_uT[2] : 0.0f)};
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t column = 0; column < 3; ++column) {
            const float coefficient = apply_board_calibration
                ? magnetic_calibration_.soft_iron[row][column]
                : (row == column ? 1.0f : 0.0f);
            result.corrected_uT[row] += coefficient * raw[column];
        }
        if (!isfinite(result.corrected_uT[row])) return MagneticDiagnostic{};
    }
    result.vector_valid = true;
    result.field_strength_uT =
        hypotf(hypotf(result.corrected_uT[0], result.corrected_uT[1]),
            result.corrected_uT[2]);
    if (result.field_strength_uT < config_.magnetic_minimum_total_uT ||
        result.field_strength_uT > config_.magnetic_maximum_total_uT ||
        acceleration_norm_m_s2_ < 1.0f) return result;

    const float up[3] = {
        acceleration_body_m_s2_[0] / acceleration_norm_m_s2_,
        acceleration_body_m_s2_[1] / acceleration_norm_m_s2_,
        acceleration_body_m_s2_[2] / acceleration_norm_m_s2_};
    const float vertical =
        result.corrected_uT[0] * up[0] +
        result.corrected_uT[1] * up[1] +
        result.corrected_uT[2] * up[2];
    const float horizontal[3] = {
        result.corrected_uT[0] - vertical * up[0],
        result.corrected_uT[1] - vertical * up[1],
        result.corrected_uT[2] - vertical * up[2]};
    const float horizontal_strength =
        hypotf(horizontal[0], horizontal[1]);
    if (horizontal_strength < config_.magnetic_minimum_horizontal_uT) {
        return result;
    }

    result.heading_rad = Ekf5::wrapPi(
        atan2f(horizontal[0], horizontal[1]) -
        config_.magnetic_declination_rad);
    result.heading_valid = isfinite(result.heading_rad);
    return result;
}

MagneticHeadingObservation Preprocessor::processMagnetic(
    const Sensor::MagneticData& magnetic_body)
{
    MagneticHeadingObservation result{};
    result.metadata = magnetic_body.metadata;
    const MagneticDiagnostic diagnostic = magneticDiagnostic(magnetic_body);
    result.field_strength_uT = diagnostic.field_strength_uT;
    result.heading_rad = diagnostic.heading_rad;
    if (!diagnostic.heading_valid) return result;
    if (have_previous_magnetic_heading_ &&
        fabsf(Ekf5::wrapPi(
            result.heading_rad - previous_magnetic_heading_rad_)) >
            config_.magnetic_maximum_step_rad) {
        return result;
    }
    previous_magnetic_heading_rad_ = result.heading_rad;
    have_previous_magnetic_heading_ = true;
    result.variance_rad2 = square(config_.magnetic_heading_std_rad);
    result.valid = true;
    return result;
}

StationaryObservation Preprocessor::stationaryObservation(
    float latest_gyro_z_rad_s,
    bool motor_command_active,
    uint64_t now_us)
{
    StationaryObservation result{};
    result.mean_gyro_z_rad_s = latest_gyro_z_rad_s;
    result.timestamp_us = now_us;
    const bool conditions =
        !motor_command_active &&
        fabsf(latest_encoder_speed_m_s_) <=
            config_.stationary_encoder_speed_m_s &&
        fabsf(latest_gyro_z_rad_s) <= config_.stationary_gyro_rad_s &&
        fabsf(acceleration_norm_m_s2_ - 9.80665f) <=
            config_.stationary_accel_norm_tolerance_m_s2 &&
        acceleration_window_count_ == ACCELERATION_WINDOW &&
        acceleration_variance_ <= config_.stationary_accel_variance_m2_s4;
    if (!conditions) {
        stationary_since_us_ = 0;
        return result;
    }
    if (stationary_since_us_ == 0U) stationary_since_us_ = now_us;
    result.stationary = now_us >= stationary_since_us_ &&
        now_us - stationary_since_us_ >= config_.stationary_hold_us;
    return result;
}

} // namespace Domain::Localization
