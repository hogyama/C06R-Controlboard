#pragma once

#include "localization_config.h"
#include "localization_types.h"

namespace Domain::Localization {

struct MagneticCalibration {
    float hard_iron_uT[3]{};
    float soft_iron[3][3] = {
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 1.0f}};
    bool valid = false;
};

class GyroIntervalIntegrator {
public:
    void reset(uint64_t start_timestamp_us = 0);
    bool push(const Sensor::GyroscopeData& sample);
    GyroPreintegration finish(uint64_t end_timestamp_us);
    uint32_t rejectedSamples() const { return rejected_samples_; }

private:
    Sensor::GyroscopeData previous_{};
    uint64_t interval_start_us_ = 0;
    float integrated_z_rad_ = 0.0f;
    uint16_t sample_count_ = 0;
    uint32_t rejected_samples_ = 0;
    bool have_previous_ = false;
};

class Preprocessor {
public:
    explicit Preprocessor(const Config& config = Config{});

    void reset();
    void setMagneticCalibration(const MagneticCalibration& calibration);
    const MagneticCalibration& magneticCalibration() const {
        return magnetic_calibration_;
    }

    EncoderVelocityObservation processEncoder(
        const EncoderObservation& observation);
    void updateAcceleration(const Sensor::AccelerometerData& acceleration_body);
    MagneticHeadingObservation processMagnetic(
        const Sensor::MagneticData& magnetic_body);
    StationaryObservation stationaryObservation(
        float latest_gyro_z_rad_s,
        bool motor_command_active,
        uint64_t now_us);

    float accelerationNormM_s2() const { return acceleration_norm_m_s2_; }
    float accelerationVariance() const { return acceleration_variance_; }

private:
    static constexpr uint8_t ACCELERATION_WINDOW = 8;

    Config config_;
    MagneticCalibration magnetic_calibration_{};
    EncoderObservation previous_encoder_{};
    bool have_previous_encoder_ = false;
    float latest_encoder_speed_m_s_ = 0.0f;
    float acceleration_body_m_s2_[3]{};
    float acceleration_norm_m_s2_ = 0.0f;
    float acceleration_norm_window_[ACCELERATION_WINDOW]{};
    uint8_t acceleration_window_count_ = 0;
    uint8_t acceleration_window_index_ = 0;
    float acceleration_variance_ = 0.0f;
    uint64_t acceleration_timestamp_us_ = 0;
    uint64_t stationary_since_us_ = 0;
    float previous_magnetic_heading_rad_ = 0.0f;
    bool have_previous_magnetic_heading_ = false;
};

} // namespace Domain::Localization
