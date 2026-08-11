#pragma once

#include "localization_config.h"
#include "localization_types.h"

namespace Domain::Localization {

class Ekf5 {
public:
    struct Snapshot {
        float state[STATE_COUNT];
        float covariance[STATE_COUNT][STATE_COUNT];
        uint64_t timestamp_us;
        bool initialized;
        bool yaw_initialized;
    };

    explicit Ekf5(const Config& config = Config{});

    void reset();
    void initialize(
        float px_m,
        float py_m,
        float theta_rad,
        float velocity_m_s,
        float gyro_bias_rad_s,
        uint64_t timestamp_us,
        bool yaw_initialized);

    bool initialized() const { return initialized_; }
    bool yawInitialized() const { return yaw_initialized_; }
    uint64_t timestampUs() const { return timestamp_us_; }
    const float* state() const { return state_; }
    const float (*covariance() const)[STATE_COUNT] { return covariance_; }

    bool predict(const GyroPreintegration& integration);
    bool updateEncoderVelocity(const EncoderVelocityObservation& observation);
    bool updateMagneticHeading(const MagneticHeadingObservation& observation);
    bool updateGpsPosition(const GpsObservation& observation);
    bool updateGpsVelocity(const GpsObservation& observation);
    bool updateZeroVelocity(float variance_m2_s2);
    bool updateZaru(float measured_gyro_z_rad_s, float variance_rad2_s2);
    bool setGyroBias(float bias_rad_s, float variance_rad2_s2);

    float lastMahalanobis() const { return last_mahalanobis_; }
    bool covarianceValid() const { return covariance_valid_; }
    Snapshot snapshot() const;
    bool restore(const Snapshot& snapshot);

    static float wrapPi(float angle_rad);

private:
    static constexpr uint8_t MAX_MEASUREMENT_DIMENSION = 2;

    Config config_;
    float state_[STATE_COUNT]{};
    float covariance_[STATE_COUNT][STATE_COUNT]{};
    uint64_t timestamp_us_ = 0;
    float last_mahalanobis_ = 0.0f;
    bool initialized_ = false;
    bool yaw_initialized_ = false;
    bool covariance_valid_ = true;

    bool measurementUpdate(
        const float residual[MAX_MEASUREMENT_DIMENSION],
        const float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT],
        const float variance[MAX_MEASUREMENT_DIMENSION],
        uint8_t dimension,
        float gate);
    void stabilizeCovariance();
};

} // namespace Domain::Localization
