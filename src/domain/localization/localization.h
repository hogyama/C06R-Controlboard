#pragma once

#include "localization_ekf.h"
#include "localization_health.h"
#include "localization_preprocessor.h"

namespace Domain::Localization {

struct CycleInput {
    uint64_t timestamp_us;
    GyroPreintegration gyroscope;
    EncoderVelocityObservation encoder;
    MagneticHeadingObservation magnetic;
    StationaryObservation stationary;
    bool has_encoder;
    bool has_magnetic;
    bool motor_command_active;
};

class Localization {
public:
    static constexpr uint16_t HISTORY_CAPACITY = 128;

    explicit Localization(const Config& config = Config{});

    void reset();
    bool initializeFromGps(
        const GpsObservation& gps,
        const MagneticHeadingObservation* magnetic,
        float gyro_bias_rad_s);
    bool processCycle(const CycleInput& input);
    bool processGps(const GpsObservation& observation);
    void noteAcceleration(const Sensor::AccelerometerData& observation);
    bool recoverFromLastGood();
    LocalizationEstimate estimate(uint64_t now_us);
    bool setGyroBias(float bias_rad_s, float variance_rad2_s2) {
        return ekf_.setGyroBias(bias_rad_s, variance_rad2_s2);
    }

    const Ekf5& ekf() const { return ekf_; }
    const SensorHealthReport& gpsHealth() const { return gps_health_.report(); }
    const SensorHealthReport& encoderHealth() const {
        return encoder_health_.report();
    }
    const SensorHealthReport& gyroHealth() const { return gyro_health_.report(); }
    const SensorHealthReport& accelHealth() const { return accel_health_.report(); }
    const SensorHealthReport& magneticHealth() const {
        return magnetic_health_.report();
    }

private:
    struct GpsResidual {
        float east_m;
        float north_m;
    };
    struct HistoryEntry {
        Ekf5::Snapshot before;
        CycleInput input;
    };

    Config config_;
    Ekf5 ekf_;
    HistoryEntry history_[HISTORY_CAPACITY]{};
    uint16_t history_oldest_ = 0;
    uint16_t history_count_ = 0;
    HealthTracker gps_health_{};
    HealthTracker encoder_health_{};
    HealthTracker gyro_health_{};
    HealthTracker accel_health_{};
    HealthTracker magnetic_health_{};
    uint32_t active_sensor_flags_ = 0;
    uint32_t fault_flags_ = 0;
    uint16_t cycle_status_flags_ = 0;
    float latest_gyro_z_rad_s_ = 0.0f;
    float magnetic_field_uT_ = 0.0f;
    MagneticRejectReason magnetic_reject_reason_ =
        MagneticRejectReason::None;
    GpsCourseRejectReason gps_course_reject_reason_ =
        GpsCourseRejectReason::None;
    Ekf5::Snapshot last_good_snapshot_{};
    bool have_last_good_snapshot_ = false;
    GpsResidual gps_residuals_[9]{};
    uint8_t gps_residual_count_ = 0;
    uint8_t gps_residual_write_ = 0;
    uint32_t gps_warp_count_ = 0;
    uint64_t gps_warp_timestamp_us_ = 0;
    float gps_warp_east_m_ = 0.0f;
    float gps_warp_north_m_ = 0.0f;

    HistoryEntry& historyAt(uint16_t logical_index);
    const HistoryEntry& historyAt(uint16_t logical_index) const;
    void appendHistory(const HistoryEntry& entry);
    bool applyCycle(const CycleInput& input, bool record_health);
    bool applyGpsNow(const GpsObservation& observation, bool record_health);
    bool replayDelayedGps(const GpsObservation& observation);
    void recordGpsResidual(const GpsObservation& observation);
    bool applyGpsWarp(const GpsObservation& observation);
};

} // namespace Domain::Localization
