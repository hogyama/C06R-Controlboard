#pragma once

#include <stdint.h>

#include "domain/sensor/sensor_types.h"

namespace Domain::Localization {

constexpr uint8_t STATE_COUNT = 5;

enum StateIndex : uint8_t {
    POSITION_X = 0,
    POSITION_Y,
    YAW,
    FORWARD_VELOCITY,
    GYRO_BIAS_Z
};

enum class SensorHealth : uint8_t {
    Disabled = 0,
    Fresh,
    Stale,
    Failed
};

enum class Quality : uint8_t {
    Uninitialized = 0,
    Normal,
    Degraded,
    Unreliable,
    Failed
};

enum class MagneticRejectReason : uint8_t {
    None = 0,
    InvalidInput,
    NonMonotonicTimestamp,
    TotalStrengthOutOfRange,
    GravityUnavailable,
    HighTilt,
    HorizontalStrengthTooLow,
    SuddenChange,
    MahalanobisGate,
    Timeout
};

enum class GpsCourseRejectReason : uint8_t {
    None = 0,
    VelocityUnavailable,
    SpeedTooLow,
    SpeedAccuracyTooLarge,
    InvalidCourseUncertainty,
    MahalanobisGate
};

enum StatusFlag : uint16_t {
    STATUS_NONE                = 0,
    STATUS_INITIALIZED         = 1U << 0,
    STATUS_POSITION_USABLE     = 1U << 1,
    STATUS_YAW_USABLE          = 1U << 2,
    STATUS_GPS_USED            = 1U << 3,
    STATUS_ENCODER_USED        = 1U << 4,
    STATUS_IMU_USED            = 1U << 5,
    STATUS_MAGNETIC_USED       = 1U << 6,
    STATUS_GPS_REJECTED        = 1U << 7,
    STATUS_MAGNETIC_REJECTED   = 1U << 8,
    STATUS_DEGRADED            = 1U << 9,
    STATUS_ENCODER_REJECTED    = 1U << 10,
    STATUS_IMU_REJECTED        = 1U << 11,
    STATUS_GPS_UNHEALTHY       = 1U << 12,
    STATUS_MAGNETIC_UNHEALTHY  = 1U << 13,
    STATUS_ENCODER_UNHEALTHY   = 1U << 14,
    STATUS_GPS_WARPED          = 1U << 15
};

enum ActiveSensorFlag : uint32_t {
    ACTIVE_NONE          = 0,
    ACTIVE_GYRO_BOARD    = 1UL << 0,
    ACTIVE_GYRO_CAN      = 1UL << 1,
    ACTIVE_ACCEL_BOARD   = 1UL << 2,
    ACTIVE_ACCEL_CAN     = 1UL << 3,
    ACTIVE_MAG_BOARD     = 1UL << 4,
    ACTIVE_MAG_CAN       = 1UL << 5,
    ACTIVE_ENCODER_CAN   = 1UL << 6,
    ACTIVE_GNSS          = 1UL << 7,
    ACTIVE_ZUPT          = 1UL << 8,
    ACTIVE_ZARU          = 1UL << 9
};

enum FaultFlag : uint32_t {
    FAULT_NONE                  = 0,
    FAULT_GYRO_TIMEOUT          = 1UL << 0,
    FAULT_ACCEL_TIMEOUT         = 1UL << 1,
    FAULT_MAG_TIMEOUT           = 1UL << 2,
    FAULT_ENCODER_TIMEOUT       = 1UL << 3,
    FAULT_GNSS_TIMEOUT          = 1UL << 4,
    FAULT_GYRO_GAP              = 1UL << 5,
    FAULT_ENCODER_DISCONTINUITY = 1UL << 6,
    FAULT_ENCODER_RANGE         = 1UL << 7,
    FAULT_MAG_DISTURBANCE       = 1UL << 8,
    FAULT_GNSS_OUTLIER          = 1UL << 9,
    FAULT_COVARIANCE            = 1UL << 10
};

enum AnomalyFlag : uint16_t {
    ANOMALY_NONE                  = 0,
    ANOMALY_WHEEL_NOT_MOVING      = 1U << 0,
    ANOMALY_WHEEL_SLIP            = 1U << 1,
    ANOMALY_ONE_SIDE_BLOCKED      = 1U << 2,
    ANOMALY_ENCODER_GYRO_MISMATCH = 1U << 3,
    ANOMALY_GPS_ENCODER_MISMATCH  = 1U << 4,
    ANOMALY_HIGH_TILT             = 1U << 5,
    ANOMALY_MOTION_UNOBSERVABLE   = 1U << 6
};

struct GyroPreintegration {
    uint64_t start_timestamp_us;
    uint64_t end_timestamp_us;
    float integrated_z_rad;
    float latest_z_rad_s;
    Sensor::Source source;
    uint16_t sample_count;
    bool valid;
};

struct EncoderObservation {
    int32_t left_cumulative_mm;
    int32_t right_cumulative_mm;
    uint32_t sequence;
    Sensor::SampleMetadata metadata;
    bool left_valid;
    bool right_valid;
};

struct EncoderVelocityObservation {
    float velocity_m_s;
    float left_velocity_m_s;
    float right_velocity_m_s;
    float delta_distance_m;
    float delta_yaw_rad;
    float yaw_rate_rad_s;
    float yaw_variance_rad2;
    uint64_t interval_us;
    float variance_m2_s2;
    Sensor::SampleMetadata metadata;
    bool valid;
};

struct MagneticHeadingObservation {
    float heading_rad;
    float field_strength_uT;
    float variance_rad2;
    Sensor::SampleMetadata metadata;
    bool valid;
};

struct GpsObservation {
    float east_m;
    float north_m;
    float horizontal_accuracy_m;
    float velocity_east_m_s;
    float velocity_north_m_s;
    float speed_accuracy_m_s;
    uint8_t fix_type;
    uint8_t satellites;
    Sensor::SampleMetadata metadata;
    bool position_valid;
    bool velocity_valid;
};

struct StationaryObservation {
    float mean_gyro_z_rad_s;
    bool stationary;
    uint64_t timestamp_us;
};

struct SensorHealthReport {
    uint64_t last_timestamp_us;
    uint64_t last_received_us;
    float last_innovation;
    float last_mahalanobis;
    uint16_t consecutive_bad;
    uint16_t consecutive_good;
    SensorHealth state;
    bool active;
};

struct LocalizationEstimate {
    float px_m;
    float py_m;
    float theta_rad;
    float velocity_m_s;
    float gyro_bias_rad_s;
    float covariance[STATE_COUNT][STATE_COUNT];
    uint64_t timestamp_us;
    uint32_t active_sensor_flags;
    uint32_t fault_flags;
    uint16_t status_flags;
    Quality quality;
    SensorHealth gps_health;
    SensorHealth encoder_health;
    SensorHealth imu_health;
    SensorHealth magnetic_health;
    float latest_gyro_z_rad_s;
    float magnetic_field_uT;
    float magnetic_mahalanobis;
    MagneticRejectReason magnetic_reject_reason;
    GpsCourseRejectReason gps_course_reject_reason;
    uint16_t anomaly_flags;
    uint64_t anomaly_since_us;
    uint32_t gps_warp_count;
    uint64_t gps_warp_timestamp_us;
    float gps_warp_east_m;
    float gps_warp_north_m;
    bool initialized;
    bool valid;
};

} // namespace Domain::Localization
