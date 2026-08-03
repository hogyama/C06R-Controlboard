#pragma once

#include <stdint.h>

namespace Domain::Fusion {

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
    STATUS_OUTSIDE_FIELD       = 1U << 15
};

struct Output {
    uint32_t timestamp_ms;

    // South-west field origin. +X east, +Y north.
    int32_t x_mm;
    int32_t y_mm;

    // Zero east, counter-clockwise positive, range [0, 2*pi).
    float yaw_rad;

    // Body-forward velocity. Forward positive, reverse negative.
    float forward_velocity_mm_s;

    // Rotation around world +Z. Counter-clockwise positive.
    float yaw_rate_rad_s;

    uint32_t position_std_mm;
    float yaw_std_rad;
    uint16_t status_flags;

    // Navigationは推定値だけでなく、この品質と鮮度を見て縮退制御する。
    Quality quality;
    SensorHealth gps_health;
    SensorHealth encoder_health;
    SensorHealth imu_health;
    SensorHealth magnetic_health;
    uint16_t anomaly_flags;
    uint32_t anomaly_since_ms;
};

struct ImuObservation {
    uint32_t timestamp_ms;

    // Body frame: +X forward, +Y left, +Z up.
    // Accelerometer values are specific force, so a stationary upright sensor
    // reports approximately +1 g on Z.
    float accel_x_g;
    float accel_y_g;
    float accel_z_g;

    // Body frame, right-hand rule.
    float gyro_x_rad_s;
    float gyro_y_rad_s;
    float gyro_z_rad_s;
};

struct MagneticObservation {
    uint32_t timestamp_ms;

    // Calibrated body-frame magnetic field: +X forward, +Y left, +Z up.
    float x_uT;
    float y_uT;
    float z_uT;
    // 停止中でもモーター通電中なら強いYaw補正を避ける。
    bool motor_command_active;
};

struct EncoderObservation {
    uint32_t timestamp_ms;
    int32_t left_mm;
    int32_t right_mm;
};

// GPS observation after Domain geodesy converts latitude/longitude to the
// south-west-origin local east/north frame.
struct GpsUpdate {
    uint32_t timestamp_ms;
    int32_t x_mm;
    int32_t y_mm;
    uint32_t horizontal_accuracy_mm;
    int32_t velocity_north_mm_s;
    int32_t velocity_east_mm_s;
    uint32_t speed_accuracy_mm_s;
    uint8_t fix_type;
    bool fix_ok;
};

} // namespace Domain::Fusion
