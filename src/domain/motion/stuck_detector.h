#pragma once

#include <stdint.h>

#include "motion_types.h"

namespace Domain::Motion {

struct DetectorConfig {
    float half_track_mm = 90.0f;
    float minimum_translation_command_mm_s = 50.0f;
    float minimum_rotation_command_rad_s = 0.20f;
    float minimum_wheel_command_mm_s = 100.0f;
    float stopped_velocity_mm_s = 20.0f;
    float moving_velocity_mm_s = 50.0f;
    float stopped_yaw_rate_rad_s = 0.10f;
    float stopped_ratio = 0.20f;
    float moving_ratio = 0.50f;

    uint32_t command_arm_ms = 300;
    uint32_t command_gap_reset_ms = 800;
    uint32_t wheel_blocked_arm_ms = 1500;
    uint32_t direction_mismatch_arm_ms = 500;
    uint32_t rotation_blocked_arm_ms = 1500;
    uint32_t slip_gyro_mismatch_arm_ms = 1000;

    uint32_t gps_window_ms = 6000;
    uint8_t gps_minimum_samples = 6;
    uint32_t gps_maximum_accuracy_mm = 3000;
    uint32_t gps_maximum_speed_accuracy_mm_s = 500;
    float slip_speed_ratio = 0.30f;
    float healthy_speed_ratio = 0.60f;
    float slip_distance_ratio = 0.35f;
    float healthy_distance_ratio = 0.65f;
    float slip_minimum_encoder_mm = 1500.0f;
    uint8_t slip_speed_mismatch_samples = 2;
    uint32_t slip_no_ground_arm_ms = 6000;
    float slip_no_ground_encoder_mm = 1200.0f;
    uint32_t slip_after_impact_arm_ms = 2000;
    float slip_after_impact_encoder_mm = 400.0f;
    float acceleration_impact_delta_g = 0.25f;
    uint32_t acceleration_impact_hold_ms = 3000;
};

struct DetectorSample {
    uint32_t timestamp_ms;
    uint32_t dt_ms;
    bool navigation_active;
    bool command_valid;
    float command_velocity_mm_s;
    float command_yaw_rate_rad_s;

    bool encoder_available;
    int32_t encoder_left_mm;
    int32_t encoder_right_mm;
    float encoder_left_velocity_mm_s;
    float encoder_right_velocity_mm_s;

    bool gyro_available;
    float gyro_yaw_rate_rad_s;

    bool acceleration_available;
    float acceleration_x_g;
    float acceleration_y_g;
    float acceleration_z_g;

    bool gps_available;
    bool gps_updated;
    uint32_t gps_horizontal_accuracy_mm;
    int32_t gps_x_mm;
    int32_t gps_y_mm;
    bool gps_velocity_available;
    float gps_velocity_east_mm_s;
    float gps_velocity_north_mm_s;
    float gps_speed_accuracy_mm_s;
};

class StuckDetector {
public:
    explicit StuckDetector(
        const DetectorConfig& config = DetectorConfig{});

    void reset(uint32_t timestamp_ms = 0);
    Assessment update(const DetectorSample& sample);
    void completeVerification(Reason reason, bool movement_confirmed);
    StuckScores scores() const;
    DetectorDiagnostics diagnostics(uint32_t timestamp_ms) const;

private:
    struct GpsPoint {
        uint32_t timestamp_ms;
        int32_t x_mm;
        int32_t y_mm;
        int32_t encoder_mean_mm;
    };

    static constexpr uint8_t GPS_POINT_CAPACITY = 8;
    DetectorConfig config_;
    uint32_t command_active_ms_;
    uint32_t last_motion_command_ms_;
    uint32_t wheel_blocked_active_ms_;
    uint32_t direction_mismatch_active_ms_;
    uint32_t rotation_blocked_active_ms_;
    uint32_t slip_gyro_mismatch_active_ms_;
    uint32_t slip_no_ground_active_ms_;
    float slip_no_ground_encoder_mm_;
    uint8_t gps_speed_mismatch_count_;
    uint32_t acceleration_impact_until_ms_;
    float previous_acceleration_x_g_;
    bool have_previous_acceleration_;
    GpsPoint gps_points_[GPS_POINT_CAPACITY];
    uint8_t gps_point_count_;
    float gps_displacement_mm_;
    float gps_encoder_distance_mm_;
    StuckScores scores_;
    bool suspend_latched_;

    static float absolute(float value);
    static uint32_t updateTimer(
        uint32_t value, bool evidence, bool healthy, uint32_t dt_ms);
    static uint16_t timerScore(uint32_t value, uint32_t threshold_ms);
    void resetSlipTracking();
    void pushGpsPoint(const DetectorSample& sample);
    bool gpsDistanceMismatch(bool& healthy) const;
};

} // namespace Domain::Motion
