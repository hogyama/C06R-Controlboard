#pragma once

#include <stdint.h>

#include "motion_types.h"

namespace Domain::Motion {

struct DetectorConfig {
    float half_track_mm = 90.0f;
    float minimum_translation_command_mm_s = 50.0f;
    float minimum_rotation_command_rad_s = 0.20f;
    float minimum_wheel_command_mm_s = 100.0f;
    float stopped_velocity_mm_s = 60.0f;
    float stopped_yaw_rate_rad_s = 0.15f;
    float stopped_ratio = 0.20f;
    float moving_ratio = 0.55f;

    uint32_t command_arm_ms = 700;
    uint32_t command_gap_reset_ms = 800;
    uint16_t score_maximum = 1000;
    uint16_t suspend_score = 700;
    uint16_t rearm_score = 200;
    uint16_t wheel_blocked_rise_per_s = 150;
    uint16_t wheel_slip_rise_per_s = 100;
    uint16_t rotation_blocked_rise_per_s = 150;
    uint16_t body_trapped_rise_per_s = 100;
    uint16_t healthy_decay_per_s = 350;
    uint16_t neutral_decay_per_s = 30;

    uint32_t gps_window_ms = 8000;
    uint8_t gps_minimum_samples = 5;
    uint32_t gps_maximum_accuracy_mm = 2000;
    float gps_stationary_radius_mm = 1000.0f;
    float slip_minimum_encoder_mm = 3000.0f;

    float body_tilt_start_deg = 15.0f;
    float body_tilt_healthy_deg = 10.0f;
    uint32_t body_tilt_arm_ms = 2000;
    float gravity_low_pass_alpha = 0.18f;

};

struct DetectorSample {
    uint32_t timestamp_ms;
    uint32_t dt_ms;
    bool navigation_active;
    bool command_valid;
    float command_velocity_mm_s;
    float command_yaw_rate_rad_s;

    bool encoder_available;
    bool encoder_updated;
    float encoder_left_velocity_mm_s;
    float encoder_right_velocity_mm_s;

    bool gyro_available;
    float gyro_yaw_rate_rad_s;

    bool acceleration_available;
    float acceleration_x_g;
    float acceleration_y_g;
    float acceleration_z_g;

    bool fusion_available;
    bool fusion_position_usable;
    bool fusion_yaw_usable;
    float fusion_forward_velocity_mm_s;
    float fusion_yaw_rate_rad_s;
    int32_t fusion_x_mm;
    int32_t fusion_y_mm;

    bool gps_available;
    bool gps_updated;
    uint32_t gps_horizontal_accuracy_mm;
    int32_t gps_x_mm;
    int32_t gps_y_mm;

    bool path_available;
    uint32_t path_revision;
    uint16_t path_nearest_index;
    float path_distance_to_goal_mm;
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
    DetectorConfig config_;
    uint32_t command_active_ms_;
    uint32_t last_motion_command_ms_;

    uint32_t last_gps_seen_ms_;
    uint32_t gps_window_started_ms_;
    int32_t gps_start_x_mm_;
    int32_t gps_start_y_mm_;
    float gps_max_radius_mm_;
    float gps_encoder_mm_;
    uint8_t gps_sample_count_;
    bool gps_window_active_;

    StuckScores scores_;
    uint32_t body_tilt_active_ms_;
    float gravity_body_g_[3];
    bool have_gravity_;
    float current_tilt_deg_;
    bool suspend_latched_;

    static float absolute(float value);
    void resetGpsWindow();
    void updateScore(
        uint16_t& score,
        bool evidence,
        bool healthy,
        uint16_t rise_per_s,
        uint32_t dt_ms);
    uint16_t scoreForReason(Reason reason) const;
};

} // namespace Domain::Motion
