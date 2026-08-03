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
    float moving_ratio = 0.45f;

    uint32_t command_arm_ms = 700;
    uint32_t command_gap_reset_ms = 800;
    uint32_t blocked_confirm_ms = 3000;
    uint32_t single_source_confirm_ms = 6000;
    uint32_t wheel_blocked_confirm_ms = 2500;
    uint32_t rotation_confirm_ms = 2500;
    uint32_t unobservable_confirm_ms = 5000;

    uint32_t gps_window_ms = 10000;
    uint8_t gps_minimum_samples = 5;
    uint32_t gps_maximum_accuracy_mm = 2000;
    float gps_minimum_expected_mm = 3000.0f;
    float gps_stationary_radius_mm = 1000.0f;
    float slip_minimum_encoder_mm = 5000.0f;

    uint32_t path_window_ms = 15000;
    float path_minimum_commanded_mm = 4000.0f;
    float path_minimum_goal_improvement_mm = 700.0f;
    uint16_t path_minimum_index_advance = 1;

    uint32_t oscillation_window_ms = 12000;
    uint8_t oscillation_minimum_reversals = 4;
    float oscillation_minimum_motion_mm = 4000.0f;
    float oscillation_maximum_radius_mm = 2000.0f;
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

private:
    static constexpr uint8_t REASON_COUNT = 13;

    DetectorConfig config_;
    uint32_t candidate_since_ms_[REASON_COUNT];
    uint32_t candidate_active_ms_[REASON_COUNT];
    uint32_t command_active_ms_;
    uint32_t last_motion_command_ms_;

    uint32_t last_gps_seen_ms_;
    uint32_t gps_window_started_ms_;
    int32_t gps_start_x_mm_;
    int32_t gps_start_y_mm_;
    float gps_max_radius_mm_;
    float gps_expected_mm_;
    float gps_encoder_mm_;
    uint8_t gps_sample_count_;
    bool gps_window_active_;

    uint32_t path_window_started_ms_;
    uint32_t path_revision_;
    uint16_t path_start_index_;
    float path_start_goal_distance_mm_;
    float path_commanded_mm_;
    bool path_window_active_;

    uint32_t oscillation_window_started_ms_;
    int32_t oscillation_start_x_mm_;
    int32_t oscillation_start_y_mm_;
    float oscillation_max_radius_mm_;
    float oscillation_motion_mm_;
    uint8_t oscillation_reversals_;
    int8_t previous_translation_sign_;
    int8_t previous_rotation_sign_;
    bool oscillation_window_active_;

    int32_t previous_fusion_x_mm_;
    int32_t previous_fusion_y_mm_;
    bool have_previous_fusion_position_;

    static float absolute(float value);
    static int8_t signWithDeadband(float value, float deadband);
    static uint8_t reasonIndex(Reason reason);

    bool confirmCandidate(
        Reason reason,
        bool suspected,
        uint32_t required_ms,
        uint32_t now_ms,
        uint32_t active_dt_ms);
    void clearCandidates();
    void resetGpsWindow();
    void resetPathWindow();
    void resetOscillationWindow();
};

} // namespace Domain::Motion
