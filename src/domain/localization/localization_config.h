#pragma once

#include <stdint.h>

namespace Domain::Localization {

struct Config {
    // Process noise variance rates. Q is rebuilt for every prediction interval.
    float distance_noise_m2_s = 0.04f;
    float angle_noise_rad2_s = 0.0064f;
    float velocity_noise_m2_s3 = 0.25f;
    float gyro_bias_noise_rad2_s3 = 9.0e-6f;

    float initial_position_std_m = 5.0f;
    float initial_yaw_std_rad = 1.5f;
    float initial_velocity_std_m_s = 1.0f;
    float initial_bias_std_rad_s = 0.08f;

    float encoder_velocity_std_m_s = 0.12f;
    float encoder_maximum_speed_m_s = 3.0f;
    float encoder_maximum_left_right_difference_m_s = 2.0f;
    uint64_t encoder_minimum_interval_us = 2000;
    uint64_t encoder_maximum_interval_us = 300000;

    float magnetic_heading_std_rad = 0.30f;
    float magnetic_declination_rad = 0.0f;
    float magnetic_minimum_total_uT = 10.0f;
    float magnetic_maximum_total_uT = 100.0f;
    float magnetic_minimum_horizontal_uT = 5.0f;
    float magnetic_maximum_step_rad = 1.2f;

    float gps_position_noise_floor_m = 0.5f;
    float gps_velocity_noise_floor_m_s = 0.05f;
    float gps_minimum_course_speed_m_s = 0.30f;
    float gps_maximum_speed_accuracy_m_s = 1.0f;
    float gps_initial_maximum_accuracy_m = 5.0f;
    uint8_t gps_initial_minimum_fix_type = 3;
    uint8_t gps_initial_minimum_satellites = 6;

    float mahalanobis_gate_1d = 10.83f;
    float mahalanobis_gate_2d = 13.82f;

    float stationary_encoder_speed_m_s = 0.04f;
    float stationary_gyro_rad_s = 0.04f;
    float stationary_accel_norm_tolerance_m_s2 = 0.8f;
    float stationary_accel_variance_m2_s4 = 0.12f;
    uint64_t stationary_hold_us = 500000;

    uint64_t gyro_stale_us = 50000;
    uint64_t gyro_failed_us = 500000;
    uint64_t accel_stale_us = 150000;
    uint64_t accel_failed_us = 1000000;
    uint64_t encoder_stale_us = 150000;
    uint64_t encoder_failed_us = 500000;
    uint64_t magnetic_stale_us = 300000;
    uint64_t magnetic_failed_us = 2000000;
    uint64_t gps_stale_us = 2500000;
    uint64_t gps_failed_us = 10000000;

    uint16_t health_failure_rejections = 5;
    uint16_t health_recovery_accepts = 10;

    float maximum_position_std_m = 12.0f;
    float maximum_yaw_std_rad = 0.8f;
    float field_size_x_m = 60.0f;
    float field_size_y_m = 60.0f;
};

} // namespace Domain::Localization
