#pragma once

#include <stdint.h>

#include "fusion_types.h"

namespace Domain::Fusion {

struct Config {
    float track_width_mm = 180.0f;
    float gravity_mm_s2 = 9806.65f;

    // Continuous-time IMU noise densities and bias random walks.
    float acceleration_noise_mm_s2 = 1500.0f;
    float gyro_noise_rad_s = 0.08f;
    float gyro_bias_walk_rad_s2 = 0.003f;
    float acceleration_bias_walk_mm_s3 = 100.0f;

    // Gravity-direction correction is enabled only under low dynamics.
    float gravity_direction_std = 0.08f;
    float gravity_norm_tolerance_g = 0.18f;
    float gravity_jerk_limit_g_s = 4.0f;

    float encoder_velocity_std_mm_s = 150.0f;
    float encoder_lateral_velocity_std_mm_s = 80.0f;
    float encoder_yaw_rate_std_rad_s = 0.20f;
    float maximum_encoder_speed_mm_s = 3000.0f;
    uint32_t maximum_encoder_interval_ms = 300;
    uint32_t encoder_imu_alignment_tolerance_ms = 30;

    // 停止中は強く、通常直進中は少し弱く絶対Yawを補正する。
    float magnetic_stationary_yaw_std_rad = 0.20f;
    float magnetic_moving_yaw_std_rad = 0.35f;
    float magnetic_declination_rad = 0.0f; // East-positive declination.
    float magnetic_min_total_uT = 10.0f;
    float magnetic_max_total_uT = 100.0f;
    float magnetic_min_horizontal_uT = 5.0f;
    float magnetic_strength_relative_tolerance = 0.25f;

    // モーター磁界と旋回中の磁気外乱を避け、停止・低速時だけ絶対方位補正する。
    float magnetic_stationary_max_speed_mm_s = 150.0f;
    float magnetic_stationary_max_yaw_rate_rad_s = 0.20f;
    float magnetic_moving_max_speed_mm_s = 800.0f;
    float magnetic_moving_max_yaw_rate_rad_s = 0.70f;

    uint32_t minimum_gps_position_std_mm = 500;
    uint32_t maximum_gps_position_std_mm = 10000;
    uint32_t minimum_gps_speed_std_mm_s = 50;
    uint32_t maximum_gps_speed_std_mm_s = 3000;
    uint32_t minimum_gps_velocity_mm_s = 300;
    float minimum_gps_course_std_rad = 0.05f;
    float maximum_gps_course_std_rad = 0.70f;
    uint32_t maximum_delayed_gps_ms = 750;
    uint32_t maximum_future_measurement_ms = 50;

    // Chi-square gates for 1, 2 and 3 dimensional measurements.
    float nis_soft_1d = 3.84f;
    float nis_hard_1d = 10.83f;
    float nis_soft_2d = 5.99f;
    float nis_hard_2d = 13.82f;
    float nis_soft_3d = 7.81f;
    float nis_hard_3d = 16.27f;

    uint32_t maximum_prediction_step_ms = 20;
    uint32_t maximum_prediction_gap_ms = 2000;

    uint8_t health_failure_rejections = 5;
    uint8_t health_recovery_accepts = 10;

    uint32_t maximum_yaw_aiding_age_ms = 30000;
    uint32_t maximum_imu_age_ms = 150;
    uint32_t maximum_usable_position_std_mm = 12000;
    float maximum_usable_yaw_std_rad = 0.8f;

    int32_t field_size_x_mm = 60000;
    int32_t field_size_y_mm = 60000;

    // センサごとに更新周期が違うため、鮮度と故障の境界を分ける。
    uint32_t imu_stale_ms = 50;
    uint32_t imu_failed_ms = 500;
    uint32_t encoder_stale_ms = 50;
    uint32_t encoder_failed_ms = 500;
    uint32_t magnetic_stale_ms = 300;
    uint32_t magnetic_failed_ms = 2000;
    uint32_t gps_stale_ms = 2500;
    uint32_t gps_failed_ms = 10000;
    uint32_t gps_resnap_distance_mm = 5000;
    uint32_t gps_resnap_stable_radius_mm = 3000;
    uint8_t gps_resnap_required_samples = 3;
    uint32_t gps_resnap_maximum_accuracy_mm = 5000;
};

class Filter {
public:
    explicit Filter(const Config& config = Config{});

    void reset();

    void initialize(
        int32_t x_mm,
        int32_t y_mm,
        float yaw_rad,
        bool yaw_usable,
        uint32_t timestamp_ms,
        uint32_t position_std_mm);

    bool initialized() const;

    void beginCycle();

    bool predict(const ImuObservation& observation);
    bool updateEncoder(const EncoderObservation& observation);
    bool updateGps(const GpsUpdate& observation);
    bool updateMagnetic(const MagneticObservation& observation);

    Output output(uint32_t timestamp_ms) const;

private:
    // Error state:
    // [position EN(2), velocity EN(2), attitude error(3),
    //  gyro bias(3), accelerometer bias(3)]
    static constexpr uint8_t STATE_COUNT = 13;
    static constexpr uint8_t MAX_MEASUREMENT_DIMENSION = 3;

    enum StateIndex : uint8_t {
        POSITION_X = 0,
        POSITION_Y,
        VELOCITY_X,
        VELOCITY_Y,
        ATTITUDE_X,
        ATTITUDE_Y,
        ATTITUDE_Z,
        GYRO_BIAS_X,
        GYRO_BIAS_Y,
        GYRO_BIAS_Z,
        ACCELERATION_BIAS_X,
        ACCELERATION_BIAS_Y,
        ACCELERATION_BIAS_Z
    };

    enum class UpdateResult : uint8_t {
        Rejected,
        Accepted,
        SoftAccepted
    };

    struct HealthTracker {
        uint8_t rejection_count;
        uint8_t acceptance_count;
        bool failed;
    };

    Config config_;

    float position_world_mm_[2];
    float velocity_world_mm_s_[2];
    float quaternion_world_from_body_[4]; // [w, x, y, z]
    float gyro_bias_rad_s_[3];
    float acceleration_bias_mm_s2_[3];
    float covariance_[STATE_COUNT][STATE_COUNT];

    bool initialized_;
    bool yaw_reference_usable_;

    uint32_t last_state_timestamp_ms_;
    uint32_t last_predict_timestamp_ms_;
    uint32_t last_gps_timestamp_ms_;
    uint32_t last_gps_position_used_timestamp_ms_;
    uint32_t last_yaw_aiding_timestamp_ms_;
    uint32_t last_magnetic_timestamp_ms_;
    uint32_t last_encoder_used_timestamp_ms_;

    bool have_previous_encoder_;
    EncoderObservation previous_encoder_;

    float gyro_interval_z_integral_rad_;
    float gyro_interval_duration_s_;
    uint32_t gyro_interval_start_timestamp_ms_;

    float latest_gyro_body_rad_s_[3];

    bool have_previous_acceleration_;
    float previous_acceleration_body_g_[3];
    uint32_t previous_acceleration_timestamp_ms_;

    bool have_magnetic_reference_strength_;
    float magnetic_reference_strength_uT_;

    HealthTracker gps_health_;
    HealthTracker magnetic_health_;
    HealthTracker encoder_health_;

    uint8_t gps_resnap_sample_count_;
    float gps_resnap_anchor_x_mm_;
    float gps_resnap_anchor_y_mm_;
    float gps_resnap_sum_x_mm_;
    float gps_resnap_sum_y_mm_;

    uint16_t cycle_status_flags_;

    static SensorHealth sensorHealthAt(
        uint32_t now_ms,
        uint32_t last_timestamp_ms,
        uint32_t stale_ms,
        uint32_t failed_ms);

    static float normalizePi(float angle);
    static float normalizeTwoPi(float angle);
    static int32_t signedTimeDifference(uint32_t first, uint32_t second);

    static void normalizeQuaternion(float quaternion[4]);
    static void quaternionFromYaw(float yaw_rad, float quaternion[4]);
    static void deltaQuaternion(
        const float rotation_vector_rad[3],
        float quaternion[4]);
    static void multiplyQuaternion(
        const float left[4],
        const float right[4],
        float result[4]);
    static void rotationMatrix(
        const float quaternion[4],
        float matrix[3][3]);
    static void rotateBodyToWorld(
        const float quaternion[4],
        const float body[3],
        float world[3]);
    static void rotateWorldToBody(
        const float quaternion[4],
        const float world[3],
        float body[3]);
    static float yawFromQuaternion(const float quaternion[4]);

    void propagateStep(
        const float acceleration_body_mm_s2[3],
        const float gyro_body_rad_s[3],
        float dt_s);
    void updateGravityDirection(
        const ImuObservation& observation,
        const float acceleration_body_mm_s2[3]);

    UpdateResult measurementUpdate(
        const float residual[MAX_MEASUREMENT_DIMENSION],
        const float observation_jacobian
            [MAX_MEASUREMENT_DIMENSION][STATE_COUNT],
        const float measurement_variance[MAX_MEASUREMENT_DIMENSION],
        uint8_t measurement_dimension,
        float soft_gate,
        float hard_gate);

    void injectError(const float error_state[STATE_COUNT]);
    void applyAttitudeResetJacobian(const float attitude_error[3]);
    void stabilizeCovariance();
    void inflateCovariance(float position_factor, float attitude_factor);

    static void noteAccepted(
        HealthTracker& tracker,
        const Config& config);
    static void noteRejected(
        HealthTracker& tracker,
        const Config& config);

    bool positionUsableAt(uint32_t timestamp_ms) const;
    bool yawUsableAt(uint32_t timestamp_ms) const;
    float forwardVelocity() const;
    float yawRate() const;
    float yawVariance() const;
};

} // namespace Domain::Fusion
