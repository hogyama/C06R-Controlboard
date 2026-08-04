#pragma once

#include <stdint.h>

#include "fusion_types.h"

namespace Domain::Fusion {

struct Config {
    float track_width_mm = 180.0f;

    // ジャイロ予測と前方速度のランダムウォーク。
    float gyro_noise_rad_s = 0.08f;
    float gyro_bias_walk_rad_s2 = 0.003f;
    float velocity_process_noise_mm_s2 = 500.0f;

    // 加速度は位置へ積分せず、地磁気の傾斜補正だけに使う。
    float gravity_norm_tolerance_g = 0.18f;
    float gravity_jerk_limit_g_s = 4.0f;
    float gravity_direction_filter_gain = 0.10f;

    float encoder_yaw_rate_std_rad_s = 0.20f;
    float maximum_encoder_speed_mm_s = 3000.0f;
    uint32_t maximum_encoder_interval_ms = 300;
    uint32_t encoder_imu_alignment_tolerance_ms = 30;

    // 同一区間のジャイロと車輪Yawの差だけを弱く補正する。
    float encoder_yaw_blend = 0.20f;

    // 左右タイヤ移動量の分散: q_abs*|d| + (q_rel*d)^2。
    float wheel_noise_mm2_per_mm = 2.0f;
    float wheel_relative_noise = 0.05f;
    float encoder_slip_noise_multiplier = 4.0f;

    // エンコーダ欠落中に増やす位置標準偏差密度 [mm/sqrt(s)]。
    float missing_encoder_position_noise_mm_sqrt_s = 300.0f;

    // 停止中は強く、通常走行中は少し弱く絶対Yawを補正する。
    float magnetic_stationary_yaw_std_rad = 0.20f;
    float magnetic_moving_yaw_std_rad = 0.35f;
    float magnetic_declination_rad = 0.0f; // 東偏角を正とする。
    float magnetic_min_total_uT = 10.0f;
    float magnetic_max_total_uT = 100.0f;
    float magnetic_min_horizontal_uT = 5.0f;
    float magnetic_strength_relative_tolerance = 0.25f;
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

    // 95%付近をsoft、99.5%付近をhard棄却境界にする。
    float nis_soft_1d = 3.84f;
    float nis_hard_1d = 10.83f;
    float nis_soft_2d = 5.99f;
    float nis_hard_2d = 13.82f;

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
    // 地上走行専用状態: [X, Y, Yaw, 前方速度, Zジャイロバイアス]
    static constexpr uint8_t STATE_COUNT = 5;
    static constexpr uint8_t MAX_MEASUREMENT_DIMENSION = 2;

    enum StateIndex : uint8_t {
        POSITION_X = 0,
        POSITION_Y,
        YAW,
        FORWARD_VELOCITY,
        GYRO_BIAS_Z
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
    float state_[STATE_COUNT];
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
    float latest_gyro_z_rad_s_;

    // EKF状態には含めず、地磁気の傾斜補正にだけ使う重力方向。
    float gravity_body_unit_[3];
    bool have_gravity_direction_;
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

    static float normalizePi(float angle);
    static float normalizeTwoPi(float angle);
    static int32_t signedTimeDifference(uint32_t first, uint32_t second);
    static SensorHealth sensorHealthAt(
        uint32_t now_ms,
        uint32_t last_timestamp_ms,
        uint32_t stale_ms,
        uint32_t failed_ms);

    void propagateStep(float gyro_z_rad_s, float dt_s);
    void updateGravityDirection(const ImuObservation& observation);
    void propagateWheelMotion(
        float delta_left_mm,
        float delta_right_mm,
        float delta_forward_mm,
        float midpoint_yaw_rad,
        float yaw_blend,
        bool slip_likely,
        float dt_s);

    UpdateResult measurementUpdate(
        const float residual[MAX_MEASUREMENT_DIMENSION],
        const float jacobian[MAX_MEASUREMENT_DIMENSION][STATE_COUNT],
        const float variance[MAX_MEASUREMENT_DIMENSION],
        uint8_t dimension,
        float soft_gate,
        float hard_gate);

    void stabilizeCovariance();
    void inflateCovariance(float position_factor, float yaw_factor);
    void resetStateCovariance(uint8_t state_index, float variance);

    static void noteAccepted(HealthTracker& tracker, const Config& config);
    static void noteRejected(HealthTracker& tracker, const Config& config);

    bool positionUsableAt(uint32_t timestamp_ms) const;
    bool yawUsableAt(uint32_t timestamp_ms) const;
    float yawRate() const;
};

} // namespace Domain::Fusion
