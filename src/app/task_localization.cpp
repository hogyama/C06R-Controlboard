#include "tasks.h"

#include "app_queue.h"
#include "app_types.h"
#include "domain/geodesy/gps_to_xy.h"
#include "domain/localization/localization.h"
#include "domain/localization/magnetic_calibrator.h"
#include "domain/sensor/sensor_freshness.h"
#include "platform/board_config.h"
#include "platform/field_config.h"
#include "platform/sensor_axis_transform.h"

#include <Preferences.h>
#include <esp_timer.h>
#include <math.h>

namespace {

constexpr uint64_t LOCALIZATION_PERIOD_US = 10000;
constexpr uint64_t GYRO_BIAS_CALIBRATION_US = 1000000;
constexpr uint32_t GYRO_BIAS_MINIMUM_SAMPLES = 32;
constexpr char CALIBRATION_NAMESPACE[] = "localization";

struct RateCounter {
    uint32_t count = 0;
    float rate_hz = 0.0f;
};

struct BiasCalibration {
    double sum = 0.0;
    double sum_square = 0.0;
    float minimum = 0.0f;
    float maximum = 0.0f;
    uint64_t started_us = 0;
    uint32_t count = 0;
    bool active = false;

    void start(uint64_t now_us)
    {
        sum = 0.0;
        sum_square = 0.0;
        minimum = 0.0f;
        maximum = 0.0f;
        started_us = now_us;
        count = 0;
        active = true;
    }

    void add(float value)
    {
        if (!active || !isfinite(value)) return;
        if (count == 0U) minimum = maximum = value;
        if (value < minimum) minimum = value;
        if (value > maximum) maximum = value;
        sum += value;
        sum_square += static_cast<double>(value) * value;
        if (count != UINT32_MAX) ++count;
    }
};

bool fresh(const Sensor::SampleMetadata& metadata, uint64_t now_us, uint64_t age_us)
{
    return Sensor::sampleIsFresh(metadata, now_us, age_us);
}

Domain::Localization::SensorHealth debugHealth(
    const Sensor::SampleMetadata& metadata,
    uint64_t now_us,
    uint64_t stale_us,
    uint64_t failed_us)
{
    if (!metadata.valid || metadata.received_us == 0U ||
        now_us < metadata.received_us ||
        now_us - metadata.received_us > failed_us) {
        return Domain::Localization::SensorHealth::Failed;
    }
    if (now_us - metadata.received_us > stale_us) {
        return Domain::Localization::SensorHealth::Stale;
    }
    return Domain::Localization::SensorHealth::Fresh;
}

Sensor::GyroscopeData gyroToBody(const Sensor::GyroscopeData& sensor)
{
    const auto body = SensorAxisTransform::imuToBody(
        sensor.metadata.source,
        sensor.x_rad_s, sensor.y_rad_s, sensor.z_rad_s);
    Sensor::GyroscopeData result{};
    result.x_rad_s = body.x;
    result.y_rad_s = body.y;
    result.z_rad_s = body.z;
    result.metadata = sensor.metadata;
    return result;
}

Sensor::AccelerometerData accelerationToBody(
    const Sensor::AccelerometerData& sensor)
{
    const auto body = SensorAxisTransform::imuToBody(
        sensor.metadata.source,
        sensor.x_m_s2, sensor.y_m_s2, sensor.z_m_s2);
    Sensor::AccelerometerData result{};
    result.x_m_s2 = body.x;
    result.y_m_s2 = body.y;
    result.z_m_s2 = body.z;
    result.metadata = sensor.metadata;
    return result;
}

Sensor::MagneticData magneticToBody(const Sensor::MagneticData& sensor)
{
    const auto body = SensorAxisTransform::magneticToBody(
        sensor.metadata.source,
        sensor.x_uT, sensor.y_uT, sensor.z_uT);
    Sensor::MagneticData result{};
    result.x_uT = body.x;
    result.y_uT = body.y;
    result.z_uT = body.z;
    result.metadata = sensor.metadata;
    return result;
}

bool motorCommandActive(uint32_t now_ms)
{
    JogData jog{};
    return xQueuePeek(mbx_can_jog_cmd, &jog, 0) == pdTRUE &&
        static_cast<uint32_t>(now_ms - jog.timestamp_ms) < jog.duration_ms &&
        (fabsf(jog.velocity_mm_s) > 1.0f || fabsf(jog.omega_rad_s) > 0.01f);
}

Coordinate coordinateFromEstimate(
    const Domain::Localization::LocalizationEstimate& estimate,
    const Domain::Localization::GpsObservation& gps,
    const Can::Data::Encoder& encoder)
{
    using namespace Domain::Localization;
    Coordinate coordinate{};
    coordinate.x_mm = static_cast<int32_t>(lroundf(estimate.px_m * 1000.0f));
    coordinate.y_mm = static_cast<int32_t>(lroundf(estimate.py_m * 1000.0f));
    coordinate.heading_rad = estimate.theta_rad;
    coordinate.timestamp_ms = static_cast<uint32_t>(estimate.timestamp_us / 1000ULL);
    coordinate.forward_velocity_mm_s = estimate.velocity_m_s * 1000.0f;
    coordinate.yaw_rate_rad_s =
        estimate.latest_gyro_z_rad_s - estimate.gyro_bias_rad_s;
    coordinate.position_std_mm = static_cast<uint32_t>(lroundf(
        sqrtf(fmaxf(
            estimate.covariance[POSITION_X][POSITION_X],
            estimate.covariance[POSITION_Y][POSITION_Y])) * 1000.0f));
    coordinate.yaw_std_rad = sqrtf(estimate.covariance[YAW][YAW]);
    coordinate.localization_status_flags = estimate.status_flags;
    coordinate.localization_quality = estimate.quality;
    coordinate.gps_health = estimate.gps_health;
    coordinate.encoder_health = estimate.encoder_health;
    coordinate.imu_health = estimate.imu_health;
    coordinate.magnetic_health = estimate.magnetic_health;
    coordinate.magnetic_reject_reason = estimate.magnetic_reject_reason;
    coordinate.magnetic_total_uT = estimate.magnetic_field_uT;
    coordinate.magnetic_nis = estimate.magnetic_mahalanobis;
    coordinate.gps_course_reject_reason = estimate.gps_course_reject_reason;
    coordinate.motion_anomaly_flags = estimate.anomaly_flags;
    coordinate.motion_anomaly_since_ms =
        static_cast<uint32_t>(estimate.anomaly_since_us / 1000ULL);
    coordinate.gps_warp_count = estimate.gps_warp_count;
    coordinate.gps_warp_timestamp_ms = static_cast<uint32_t>(
        estimate.gps_warp_timestamp_us / 1000ULL);
    coordinate.gps_warp_east_mm = static_cast<int32_t>(lroundf(
        estimate.gps_warp_east_m * 1000.0f));
    coordinate.gps_warp_north_mm = static_cast<int32_t>(lroundf(
        estimate.gps_warp_north_m * 1000.0f));
    coordinate.is_first_gps_valid = estimate.initialized;
    if (gps.position_valid) {
        coordinate.gps_x_mm = static_cast<int32_t>(lroundf(gps.east_m * 1000.0f));
        coordinate.gps_y_mm = static_cast<int32_t>(lroundf(gps.north_m * 1000.0f));
    }
    coordinate.encoder_left_mm = encoder.left_mm;
    coordinate.encoder_right_mm = encoder.right_mm;
    if ((estimate.status_flags & STATUS_GPS_USED) != 0U) {
        coordinate.source_flags |= CORD_SRC_GPS;
    }
    if ((estimate.status_flags & STATUS_ENCODER_USED) != 0U) {
        coordinate.source_flags |= CORD_SRC_ENCODER;
    }
    if ((estimate.status_flags & STATUS_YAW_USABLE) != 0U) {
        coordinate.source_flags |= CORD_SRC_HEADING;
    }
    if ((estimate.status_flags &
         (STATUS_GPS_USED | STATUS_ENCODER_USED | STATUS_IMU_USED |
          STATUS_MAGNETIC_USED)) == 0U) {
        coordinate.source_flags |= CORD_SRC_HOLD;
    }
    return coordinate;
}

} // namespace

void taskLocalization(void* pvParameters)
{
    (void)pvParameters;
    using namespace Domain::Localization;

    Config config{};
    config.magnetic_declination_rad = FieldConfig::MAGNETIC_DECLINATION_RAD;
    config.field_size_x_m = FieldConfig::SIZE_X_MM * 0.001f;
    config.field_size_y_m = FieldConfig::SIZE_Y_MM * 0.001f;
    // Localization owns a 128-step delayed-observation history. Keep it in
    // static storage rather than on the FreeRTOS task stack.
    static Localization localization(config);
    static Preprocessor preprocessor(config);
    static GyroIntervalIntegrator gyro_integrator;
    static MagneticCalibrator magnetic_calibrator;
    const Domain::Geodesy::GpsToXY gps_to_xy(
        FieldConfig::GOAL_LATITUDE_E7,
        FieldConfig::GOAL_LONGITUDE_E7,
        FieldConfig::GOAL_X_MM,
        FieldConfig::GOAL_Y_MM);

    Preferences preferences;
    preferences.begin(CALIBRATION_NAMESPACE, false);
    float configured_gyro_bias = preferences.getFloat("gyro_z", 0.0f);
    MagneticCalibration magnetic_calibration{};
    magnetic_calibration.valid = preferences.getBool("mag_valid", false);
    for (uint8_t i = 0; i < 3; ++i) {
        char key[8]{};
        snprintf(key, sizeof(key), "mag_h%u", i);
        magnetic_calibration.hard_iron_uT[i] = preferences.getFloat(key, 0.0f);
        for (uint8_t j = 0; j < 3; ++j) {
            snprintf(key, sizeof(key), "mag_s%u%u", i, j);
            magnetic_calibration.soft_iron[i][j] = preferences.getFloat(
                key, i == j ? 1.0f : 0.0f);
        }
    }
    preprocessor.setMagneticCalibration(magnetic_calibration);

    BiasCalibration bias_calibration{};
    LocalizationDebugStatus debug_status{};
    uint32_t debug_generation = 0;
    uint32_t magnetic_calibration_generation = 0;
    uint32_t magnetic_reset_generation = 0;
    uint16_t observed_navigation_reset_count = 0;
    bool sequence_reset_applied = false;
    uint64_t last_acceleration_timestamp_us = 0;
    uint64_t last_magnetic_timestamp_us = 0;
    uint64_t last_gps_timestamp_us = 0;
    uint64_t processed_gps_timestamp_us = 0;
    Sensor::Source active_gyro_source = Sensor::Source::None;
    uint8_t board_gyro_recovery_count = 0;
    Sensor::AccelerometerData latest_acceleration{};
    Sensor::MagneticData latest_magnetic{};
    Can::Data::Encoder latest_encoder{};
    GpsObservation latest_gps{};
    RateCounter gyro_board_rate{}, gyro_can_rate{};
    RateCounter acceleration_board_rate{}, acceleration_can_rate{};
    RateCounter magnetic_board_rate{}, magnetic_can_rate{};
    RateCounter encoder_rate{}, gps_rate{}, pressure_rate{};
    uint64_t rate_window_started_us = static_cast<uint64_t>(esp_timer_get_time());
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
        const uint32_t now_ms = millis();

        SystemData system{};
        if (xQueuePeek(mbx_system_data, &system, 0) != pdTRUE) continue;
        const bool sequence_reset =
            system.boot_mode != BootMode::DEBUG &&
            (system.state == SystemState::STATE_PRELAUNCH ||
             system.state == SystemState::STATE_AWAIT_ASCENT);
        const bool navigation_reset = system.navigation_reset_count !=
            observed_navigation_reset_count;
        if ((sequence_reset && !sequence_reset_applied) || navigation_reset) {
            observed_navigation_reset_count = system.navigation_reset_count;
            if (sequence_reset || !localization.recoverFromLastGood()) {
                localization.reset();
            }
            preprocessor.reset();
            preprocessor.setMagneticCalibration(magnetic_calibration);
            gyro_integrator.reset(now_us);
            xQueueReset(fifo_board_gyroscope);
            xQueueReset(fifo_can_gyroscope);
            xQueueReset(fifo_can_encoder);
            last_gps_timestamp_us = 0;
            processed_gps_timestamp_us = 0;
        }
        sequence_reset_applied = sequence_reset;

        LocalizationDebugCommand debug_command{};
        while (xQueueReceive(
                   fifo_localization_debug_command,
                   &debug_command,
                   0) == pdTRUE) {
            if (debug_command == LocalizationDebugCommand::CalibrateGyroBias) {
                bias_calibration.start(now_us);
                debug_status.gyro_bias_calibrating = true;
                debug_status.gyro_bias_last_success = false;
            } else if (debug_command ==
                LocalizationDebugCommand::CalibrateMagnetic) {
                magnetic_calibrator.start();
                debug_status.magnetic_calibrating = true;
                debug_status.magnetic_calibration_last_success = false;
            } else if (
                debug_command ==
                LocalizationDebugCommand::ResetMagneticCalibration) {
                magnetic_calibrator.cancel();
                magnetic_calibration = {};
                preprocessor.setMagneticCalibration(magnetic_calibration);
                preferences.remove("mag_valid");
                for (uint8_t i = 0; i < 3; ++i) {
                    char key[8]{};
                    snprintf(key, sizeof(key), "mag_h%u", i);
                    preferences.remove(key);
                    for (uint8_t j = 0; j < 3; ++j) {
                        snprintf(key, sizeof(key), "mag_s%u%u", i, j);
                        preferences.remove(key);
                    }
                }
                ++magnetic_reset_generation;
            }
        }

        Sensor::GyroscopeData board_latest{};
        Sensor::GyroscopeData can_latest{};
        const bool board_gyro_fresh =
            xQueuePeek(mbx_board_gyroscope, &board_latest, 0) == pdTRUE &&
            fresh(board_latest.metadata, now_us, config.gyro_stale_us);
        const bool can_gyro_fresh =
            xQueuePeek(mbx_can_gyroscope, &can_latest, 0) == pdTRUE &&
            fresh(can_latest.metadata, now_us, config.gyro_stale_us);
        Sensor::Source selected_gyro_source = active_gyro_source;
        if (active_gyro_source == Sensor::Source::BoardI2c) {
            if (!board_gyro_fresh) {
                selected_gyro_source = can_gyro_fresh
                    ? Sensor::Source::Can : Sensor::Source::None;
                board_gyro_recovery_count = 0;
            }
        } else if (active_gyro_source == Sensor::Source::Can) {
            if (board_gyro_fresh) {
                if (board_gyro_recovery_count < 10U) {
                    ++board_gyro_recovery_count;
                }
                if (!can_gyro_fresh || board_gyro_recovery_count >= 10U) {
                    selected_gyro_source = Sensor::Source::BoardI2c;
                    board_gyro_recovery_count = 0;
                }
            } else {
                board_gyro_recovery_count = 0;
            }
            if (!can_gyro_fresh && !board_gyro_fresh) {
                selected_gyro_source = Sensor::Source::None;
            }
        } else {
            selected_gyro_source = board_gyro_fresh
                ? Sensor::Source::BoardI2c
                : (can_gyro_fresh
                    ? Sensor::Source::Can : Sensor::Source::None);
        }
        if (selected_gyro_source != active_gyro_source) {
            active_gyro_source = selected_gyro_source;
            gyro_integrator.reset(now_us);
        }

        Sensor::GyroscopeData gyro_sample{};
        while (xQueueReceive(fifo_board_gyroscope, &gyro_sample, 0) == pdTRUE) {
            ++gyro_board_rate.count;
            if (active_gyro_source == Sensor::Source::BoardI2c) {
                const Sensor::GyroscopeData body = gyroToBody(gyro_sample);
                gyro_integrator.push(body);
                bias_calibration.add(body.z_rad_s);
            }
        }
        while (xQueueReceive(fifo_can_gyroscope, &gyro_sample, 0) == pdTRUE) {
            ++gyro_can_rate.count;
            if (active_gyro_source == Sensor::Source::Can) {
                const Sensor::GyroscopeData body = gyroToBody(gyro_sample);
                gyro_integrator.push(body);
                bias_calibration.add(body.z_rad_s);
            }
        }

        if (bias_calibration.active &&
            now_us - bias_calibration.started_us >=
                GYRO_BIAS_CALIBRATION_US) {
            bias_calibration.active = false;
            const bool success =
                bias_calibration.count >= GYRO_BIAS_MINIMUM_SAMPLES;
            debug_status.gyro_bias_calibrating = false;
            debug_status.gyro_bias_last_success = success;
            debug_status.gyro_bias_samples = bias_calibration.count;
            if (success) {
                const double mean = bias_calibration.sum /
                    static_cast<double>(bias_calibration.count);
                const double variance = fmax(
                    bias_calibration.sum_square /
                        static_cast<double>(bias_calibration.count) -
                        mean * mean,
                    0.0);
                configured_gyro_bias = static_cast<float>(mean);
                debug_status.gyro_bias_z_rad_s = configured_gyro_bias;
                debug_status.gyro_bias_std_rad_s =
                    static_cast<float>(sqrt(variance));
                debug_status.gyro_bias_max_deviation_rad_s = fmaxf(
                    fabsf(bias_calibration.minimum - configured_gyro_bias),
                    fabsf(bias_calibration.maximum - configured_gyro_bias));
                preferences.putFloat("gyro_z", configured_gyro_bias);
                localization.setGyroBias(
                    configured_gyro_bias, static_cast<float>(variance));
            }
            debug_status.gyro_bias_generation = ++debug_generation;
        }

        Sensor::AccelerometerData selected_acceleration{};
        if (xQueuePeek(mbx_acceleration, &selected_acceleration, 0) == pdTRUE &&
            selected_acceleration.metadata.timestamp_us !=
                last_acceleration_timestamp_us) {
            last_acceleration_timestamp_us =
                selected_acceleration.metadata.timestamp_us;
            latest_acceleration = accelerationToBody(selected_acceleration);
            preprocessor.updateAcceleration(latest_acceleration);
            localization.noteAcceleration(latest_acceleration);
        }
        Sensor::AccelerometerData physical_acceleration{};
        static uint64_t last_board_accel_us = 0, last_can_accel_us = 0;
        if (xQueuePeek(mbx_board_acceleration, &physical_acceleration, 0) == pdTRUE &&
            physical_acceleration.metadata.timestamp_us != last_board_accel_us) {
            last_board_accel_us = physical_acceleration.metadata.timestamp_us;
            ++acceleration_board_rate.count;
        }
        if (xQueuePeek(mbx_can_acceleration, &physical_acceleration, 0) == pdTRUE &&
            physical_acceleration.metadata.timestamp_us != last_can_accel_us) {
            last_can_accel_us = physical_acceleration.metadata.timestamp_us;
            ++acceleration_can_rate.count;
        }

        CycleInput cycle{};
        cycle.timestamp_us = now_us;
        cycle.gyroscope = gyro_integrator.finish(now_us);
        cycle.motor_command_active = motorCommandActive(now_ms);
        cycle.stationary = preprocessor.stationaryObservation(
            cycle.gyroscope.latest_z_rad_s,
            cycle.motor_command_active,
            now_us);

        Can::Data::Encoder raw_encoder{};
        while (xQueueReceive(fifo_can_encoder, &raw_encoder, 0) == pdTRUE) {
            latest_encoder = raw_encoder;
            ++encoder_rate.count;
            EncoderObservation observation{};
            observation.left_cumulative_mm = raw_encoder.left_mm;
            observation.right_cumulative_mm = raw_encoder.right_mm;
            observation.sequence = raw_encoder.sequence;
            observation.metadata = raw_encoder.metadata;
            observation.left_valid = raw_encoder.left_valid;
            observation.right_valid = raw_encoder.right_valid;
            cycle.encoder = preprocessor.processEncoder(observation);
            cycle.has_encoder = true;
        }

        Sensor::MagneticData selected_magnetic{};
        if (xQueuePeek(mbx_magnetic, &selected_magnetic, 0) == pdTRUE &&
            selected_magnetic.metadata.timestamp_us != last_magnetic_timestamp_us) {
            last_magnetic_timestamp_us = selected_magnetic.metadata.timestamp_us;
            latest_magnetic = magneticToBody(selected_magnetic);
            cycle.magnetic = preprocessor.processMagnetic(latest_magnetic);
            cycle.has_magnetic = true;
        }
        Sensor::MagneticData physical_magnetic{};
        static uint64_t last_board_mag_us = 0, last_can_mag_us = 0;
        if (xQueuePeek(mbx_board_magnetic, &physical_magnetic, 0) == pdTRUE &&
            physical_magnetic.metadata.timestamp_us != last_board_mag_us) {
            last_board_mag_us = physical_magnetic.metadata.timestamp_us;
            ++magnetic_board_rate.count;
            magnetic_calibrator.add(magneticToBody(physical_magnetic));
        }
        if (xQueuePeek(mbx_can_magnetic, &physical_magnetic, 0) == pdTRUE &&
            physical_magnetic.metadata.timestamp_us != last_can_mag_us) {
            last_can_mag_us = physical_magnetic.metadata.timestamp_us;
            ++magnetic_can_rate.count;
        }
        if (!magnetic_calibrator.active() &&
            magnetic_calibrator.result() ==
                MagneticCalibrationResult::Collecting) {
            const bool success =
                magnetic_calibrator.finish(magnetic_calibration);
            if (magnetic_calibrator.active()) {
                // Preliminary coverage can still be one-sided around the
                // fitted hard-iron center. Keep collecting without reporting
                // a failed calibration or requiring another start command.
                debug_status.magnetic_calibrating = true;
            } else {
                debug_status.magnetic_calibration_last_success = success;
                debug_status.magnetic_calibrating = false;
                ++magnetic_calibration_generation;
                if (success) {
                    preprocessor.setMagneticCalibration(magnetic_calibration);
                    for (uint8_t i = 0; i < 3; ++i) {
                        char key[8]{};
                        snprintf(key, sizeof(key), "mag_h%u", i);
                        preferences.putFloat(
                            key, magnetic_calibration.hard_iron_uT[i]);
                        for (uint8_t j = 0; j < 3; ++j) {
                            snprintf(key, sizeof(key), "mag_s%u%u", i, j);
                            preferences.putFloat(
                                key, magnetic_calibration.soft_iron[i][j]);
                        }
                    }
                    preferences.putBool("mag_valid", true);
                }
            }
        }

        Gps::NavPvtObservation nav_pvt{};
        if (xQueuePeek(mbx_gps_nav_pvt_observation, &nav_pvt, 0) == pdTRUE &&
            nav_pvt.metadata.timestamp_us != 0U &&
            nav_pvt.metadata.timestamp_us != last_gps_timestamp_us) {
            last_gps_timestamp_us = nav_pvt.metadata.timestamp_us;
            int32_t x_mm = 0, y_mm = 0;
            if (gps_to_xy.convert(
                    nav_pvt.latitude_e7,
                    nav_pvt.longitude_e7,
                    x_mm,
                    y_mm)) {
                latest_gps = {};
                latest_gps.east_m = x_mm * 0.001f;
                latest_gps.north_m = y_mm * 0.001f;
                latest_gps.horizontal_accuracy_m =
                    nav_pvt.horizontal_accuracy_mm * 0.001f;
                latest_gps.velocity_east_m_s =
                    nav_pvt.velocity_east_mm_s * 0.001f;
                latest_gps.velocity_north_m_s =
                    nav_pvt.velocity_north_mm_s * 0.001f;
                latest_gps.speed_accuracy_m_s =
                    nav_pvt.speed_accuracy_mm_s * 0.001f;
                latest_gps.fix_type = nav_pvt.fix_type;
                latest_gps.satellites = nav_pvt.satellites;
                latest_gps.metadata = nav_pvt.metadata;
                latest_gps.position_valid = nav_pvt.metadata.valid &&
                    nav_pvt.fix_ok && nav_pvt.fix_type >= 3U;
                latest_gps.velocity_valid = latest_gps.position_valid &&
                    isfinite(latest_gps.velocity_east_m_s) &&
                    isfinite(latest_gps.velocity_north_m_s);
                xQueueOverwrite(mbx_gps_local_observation, &latest_gps);
                ++gps_rate.count;
            }
        }

        if (system.boot_mode != BootMode::DEBUG && !sequence_reset) {
            const bool gps_initialization_usable =
                latest_gps.position_valid &&
                latest_gps.fix_type >= config.gps_initial_minimum_fix_type &&
                latest_gps.satellites >=
                    config.gps_initial_minimum_satellites &&
                latest_gps.horizontal_accuracy_m <=
                    config.gps_initial_maximum_accuracy_m;
            if (!localization.ekf().initialized() &&
                gps_initialization_usable) {
                const MagneticHeadingObservation* initial_heading =
                    cycle.has_magnetic && cycle.magnetic.valid
                        ? &cycle.magnetic : nullptr;
                localization.initializeFromGps(
                    latest_gps, initial_heading, configured_gyro_bias);
                processed_gps_timestamp_us = latest_gps.metadata.timestamp_us;
                gyro_integrator.reset(now_us);
            }
            if (localization.ekf().initialized()) {
                localization.processCycle(cycle);
                if (system.gps_localization_enabled && latest_gps.position_valid &&
                    latest_gps.metadata.timestamp_us != 0U) {
                    if (latest_gps.metadata.timestamp_us !=
                        processed_gps_timestamp_us) {
                        localization.processGps(latest_gps);
                        processed_gps_timestamp_us =
                            latest_gps.metadata.timestamp_us;
                    }
                }
                LocalizationEstimate estimate = localization.estimate(now_us);
                xQueueOverwrite(mbx_localization_estimate, &estimate);
                const Coordinate coordinate = coordinateFromEstimate(
                    estimate, latest_gps, latest_encoder);
                xQueueOverwrite(mbx_coordinate, &coordinate);
            }
        }

        if (now_us - rate_window_started_us >= 1000000U) {
            const float scale = 1000000.0f /
                static_cast<float>(now_us - rate_window_started_us);
            RateCounter* rates[] = {
                &gyro_board_rate, &gyro_can_rate,
                &acceleration_board_rate, &acceleration_can_rate,
                &magnetic_board_rate, &magnetic_can_rate,
                &encoder_rate, &gps_rate, &pressure_rate};
            for (RateCounter* rate : rates) {
                rate->rate_hz = rate->count * scale;
                rate->count = 0;
            }
            rate_window_started_us = now_us;
        }

        Sensor::AccelerometerData board_accel{}, can_accel{};
        Sensor::MagneticData board_mag{}, can_mag{};
        Sensor::PressureData pressure{};
        xQueuePeek(mbx_board_acceleration, &board_accel, 0);
        xQueuePeek(mbx_can_acceleration, &can_accel, 0);
        xQueuePeek(mbx_board_magnetic, &board_mag, 0);
        xQueuePeek(mbx_can_magnetic, &can_mag, 0);
        xQueuePeek(mbx_pressure, &pressure, 0);
        static uint64_t last_pressure_us = 0;
        if (pressure.metadata.timestamp_us != 0U &&
            pressure.metadata.timestamp_us != last_pressure_us) {
            last_pressure_us = pressure.metadata.timestamp_us;
            ++pressure_rate.count;
        }
        debug_status.gyroscope = {
            debugHealth(board_latest.metadata, now_us,
                config.gyro_stale_us, config.gyro_failed_us),
            debugHealth(can_latest.metadata, now_us,
                config.gyro_stale_us, config.gyro_failed_us),
            active_gyro_source,
            gyro_board_rate.rate_hz,
            gyro_can_rate.rate_hz};
        debug_status.accelerometer = {
            debugHealth(board_accel.metadata, now_us,
                config.accel_stale_us, config.accel_failed_us),
            debugHealth(can_accel.metadata, now_us,
                config.accel_stale_us, config.accel_failed_us),
            latest_acceleration.metadata.source,
            acceleration_board_rate.rate_hz,
            acceleration_can_rate.rate_hz};
        debug_status.magnetic = {
            debugHealth(board_mag.metadata, now_us,
                config.magnetic_stale_us, config.magnetic_failed_us),
            debugHealth(can_mag.metadata, now_us,
                config.magnetic_stale_us, config.magnetic_failed_us),
            latest_magnetic.metadata.source,
            magnetic_board_rate.rate_hz,
            magnetic_can_rate.rate_hz};
        debug_status.magnetic_diagnostic_acceleration_source =
            latest_acceleration.metadata.source;
        debug_status.board_corrected_magnetic_valid = false;
        debug_status.board_magnetic_yaw_valid = false;
        debug_status.can_magnetic_yaw_valid = false;
        if (Sensor::sampleIsFresh(
                board_mag.metadata, now_us, config.magnetic_stale_us)) {
            const MagneticDiagnostic diagnostic =
                preprocessor.magneticDiagnostic(magneticToBody(board_mag));
            for (uint8_t axis = 0; axis < 3U; ++axis) {
                debug_status.board_corrected_magnetic_uT[axis] =
                    diagnostic.corrected_uT[axis];
            }
            debug_status.board_corrected_magnetic_valid =
                diagnostic.vector_valid && diagnostic.calibration_applied;
            debug_status.board_magnetic_yaw_rad = diagnostic.heading_rad;
            debug_status.board_magnetic_yaw_valid =
                diagnostic.heading_valid && diagnostic.calibration_applied;
        }
        if (Sensor::sampleIsFresh(
                can_mag.metadata, now_us, config.magnetic_stale_us)) {
            const MagneticDiagnostic diagnostic =
                preprocessor.magneticDiagnostic(magneticToBody(can_mag));
            debug_status.can_magnetic_yaw_rad = diagnostic.heading_rad;
            debug_status.can_magnetic_yaw_valid = diagnostic.heading_valid;
        }
        debug_status.encoder_health = debugHealth(
            latest_encoder.metadata, now_us,
            config.encoder_stale_us, config.encoder_failed_us);
        debug_status.gps_health = debugHealth(
            latest_gps.metadata, now_us,
            config.gps_stale_us, config.gps_failed_us);
        debug_status.pressure_health = debugHealth(
            pressure.metadata, now_us, 300000U, 2000000U);
        debug_status.encoder_rate_hz = encoder_rate.rate_hz;
        debug_status.gps_rate_hz = gps_rate.rate_hz;
        debug_status.pressure_rate_hz = pressure_rate.rate_hz;
        debug_status.gyro_bias_z_rad_s = configured_gyro_bias;
        debug_status.gyro_bias_calibrating = bias_calibration.active;
        debug_status.magnetic_calibration_valid = magnetic_calibration.valid;
        debug_status.magnetic_calibrating = magnetic_calibrator.active();
        debug_status.magnetic_calibration_samples =
            magnetic_calibrator.sampleCount();
        debug_status.magnetic_calibration_target_samples =
            MagneticCalibrator::MINIMUM_SAMPLE_COUNT;
        debug_status.magnetic_calibration_direction_bins =
            magnetic_calibrator.directionBins();
        debug_status.magnetic_calibration_target_direction_bins =
            magnetic_calibrator.directionTargetBins();
        debug_status.magnetic_calibration_current_direction =
            magnetic_calibrator.currentDirectionBin();
        for (uint8_t bin = 0;
             bin < MagneticCalibrator::DIRECTION_BIN_COUNT; ++bin) {
            debug_status.magnetic_calibration_direction_counts[bin] =
                magnetic_calibrator.directionBinCount(bin);
        }
        debug_status.magnetic_calibration_progress_percent =
            magnetic_calibrator.progressPercent();
        debug_status.magnetic_calibration_need =
            static_cast<uint8_t>(magnetic_calibrator.need());
        for (uint8_t axis = 0; axis < 3; ++axis) {
            debug_status.magnetic_calibration_axis_range_uT[axis] =
                magnetic_calibrator.axisRangeUT(axis);
        }
        debug_status.magnetic_calibration_eigenvalue_ratio =
            magnetic_calibrator.eigenvalueRatio();
        debug_status.magnetic_calibration_result =
            static_cast<uint8_t>(magnetic_calibrator.result());
        debug_status.magnetic_calibration_rms_uT =
            magnetic_calibrator.rmsErrorUT();
        debug_status.magnetic_calibration_generation =
            magnetic_calibration_generation;
        for (uint8_t i = 0; i < 3; ++i) {
            debug_status.magnetic_hard_iron_uT[i] =
                magnetic_calibration.hard_iron_uT[i];
            for (uint8_t j = 0; j < 3; ++j) {
                debug_status.magnetic_soft_iron[i][j] =
                    magnetic_calibration.soft_iron[i][j];
            }
        }
        debug_status.magnetic_reset_generation = magnetic_reset_generation;
        debug_status.timestamp_us = now_us;
        xQueueOverwrite(mbx_localization_debug_status, &debug_status);
    }
}
