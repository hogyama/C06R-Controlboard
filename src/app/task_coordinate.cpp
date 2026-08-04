#include "tasks.h"

#include "app_context.h"
#include "app_queue.h"
#include "app_types.h"
#include "domain/fusion/fusion_filter.h"
#include "domain/geodesy/gps_to_xy.h"
#include "platform/board_config.h"
#include "platform/field_config.h"
#include "platform/sensor_axis_transform.h"

#include <math.h>

namespace {

constexpr uint32_t FUSION_PERIOD_MS = 10;

constexpr uint32_t IMU_TIMEOUT_MS = 100;
constexpr uint32_t IMU_PAIR_MAX_DIFFERENCE_MS = 30;

constexpr uint32_t MAGNETIC_TIMEOUT_MS = 250;
constexpr uint32_t ENCODER_TIMEOUT_MS = 150;

constexpr uint32_t GPS_TIMEOUT_MS = 2000;
constexpr uint8_t GPS_INITIAL_REQUIRED_SAMPLES = 3;
constexpr uint8_t GPS_INITIAL_MIN_SATELLITES = 6;
constexpr uint32_t GPS_INITIAL_MAX_ACCURACY_MM = 5000;
constexpr uint32_t GPS_INITIAL_NMEA_MAX_ACCURACY_MM = 8000;
constexpr float GPS_INITIAL_STABLE_RADIUS_MM = 3000.0f;
constexpr uint32_t GPS_INITIAL_MIN_SAMPLE_INTERVAL_MS = 500;

// NMEAにはhAccがないため、HDOP×3mを位置標準偏差として保守的に扱う。
constexpr uint32_t NMEA_BASE_ACCURACY_MM = 3000;
constexpr uint32_t NMEA_MAX_ACCURACY_MM = 10000;
constexpr uint32_t GPS_MAX_FUSION_ACCURACY_MM = 10000;

constexpr float ANOMALY_COMMAND_SPEED_MM_S = 100.0f;
constexpr float ANOMALY_COMMAND_YAW_RATE_RAD_S = 0.3f;
constexpr float ANOMALY_STOPPED_WHEEL_MM_S = 60.0f;
constexpr float ANOMALY_ENCODER_GYRO_RATE_RAD_S = 1.0f;

// 反転判定は従来実績を維持し、センサ基板Z軸（上正）の加速度だけを使う。
constexpr float ATTITUDE_FLIPPED_THRESHOLD_G = -0.5f;
constexpr float ATTITUDE_TILT_THRESHOLD_G = 0.5f;

// GPS初期姿勢推定のための最低速度と最大精度。これを満たさない場合はyawを推定しない。
constexpr float GPS_INITIAL_YAW_MIN_SPEED_MM_S = 300.0f;
constexpr uint32_t GPS_INITIAL_YAW_MAX_ACCURACY_MM_S = 1000;

bool isFresh(uint32_t now_ms, uint32_t timestamp_ms, uint32_t timeout_ms)
{
    return timestamp_ms != 0 &&
        static_cast<uint32_t>(now_ms - timestamp_ms) <= timeout_ms;
}

bool readActiveJog(uint32_t now_ms, JogData& jog)
{
    return xQueuePeek(mbx_can_jog_cmd, &jog, 0) == pdTRUE &&
        static_cast<uint32_t>(now_ms - jog.timestamp_ms) < jog.duration_ms;
}

uint32_t timestampDifference(uint32_t first, uint32_t second)
{
    const int32_t difference = static_cast<int32_t>(first - second);
    return static_cast<uint32_t>(difference >= 0 ? difference : -difference);
}

float normalizeTwoPi(float angle)
{
    angle = fmodf(angle, static_cast<float>(2.0 * M_PI));
    if (angle < 0.0f) angle += static_cast<float>(2.0 * M_PI);
    return angle;
}

Attitude determineAttitude(float acceleration_z_g, bool valid)
{
    if (!valid || !isfinite(acceleration_z_g)) return Attitude::Unknown;
    if (acceleration_z_g <= ATTITUDE_FLIPPED_THRESHOLD_G) {
        return Attitude::Flipped;
    }
    if (fabsf(acceleration_z_g) <= ATTITUDE_TILT_THRESHOLD_G) {
        return Attitude::HighTilt;
    }
    return Attitude::Normal;
}

bool gpsCourseYaw(
    const Domain::Fusion::GpsUpdate& gps,
    float& yaw_rad)
{
    if (!gps.velocity_valid) return false;

    const float east = static_cast<float>(gps.velocity_east_mm_s);
    const float north = static_cast<float>(gps.velocity_north_mm_s);
    const float speed = hypotf(east, north);
    if (speed < GPS_INITIAL_YAW_MIN_SPEED_MM_S ||
        gps.speed_accuracy_mm_s > GPS_INITIAL_YAW_MAX_ACCURACY_MM_S) {
        return false;
    }
    yaw_rad = normalizeTwoPi(atan2f(north, east));
    return true;
}

uint32_t estimateNmeaAccuracyMm(const Gps::NmeaObservation& observation)
{
    if (!observation.hdop_valid || observation.hdop_x100 == 0U) {
        return NMEA_MAX_ACCURACY_MM;
    }
    const uint32_t estimate =
        static_cast<uint32_t>(observation.hdop_x100) *
        NMEA_BASE_ACCURACY_MM / 100U;
    if (estimate < NMEA_BASE_ACCURACY_MM) return NMEA_BASE_ACCURACY_MM;
    if (estimate > NMEA_MAX_ACCURACY_MM) return NMEA_MAX_ACCURACY_MM;
    return estimate;
}

Coordinate makeInitialCoordinate(uint32_t timestamp_ms)
{
    Coordinate coordinate{};
    coordinate.attitude = Attitude::Unknown;
    coordinate.source_flags = CORD_SRC_NONE;
    coordinate.timestamp_ms = timestamp_ms;
    coordinate.is_first_gps_valid = false;
    return coordinate;
}

} // namespace

void taskCoordinate(void* pvParameters)
{
    (void)pvParameters;

    Domain::Fusion::Config fusion_config{};
    fusion_config.track_width_mm = 180.0f;
    fusion_config.field_size_x_mm = FieldConfig::SIZE_X_MM;
    fusion_config.field_size_y_mm = FieldConfig::SIZE_Y_MM;
    fusion_config.magnetic_declination_rad =
        FieldConfig::MAGNETIC_DECLINATION_RAD;
    Domain::Fusion::Filter filter(fusion_config);
    const Domain::Geodesy::GpsToXY gps_to_xy(
        FieldConfig::GOAL_LATITUDE_E7,
        FieldConfig::GOAL_LONGITUDE_E7,
        FieldConfig::GOAL_X_MM,
        FieldConfig::GOAL_Y_MM);

    uint32_t last_imu_timestamp_ms = 0;
    uint32_t last_encoder_timestamp_ms = 0;
    uint32_t last_gps_timestamp_ms = 0;
    uint32_t last_magnetic_timestamp_ms = 0;

    bool first_gps_received = false;
    uint8_t gps_initial_sample_count = 0;
    int64_t gps_initial_sum_x_mm = 0;
    int64_t gps_initial_sum_y_mm = 0;
    int32_t gps_initial_anchor_x_mm = 0;
    int32_t gps_initial_anchor_y_mm = 0;
    uint32_t gps_initial_last_timestamp_ms = 0;
    Can::Data::Encoder anomaly_previous_encoder{};
    bool have_anomaly_previous_encoder = false;
    float anomaly_left_velocity_mm_s = 0.0f;
    float anomaly_right_velocity_mm_s = 0.0f;
    uint16_t previous_anomaly_flags = Domain::Fusion::ANOMALY_NONE;
    uint32_t anomaly_since_ms = 0;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(FUSION_PERIOD_MS);

    while (true) {
        xTaskDelayUntil(&last_wake, period);
        const uint32_t now_ms = millis();

        SystemData system{};
        if (xQueuePeek(mbx_system_data, &system, 0) != pdTRUE) continue;

        // DEBUGはFlash操作専用なので、センサーmailboxとEKFには触れない。
        if (system.boot_mode == BootMode::DEBUG) {
            const Domain::Fusion::GpsUpdate invalid_gps{};
            xQueueOverwrite(
                mbx_gps_local_observation,
                &invalid_gps);
            const Coordinate initial = makeInitialCoordinate(now_ms);
            xQueueOverwrite(mbx_coordinate, &initial);
            continue;
        }

        const bool reset_for_sequence =
            system.state == SystemState::STATE_PRELAUNCH ||
            system.state == SystemState::STATE_AWAIT_ASCENT;
        if (reset_for_sequence) {
            filter.reset();
            last_imu_timestamp_ms = 0;
            last_encoder_timestamp_ms = 0;
            last_gps_timestamp_ms = 0;
            last_magnetic_timestamp_ms = 0;
            first_gps_received = false;
            gps_initial_sample_count = 0;
            gps_initial_sum_x_mm = 0;
            gps_initial_sum_y_mm = 0;
            gps_initial_last_timestamp_ms = 0;
            have_anomaly_previous_encoder = false;
            previous_anomaly_flags = Domain::Fusion::ANOMALY_NONE;
            anomaly_since_ms = 0;

            const Domain::Fusion::GpsUpdate invalid_gps{};
            xQueueOverwrite(
                mbx_gps_local_observation,
                &invalid_gps);
            const Coordinate initial = makeInitialCoordinate(now_ms);
            xQueueOverwrite(mbx_coordinate, &initial);
            continue;
        }

        Can::Data::Sensor acceleration{};
        Can::Data::AngularVelocity angular_velocity{};
        Can::Data::MagneticField magnetic{};
        Can::Data::Encoder encoder{};
        Gps::NmeaObservation nmea_observation{};
        Gps::NavPvtObservation nav_pvt_observation{};
        Domain::Fusion::GpsUpdate gps_observation{};

        const bool acceleration_valid =
            xQueuePeek(mbx_can_sensor, &acceleration, 0) == pdTRUE &&
            isFresh(now_ms, acceleration.ts_ms, IMU_TIMEOUT_MS) &&
            isfinite(acceleration.acc_x) &&
            isfinite(acceleration.acc_y) &&
            isfinite(acceleration.acc_z);
        const bool angular_velocity_valid =
            xQueuePeek(
                mbx_can_angular_velocity,
                &angular_velocity,
                0) == pdTRUE &&
            isFresh(
                now_ms,
                angular_velocity.ts_ms,
                IMU_TIMEOUT_MS) &&
            isfinite(angular_velocity.x_rad_s) &&
            isfinite(angular_velocity.y_rad_s) &&
            isfinite(angular_velocity.z_rad_s);
        const bool imu_valid =
            acceleration_valid &&
            angular_velocity_valid &&
            timestampDifference(
                acceleration.ts_ms,
                angular_velocity.ts_ms) <= IMU_PAIR_MAX_DIFFERENCE_MS;

        const bool magnetic_valid =
            xQueuePeek(mbx_can_magnetic, &magnetic, 0) == pdTRUE &&
            isFresh(now_ms, magnetic.ts_ms, MAGNETIC_TIMEOUT_MS) &&
            isfinite(magnetic.x_uT) &&
            isfinite(magnetic.y_uT) &&
            isfinite(magnetic.z_uT);
        const bool encoder_valid =
            xQueuePeek(mbx_can_encoder, &encoder, 0) == pdTRUE &&
            isFresh(now_ms, encoder.ts_ms, ENCODER_TIMEOUT_MS);
        if (encoder_valid &&
            (!have_anomaly_previous_encoder ||
             encoder.ts_ms != anomaly_previous_encoder.ts_ms)) {
            if (have_anomaly_previous_encoder) {
                const uint32_t encoder_dt_ms =
                    static_cast<uint32_t>(
                        encoder.ts_ms -
                        anomaly_previous_encoder.ts_ms);
                if (encoder_dt_ms > 0U &&
                    encoder_dt_ms <= ENCODER_TIMEOUT_MS) {
                    const float scale =
                        1000.0f / static_cast<float>(encoder_dt_ms);
                    anomaly_left_velocity_mm_s =
                        static_cast<float>(
                            encoder.left_mm -
                            anomaly_previous_encoder.left_mm) *
                        scale;
                    anomaly_right_velocity_mm_s =
                        static_cast<float>(
                            encoder.right_mm -
                            anomaly_previous_encoder.right_mm) *
                        scale;
                }
            }
            anomaly_previous_encoder = encoder;
            have_anomaly_previous_encoder = true;
        }
        const bool nav_pvt_available =
            xQueuePeek(
                mbx_gps_nav_pvt_observation,
                &nav_pvt_observation,
                0) == pdTRUE &&
            nav_pvt_observation.received_ms != 0U;
        const bool nmea_available =
            xQueuePeek(
                mbx_gps_nmea_observation,
                &nmea_observation,
                0) == pdTRUE &&
            nmea_observation.location_valid &&
            isFresh(
                now_ms,
                nmea_observation.location_timestamp_ms,
                GPS_TIMEOUT_MS);
        int32_t gps_x_mm = 0;
        int32_t gps_y_mm = 0;
        const bool nav_pvt_xy_valid =
            nav_pvt_available &&
            nav_pvt_observation.fix_ok &&
            nav_pvt_observation.horizontal_accuracy_mm > 0U &&
            nav_pvt_observation.horizontal_accuracy_mm <=
                GPS_MAX_FUSION_ACCURACY_MM &&
            isFresh(
                now_ms,
                nav_pvt_observation.received_ms,
                GPS_TIMEOUT_MS) &&
            isFresh(
                now_ms,
                nav_pvt_observation.timestamp_ms,
                GPS_TIMEOUT_MS) &&
            gps_to_xy.convert(
                nav_pvt_observation.latitude_e7,
                nav_pvt_observation.longitude_e7,
                gps_x_mm,
                gps_y_mm);
        uint8_t selected_gps_satellites = 0;
        bool selected_nav_pvt = false;
        if (nav_pvt_xy_valid) {
            selected_nav_pvt = true;
            gps_observation.timestamp_ms =
                nav_pvt_observation.timestamp_ms;
            gps_observation.x_mm = gps_x_mm;
            gps_observation.y_mm = gps_y_mm;
            gps_observation.horizontal_accuracy_mm =
                nav_pvt_observation.horizontal_accuracy_mm;
            gps_observation.velocity_north_mm_s =
                nav_pvt_observation.velocity_north_mm_s;
            gps_observation.velocity_east_mm_s =
                nav_pvt_observation.velocity_east_mm_s;
            gps_observation.speed_accuracy_mm_s =
                nav_pvt_observation.speed_accuracy_mm_s;
            gps_observation.velocity_valid = true;
            gps_observation.fix_type =
                nav_pvt_observation.fix_type;
            gps_observation.fix_ok = true;
            selected_gps_satellites = nav_pvt_observation.satellites;
        } else if (nmea_available &&
                   gps_to_xy.convert(
                       nmea_observation.latitude_e7,
                       nmea_observation.longitude_e7,
                       gps_x_mm,
                       gps_y_mm)) {
            // NMEAは位置だけを補助する。速度・進行方向はNAV-PVT取得時だけ使う。
            gps_observation.timestamp_ms =
                nmea_observation.location_timestamp_ms;
            gps_observation.x_mm = gps_x_mm;
            gps_observation.y_mm = gps_y_mm;
            gps_observation.horizontal_accuracy_mm =
                estimateNmeaAccuracyMm(nmea_observation);
            gps_observation.velocity_north_mm_s = 0;
            gps_observation.velocity_east_mm_s = 0;
            gps_observation.speed_accuracy_mm_s = UINT32_MAX;
            gps_observation.velocity_valid = false;
            gps_observation.fix_type = 2;
            gps_observation.fix_ok = true;
            const bool nmea_satellites_fresh =
                nmea_observation.satellites_valid &&
                isFresh(
                    now_ms,
                    nmea_observation.satellites_timestamp_ms,
                    GPS_TIMEOUT_MS);
            selected_gps_satellites = nmea_satellites_fresh
                ? nmea_observation.satellites : 0;
        }
        xQueueOverwrite(
            mbx_gps_local_observation,
            &gps_observation);
        const bool gps_valid =
            gps_observation.fix_ok &&
            isFresh(
                now_ms,
                gps_observation.timestamp_ms,
                GPS_TIMEOUT_MS);

        if (!filter.initialized()) {
            const bool gps_initial_candidate =
                gps_valid &&
                selected_gps_satellites >=
                    GPS_INITIAL_MIN_SATELLITES &&
                gps_observation.horizontal_accuracy_mm <=
                    (selected_nav_pvt
                        ? GPS_INITIAL_MAX_ACCURACY_MM
                        : GPS_INITIAL_NMEA_MAX_ACCURACY_MM) &&
                (gps_initial_last_timestamp_ms == 0U ||
                 static_cast<int32_t>(
                     gps_observation.timestamp_ms -
                     gps_initial_last_timestamp_ms) >=
                    static_cast<int32_t>(
                        GPS_INITIAL_MIN_SAMPLE_INTERVAL_MS));
            if (gps_initial_candidate) {
                gps_initial_last_timestamp_ms =
                    gps_observation.timestamp_ms;
                const float anchor_distance_mm = hypotf(
                    static_cast<float>(
                        gps_observation.x_mm -
                        gps_initial_anchor_x_mm),
                    static_cast<float>(
                        gps_observation.y_mm -
                        gps_initial_anchor_y_mm));
                if (gps_initial_sample_count == 0U ||
                    anchor_distance_mm >
                        GPS_INITIAL_STABLE_RADIUS_MM) {
                    gps_initial_sample_count = 1;
                    gps_initial_anchor_x_mm =
                        gps_observation.x_mm;
                    gps_initial_anchor_y_mm =
                        gps_observation.y_mm;
                    gps_initial_sum_x_mm =
                        gps_observation.x_mm;
                    gps_initial_sum_y_mm =
                        gps_observation.y_mm;
                } else {
                    gps_initial_sample_count++;
                    gps_initial_sum_x_mm +=
                        gps_observation.x_mm;
                    gps_initial_sum_y_mm +=
                        gps_observation.y_mm;
                }
            }

            if (gps_initial_sample_count >=
                GPS_INITIAL_REQUIRED_SAMPLES) {
                float initial_yaw = 0.0f;
                const bool initial_yaw_valid =
                    gpsCourseYaw(gps_observation, initial_yaw);

                filter.initialize(
                    static_cast<int32_t>(
                        gps_initial_sum_x_mm /
                        gps_initial_sample_count),
                    static_cast<int32_t>(
                        gps_initial_sum_y_mm /
                        gps_initial_sample_count),
                    initial_yaw,
                    initial_yaw_valid,
                    gps_observation.timestamp_ms,
                    gps_observation.horizontal_accuracy_mm);
                first_gps_received = true;
                last_gps_timestamp_ms = gps_observation.timestamp_ms;
            }
        }

        filter.beginCycle();

        if (filter.initialized() &&
            imu_valid &&
            angular_velocity.ts_ms != last_imu_timestamp_ms) {
            const SensorAxisTransform::Vector3 body_acceleration =
                SensorAxisTransform::imuToBody(
                    acceleration.acc_x,
                    acceleration.acc_y,
                    acceleration.acc_z);
            const SensorAxisTransform::Vector3 body_angular_velocity =
                SensorAxisTransform::imuToBody(
                    angular_velocity.x_rad_s,
                    angular_velocity.y_rad_s,
                    angular_velocity.z_rad_s);

            Domain::Fusion::ImuObservation observation{};
            observation.timestamp_ms = angular_velocity.ts_ms;
            observation.accel_x_g = body_acceleration.x;
            observation.accel_y_g = body_acceleration.y;
            observation.accel_z_g = body_acceleration.z;
            observation.gyro_x_rad_s = body_angular_velocity.x;
            observation.gyro_y_rad_s = body_angular_velocity.y;
            observation.gyro_z_rad_s = body_angular_velocity.z;
            filter.predict(observation);
            last_imu_timestamp_ms = angular_velocity.ts_ms;
        }

        if (filter.initialized() &&
            encoder_valid &&
            encoder.ts_ms != last_encoder_timestamp_ms) {
            Domain::Fusion::EncoderObservation observation{};
            observation.timestamp_ms = encoder.ts_ms;
            observation.left_mm = encoder.left_mm;
            observation.right_mm = encoder.right_mm;
            filter.updateEncoder(observation);
            last_encoder_timestamp_ms = encoder.ts_ms;
        }

        if (filter.initialized() &&
            gps_valid &&
            gps_observation.timestamp_ms != last_gps_timestamp_ms) {
            filter.updateGps(gps_observation);
            last_gps_timestamp_ms = gps_observation.timestamp_ms;
            first_gps_received = true;
        }

        if (filter.initialized() &&
            magnetic_valid &&
            magnetic.ts_ms != last_magnetic_timestamp_ms) {
            const SensorAxisTransform::Vector3 body_magnetic =
                SensorAxisTransform::magneticToBody(
                    magnetic.x_uT,
                    magnetic.y_uT,
                    magnetic.z_uT);
            Domain::Fusion::MagneticObservation observation{};
            observation.timestamp_ms = magnetic.ts_ms;
            observation.x_uT = body_magnetic.x;
            observation.y_uT = body_magnetic.y;
            observation.z_uT = body_magnetic.z;
            JogData magnetic_jog{};
            const bool magnetic_jog_valid =
                readActiveJog(now_ms, magnetic_jog);
            observation.motor_command_active =
                magnetic_jog_valid &&
                (fabsf(magnetic_jog.velocity_mm_s) >=
                    ANOMALY_COMMAND_SPEED_MM_S ||
                 fabsf(magnetic_jog.omega_rad_s) >=
                    ANOMALY_COMMAND_YAW_RATE_RAD_S);
            filter.updateMagnetic(observation);
            last_magnetic_timestamp_ms = magnetic.ts_ms;
        }

        const Attitude attitude =
            determineAttitude(acceleration.acc_z, acceleration_valid);
        Coordinate coordinate = makeInitialCoordinate(now_ms);

        if (filter.initialized()) {
            const Domain::Fusion::Output fusion = filter.output(now_ms);
            coordinate.x_mm = fusion.x_mm;
            coordinate.y_mm = fusion.y_mm;
            coordinate.heading_rad = fusion.yaw_rad;
            coordinate.attitude = attitude;
            coordinate.timestamp_ms = fusion.timestamp_ms;
            coordinate.forward_velocity_mm_s = fusion.forward_velocity_mm_s;
            coordinate.yaw_rate_rad_s = fusion.yaw_rate_rad_s;
            coordinate.position_std_mm = fusion.position_std_mm;
            coordinate.yaw_std_rad = fusion.yaw_std_rad;
            coordinate.fusion_status_flags = fusion.status_flags;
            coordinate.fusion_quality = fusion.quality;
            coordinate.gps_health = fusion.gps_health;
            coordinate.encoder_health = fusion.encoder_health;
            coordinate.imu_health = fusion.imu_health;
            coordinate.magnetic_health = fusion.magnetic_health;
            coordinate.motion_anomaly_flags = fusion.anomaly_flags;
            coordinate.motion_anomaly_since_ms =
                fusion.anomaly_since_ms;

            // Localizationはスタックを確定せず、物理的な不整合だけを通知する。
            JogData jog{};
            const bool command_valid =
                readActiveJog(now_ms, jog);
            const float command_velocity =
                command_valid ? jog.velocity_mm_s : 0.0f;
            const float command_yaw_rate =
                command_valid ? jog.omega_rad_s : 0.0f;
            const bool translation_requested =
                fabsf(command_velocity) >=
                ANOMALY_COMMAND_SPEED_MM_S;
            const bool rotation_requested =
                fabsf(command_yaw_rate) >=
                ANOMALY_COMMAND_YAW_RATE_RAD_S;
            const bool left_stopped =
                fabsf(anomaly_left_velocity_mm_s) <
                ANOMALY_STOPPED_WHEEL_MM_S;
            const bool right_stopped =
                fabsf(anomaly_right_velocity_mm_s) <
                ANOMALY_STOPPED_WHEEL_MM_S;

            uint16_t anomaly_flags = Domain::Fusion::ANOMALY_NONE;
            if ((translation_requested || rotation_requested) &&
                encoder_valid && left_stopped && right_stopped) {
                anomaly_flags |=
                    Domain::Fusion::ANOMALY_WHEEL_NOT_MOVING;
            }
            if (translation_requested && encoder_valid &&
                left_stopped != right_stopped) {
                anomaly_flags |=
                    Domain::Fusion::ANOMALY_ONE_SIDE_BLOCKED;
            }
            if (encoder_valid && angular_velocity_valid) {
                const float encoder_yaw_rate =
                    (anomaly_right_velocity_mm_s -
                     anomaly_left_velocity_mm_s) /
                    fusion_config.track_width_mm;
                if (fabsf(
                        encoder_yaw_rate -
                        angular_velocity.z_rad_s) >
                    ANOMALY_ENCODER_GYRO_RATE_RAD_S) {
                    anomaly_flags |=
                        Domain::Fusion::
                            ANOMALY_ENCODER_GYRO_MISMATCH;
                }
            }
            if (attitude == Attitude::HighTilt ||
                attitude == Attitude::Flipped) {
                anomaly_flags |=
                    Domain::Fusion::ANOMALY_HIGH_TILT;
            }
            if ((translation_requested || rotation_requested) &&
                !encoder_valid && !imu_valid) {
                anomaly_flags |=
                    Domain::Fusion::
                        ANOMALY_MOTION_UNOBSERVABLE;
            }
            if (anomaly_flags == Domain::Fusion::ANOMALY_NONE) {
                anomaly_since_ms = 0;
            } else if (anomaly_flags != previous_anomaly_flags ||
                       anomaly_since_ms == 0U) {
                anomaly_since_ms = now_ms;
            }
            previous_anomaly_flags = anomaly_flags;
            coordinate.motion_anomaly_flags = anomaly_flags;
            coordinate.motion_anomaly_since_ms = anomaly_since_ms;
            coordinate.is_first_gps_valid = first_gps_received;

            if ((fusion.status_flags &
                 Domain::Fusion::STATUS_GPS_USED) != 0U) {
                coordinate.source_flags |= CORD_SRC_GPS;
            }
            if ((fusion.status_flags &
                 Domain::Fusion::STATUS_ENCODER_USED) != 0U) {
                coordinate.source_flags |= CORD_SRC_ENCODER;
            }
            if ((fusion.status_flags &
                 Domain::Fusion::STATUS_YAW_USABLE) != 0U) {
                coordinate.source_flags |= CORD_SRC_HEADING;
            }
            const uint16_t used_this_cycle =
                Domain::Fusion::STATUS_GPS_USED |
                Domain::Fusion::STATUS_ENCODER_USED |
                Domain::Fusion::STATUS_IMU_USED |
                Domain::Fusion::STATUS_MAGNETIC_USED;
            if ((fusion.status_flags & used_this_cycle) == 0U) {
                coordinate.source_flags |= CORD_SRC_HOLD;
            }
        } else {
            coordinate.attitude = attitude;
        }

        if (gps_valid) {
            coordinate.gps_x_mm = gps_observation.x_mm;
            coordinate.gps_y_mm = gps_observation.y_mm;
        }
        if (encoder_valid) {
            coordinate.encoder_left_mm = encoder.left_mm;
            coordinate.encoder_right_mm = encoder.right_mm;
        }

        xQueueOverwrite(mbx_coordinate, &coordinate);
    }
}
