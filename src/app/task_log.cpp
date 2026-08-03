#include "tasks.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "algorithm/astar.h"
#include "domain/fusion/fusion_types.h"
#include "platform/sensor_axis_transform.h"

#include <math.h>

namespace {

// Flash snapshotとTWELITE snapshotを全状態で10 Hzに統一する。
constexpr uint32_t LOG_PERIOD_MS = 100;
constexpr uint32_t IMU_TIMEOUT_MS = 100;
constexpr uint32_t MAGNETIC_TIMEOUT_MS = 250;
constexpr uint32_t ENCODER_TIMEOUT_MS = 150;
constexpr uint32_t GPS_TIMEOUT_MS = 2000;
constexpr uint32_t CAMERA_TIMEOUT_MS = 2000;

enum ValidFlag : uint16_t {
    VALID_GPS = 1U << 0,
    VALID_IMU = 1U << 1,
    VALID_MAGNETIC = 1U << 2,
    VALID_ENCODER = 1U << 3,
    VALID_COORDINATE = 1U << 4,
    VALID_CAMERA = 1U << 5,
    VALID_JOG = 1U << 6,
    VALID_GPS_NMEA = 1U << 7,
    VALID_GPS_NAV_PVT = 1U << 8
};

bool isFresh(uint32_t now, uint32_t timestamp, uint32_t timeout)
{
    return timestamp != 0 && static_cast<uint32_t>(now - timestamp) <= timeout;
}

uint16_t ageMs(uint32_t now, uint32_t timestamp)
{
    if (timestamp == 0) return UINT16_MAX;
    const uint32_t age = static_cast<uint32_t>(now - timestamp);
    return age > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(age);
}

int16_t i16(float value)
{
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return static_cast<int16_t>(lroundf(value));
}

uint16_t yawDeg1e2(float rad)
{
    float angle = fmodf(rad, 2.0f * static_cast<float>(M_PI));
    if (angle < 0.0f) angle += 2.0f * static_cast<float>(M_PI);
    return static_cast<uint16_t>(lroundf(angle * 18000.0f / M_PI));
}

bool magneticYaw(const Can::Data::MagneticField& magnetic, float& yaw)
{
    const SensorAxisTransform::Vector3 body =
        SensorAxisTransform::magneticToBody(
            magnetic.x_uT, magnetic.y_uT, magnetic.z_uT);
    const float horizontal = hypotf(body.x, body.y);
    const float total = sqrtf(
        body.x * body.x + body.y * body.y + body.z * body.z);
    if (!isfinite(horizontal) || !isfinite(total) ||
        horizontal < 5.0f || total < 10.0f || total > 100.0f) {
        return false;
    }
    yaw = static_cast<float>(M_PI_2) - atan2f(body.y, body.x);
    return true;
}

} // namespace

void taskLog(void* pvParameters)
{
    (void)pvParameters;
    uint32_t flash_message_number = 0;
    uint16_t telemetry_message_number = 0;
    uint8_t telemetry_page = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOG_PERIOD_MS));
        const uint32_t now = millis();

        SystemData system{};
        if (xQueuePeek(mbx_system_data, &system, 0) != pdTRUE ||
            system.boot_mode == BootMode::DEBUG) {
            continue;
        }

        Coordinate coordinate{};
        Gps::NmeaObservation nmea_observation{};
        Gps::NavPvtObservation gps_observation{};
        Domain::Fusion::GpsUpdate gps_local_observation{};
        Can::Data::Sensor sensor{};
        Can::Data::AngularVelocity angular{};
        Can::Data::MagneticField magnetic{};
        Can::Data::Encoder encoder{};
        JogData jog{};
        Rasp::CameraData camera{};
        StuckStatus stuck{};
        FlashStatus flash_status{};

        const bool has_coordinate = xQueuePeek(mbx_coordinate, &coordinate, 0) == pdTRUE;
        const bool has_nmea =
            xQueuePeek(
                mbx_gps_nmea_observation,
                &nmea_observation,
                0) == pdTRUE &&
            nmea_observation.sentence_timestamp_ms != 0;
        const bool has_gps =
            xQueuePeek(
                mbx_gps_nav_pvt_observation,
                &gps_observation,
                0) == pdTRUE &&
            gps_observation.received_ms != 0;
        const bool has_local_gps =
            xQueuePeek(
                mbx_gps_local_observation,
                &gps_local_observation,
                0) == pdTRUE;
        const bool has_sensor = xQueuePeek(mbx_can_sensor, &sensor, 0) == pdTRUE;
        const bool has_angular = xQueuePeek(mbx_can_angular_velocity, &angular, 0) == pdTRUE;
        const bool has_magnetic = xQueuePeek(mbx_can_magnetic, &magnetic, 0) == pdTRUE;
        const bool has_encoder = xQueuePeek(mbx_can_encoder, &encoder, 0) == pdTRUE;
        const bool has_jog = xQueuePeek(mbx_can_jog_cmd, &jog, 0) == pdTRUE;
        const bool has_camera = xQueuePeek(mbx_camera_data, &camera, 0) == pdTRUE;
        const bool has_stuck = xQueuePeek(mbx_stuck_status, &stuck, 0) == pdTRUE;
        const bool has_flash = xQueuePeek(mbx_flash_status, &flash_status, 0) == pdTRUE;

        const bool nav_pvt_selected =
            has_gps &&
            gps_observation.fix_ok &&
            gps_observation.timestamp_ms ==
                gps_local_observation.timestamp_ms;
        const bool nmea_selected =
            has_nmea &&
            nmea_observation.location_valid &&
            nmea_observation.location_timestamp_ms ==
                gps_local_observation.timestamp_ms;
        const bool gps_valid =
            has_local_gps &&
            gps_local_observation.fix_ok &&
            (nav_pvt_selected || nmea_selected) &&
            isFresh(
                now,
                gps_local_observation.timestamp_ms,
                GPS_TIMEOUT_MS);
        const bool imu_valid = has_sensor && has_angular &&
            isFresh(now, sensor.ts_ms, IMU_TIMEOUT_MS) &&
            isFresh(now, angular.ts_ms, IMU_TIMEOUT_MS);
        const bool magnetic_fresh = has_magnetic &&
            isFresh(now, magnetic.ts_ms, MAGNETIC_TIMEOUT_MS);
        const bool encoder_valid = has_encoder &&
            isFresh(now, encoder.ts_ms, ENCODER_TIMEOUT_MS);
        const bool camera_valid = has_camera &&
            isFresh(now, camera.received_ms, CAMERA_TIMEOUT_MS);
        const bool jog_valid = has_jog &&
            static_cast<uint32_t>(now - jog.timestamp_ms) < jog.duration_ms;
        const bool coordinate_valid = has_coordinate &&
            (coordinate.fusion_status_flags &
             Domain::Fusion::STATUS_POSITION_USABLE) != 0U;

        float magnetic_yaw = 0.0f;
        const bool magnetic_valid =
            magnetic_fresh && magneticYaw(magnetic, magnetic_yaw);

        uint16_t valid_flags = 0;
        if (gps_valid) valid_flags |= VALID_GPS;
        if (gps_valid && nmea_selected) valid_flags |= VALID_GPS_NMEA;
        if (gps_valid && nav_pvt_selected) valid_flags |= VALID_GPS_NAV_PVT;
        if (imu_valid) valid_flags |= VALID_IMU;
        if (magnetic_valid) valid_flags |= VALID_MAGNETIC;
        if (encoder_valid) valid_flags |= VALID_ENCODER;
        if (coordinate_valid) valid_flags |= VALID_COORDINATE;
        if (camera_valid) valid_flags |= VALID_CAMERA;
        if (jog_valid) valid_flags |= VALID_JOG;

        uint16_t jog_remain = 0;
        if (jog_valid) {
            const uint32_t remain =
                jog.duration_ms - static_cast<uint32_t>(now - jog.timestamp_ms);
            jog_remain = remain > UINT16_MAX ? UINT16_MAX : remain;
        }

        AStar::GridPos cell{};
        AStar::Config map_config{};
        const bool cell_valid = AStar::worldToGridChecked(
            static_cast<float>(coordinate.x_mm),
            static_cast<float>(coordinate.y_mm), cell, map_config);

        uint32_t map_update_count = 0;
        if (xSemaphoreTake(mutex_grid_map, pdMS_TO_TICKS(5)) == pdTRUE) {
            map_update_count = grid_map_update_count;
            xSemaphoreGive(mutex_grid_map);
        }

        flash_message_number++;
        if (flash_message_number == 0) flash_message_number = 1;

        Flash::LogFrame log{};
        log.format_version = Flash::LOG_FORMAT_VERSION;
        log.flash_file_index =
            has_flash && flash_status.initialized ? flash_status.active_file_index : -1;
        log.mission_state = static_cast<uint8_t>(system.state);
        log.boot_mode = static_cast<uint8_t>(system.boot_mode);
        log.message_number = flash_message_number;
        log.timestamp_ms = now;
        log.valid_flags = valid_flags;
        log.fusion_status_flags = coordinate.fusion_status_flags;
        log.lat_1e7 = !gps_valid ? 0
            : nav_pvt_selected ? gps_observation.latitude_e7
            : nmea_observation.latitude_e7;
        log.lng_1e7 = !gps_valid ? 0
            : nav_pvt_selected ? gps_observation.longitude_e7
            : nmea_observation.longitude_e7;
        log.gps_x_mm =
            gps_valid ? gps_local_observation.x_mm : 0;
        log.gps_y_mm =
            gps_valid ? gps_local_observation.y_mm : 0;
        log.x_mm = coordinate.x_mm;
        log.y_mm = coordinate.y_mm;
        log.yaw_deg_1e2 = yawDeg1e2(coordinate.heading_rad);
        log.forward_velocity_mm_s = i16(coordinate.forward_velocity_mm_s);
        log.yaw_rate_rad_s_x1000 = i16(coordinate.yaw_rate_rad_s * 1000.0f);
        log.position_std_mm = coordinate.position_std_mm;
        log.yaw_std_mrad = i16(coordinate.yaw_std_rad * 1000.0f);
        log.gps_fix_type = has_gps ? gps_observation.fix_type : 0;
        log.gps_satellites = has_gps ? gps_observation.satellites : 0;
        log.gps_horizontal_accuracy_mm =
            has_gps ? gps_observation.horizontal_accuracy_mm : 0;
        log.gps_velocity_east_mm_s =
            has_gps ? gps_observation.velocity_east_mm_s : 0;
        log.gps_velocity_north_mm_s =
            has_gps ? gps_observation.velocity_north_mm_s : 0;
        log.gps_speed_accuracy_mm_s =
            has_gps ? gps_observation.speed_accuracy_mm_s : 0;
        log.acc_x_mg = imu_valid ? i16(sensor.acc_x * 1000.0f) : 0;
        log.acc_y_mg = imu_valid ? i16(sensor.acc_y * 1000.0f) : 0;
        log.acc_z_mg = imu_valid ? i16(sensor.acc_z * 1000.0f) : 0;
        log.gyro_z_rad_s_x1000 = imu_valid ? i16(angular.z_rad_s * 1000.0f) : 0;
        log.magnetic_yaw_deg_1e2 = magnetic_valid ? yawDeg1e2(magnetic_yaw) : 0;
        log.pressure_pa = has_sensor ? sensor.atm : 0;
        log.encoder_left_mm = encoder_valid ? encoder.left_mm : 0;
        log.encoder_right_mm = encoder_valid ? encoder.right_mm : 0;
        log.imu_age_ms = has_sensor && has_angular
            ? max(ageMs(now, sensor.ts_ms), ageMs(now, angular.ts_ms)) : UINT16_MAX;
        log.magnetic_age_ms = has_magnetic ? ageMs(now, magnetic.ts_ms) : UINT16_MAX;
        log.encoder_age_ms = has_encoder ? ageMs(now, encoder.ts_ms) : UINT16_MAX;
        log.gps_age_ms = has_gps ? ageMs(now, gps_observation.timestamp_ms) : UINT16_MAX;
        log.cell_x = cell_valid ? static_cast<int8_t>(cell.x) : -1;
        log.cell_y = cell_valid ? static_cast<int8_t>(cell.y) : -1;
        log.attitude = static_cast<uint8_t>(coordinate.attitude);
        log.stuck_reason = has_stuck ? static_cast<uint8_t>(stuck.reason) : 0;
        log.stuck_cell_x = has_stuck ? stuck.obstacle_cell_x : UINT8_MAX;
        log.stuck_cell_y = has_stuck ? stuck.obstacle_cell_y : UINT8_MAX;
        log.jog_velocity_mm_s = jog_valid ? i16(jog.velocity_mm_s) : 0;
        log.jog_omega_rad_s_x100 = jog_valid ? i16(jog.omega_rad_s * 100.0f) : 0;
        log.jog_remain_ms = jog_remain;
        log.camera_valid = camera_valid;
        log.camera_target_found = camera_valid ? camera.frame.target_found : 0;
        log.camera_confidence = camera_valid ? camera.frame.confidence : 0;
        log.camera_occupancy_permille =
            camera_valid ? camera.frame.occupancy_permille : 0;
        log.camera_angle_error_deg10 =
            camera_valid ? camera.frame.angle_error_deg10 : 0;
        log.rasp_state = static_cast<uint8_t>(rasp.getState());
        log.gps_state = static_cast<uint8_t>(gps.getStatus());
        log.flash_used_flags = has_flash ? flash_status.used_file_flags : 0;
        log.flash_storage_full = has_flash && flash_status.storage_full;
        log.grid_map_update_count = map_update_count;
        log.fusion_quality =
            static_cast<uint8_t>(coordinate.fusion_quality);
        log.gps_health =
            static_cast<uint8_t>(coordinate.gps_health);
        log.encoder_health =
            static_cast<uint8_t>(coordinate.encoder_health);
        log.imu_health =
            static_cast<uint8_t>(coordinate.imu_health);
        log.magnetic_health =
            static_cast<uint8_t>(coordinate.magnetic_health);
        log.motion_anomaly_flags =
            coordinate.motion_anomaly_flags;
        log.motion_anomaly_age_ms =
            coordinate.motion_anomaly_since_ms == 0U
                ? 0U
                : ageMs(
                    now,
                    coordinate.motion_anomaly_since_ms);
        log.nmea_lat_1e7 =
            has_nmea && nmea_observation.location_valid
                ? nmea_observation.latitude_e7 : 0;
        log.nmea_lng_1e7 =
            has_nmea && nmea_observation.location_valid
                ? nmea_observation.longitude_e7 : 0;
        log.nmea_sentence_age_ms = has_nmea
            ? ageMs(now, nmea_observation.sentence_timestamp_ms)
            : UINT16_MAX;
        log.nmea_location_age_ms = has_nmea
            ? ageMs(now, nmea_observation.location_timestamp_ms)
            : UINT16_MAX;
        log.nmea_satellites_age_ms = has_nmea
            ? ageMs(now, nmea_observation.satellites_timestamp_ms)
            : UINT16_MAX;
        log.nmea_hdop_x100 = nmea_observation.hdop_valid
            ? nmea_observation.hdop_x100 : UINT16_MAX;
        log.nmea_satellites = nmea_observation.satellites_valid
            ? nmea_observation.satellites : 0;
        log.nmea_flags = 0;
        if (has_nmea) log.nmea_flags |= 1U << 0;
        if (nmea_observation.location_valid) log.nmea_flags |= 1U << 1;
        if (nmea_observation.satellites_valid) log.nmea_flags |= 1U << 2;
        if (nmea_observation.hdop_valid) log.nmea_flags |= 1U << 3;
        log.nav_pvt_receive_age_ms = has_gps
            ? ageMs(now, gps_observation.received_ms) : UINT16_MAX;
        xQueueOverwrite(mbx_flash_log, &log);

        telemetry_message_number++;
        if (telemetry_message_number == 0) telemetry_message_number = 1;
        // TWELITEは1秒周期なので、全ページを毎秒順番に送る。
        telemetry_page = static_cast<uint8_t>(
            (now / 1000U) % static_cast<uint8_t>(Twe::TelemetryPage::Count));
        Twe::TelemetryFrame telemetry{};
        telemetry.protocol_version = Twe::TELEMETRY_PROTOCOL_VERSION;
        telemetry.page = static_cast<Twe::TelemetryPage>(telemetry_page);
        telemetry.message_number = telemetry_message_number;
        telemetry.timestamp_ms = now;
        telemetry.mission_state = log.mission_state;
        telemetry.boot_mode = log.boot_mode;
        telemetry.valid_flags = valid_flags;

        if (telemetry.page == Twe::TelemetryPage::Navigation) {
            auto& page = telemetry.data.navigation;
            page.lat_1e7 = log.lat_1e7;
            page.lng_1e7 = log.lng_1e7;
            page.gps_x_mm = log.gps_x_mm;
            page.gps_y_mm = log.gps_y_mm;
            page.x_mm = log.x_mm;
            page.y_mm = log.y_mm;
            page.yaw_deg_1e2 = log.yaw_deg_1e2;
            page.forward_velocity_mm_s = log.forward_velocity_mm_s;
            page.yaw_rate_rad_s_x1000 = log.yaw_rate_rad_s_x1000;
            page.position_std_mm = log.position_std_mm;
            page.yaw_std_mrad = log.yaw_std_mrad;
            page.jog_velocity_mm_s = log.jog_velocity_mm_s;
            page.jog_omega_rad_s_x100 = log.jog_omega_rad_s_x100;
        } else if (telemetry.page == Twe::TelemetryPage::Sensors) {
            auto& page = telemetry.data.sensors;
            page.acc_x_mg = log.acc_x_mg;
            page.acc_y_mg = log.acc_y_mg;
            page.acc_z_mg = log.acc_z_mg;
            page.pressure_pa = log.pressure_pa;
            page.encoder_left_mm = log.encoder_left_mm;
            page.encoder_right_mm = log.encoder_right_mm;
            page.gyro_z_rad_s_x1000 = log.gyro_z_rad_s_x1000;
            page.magnetic_yaw_deg_1e2 = log.magnetic_yaw_deg_1e2;
            page.gps_fix_type = log.gps_fix_type;
            page.gps_satellites = log.gps_satellites;
            page.gps_horizontal_accuracy_mm = log.gps_horizontal_accuracy_mm;
            page.gps_velocity_east_mm_s = log.gps_velocity_east_mm_s;
            page.gps_velocity_north_mm_s = log.gps_velocity_north_mm_s;
            page.gps_speed_accuracy_mm_s = log.gps_speed_accuracy_mm_s;
        } else if (telemetry.page == Twe::TelemetryPage::Health) {
            auto& page = telemetry.data.health;
            page.flash_file_index = log.flash_file_index;
            page.flash_used_flags = log.flash_used_flags;
            page.flash_storage_full = log.flash_storage_full;
            page.rasp_state = log.rasp_state;
            page.gps_state = log.gps_state;
            page.camera_valid = log.camera_valid;
            page.camera_target_found = log.camera_target_found;
            page.camera_confidence = log.camera_confidence;
            page.camera_occupancy_permille = log.camera_occupancy_permille;
            page.camera_angle_error_deg10 = log.camera_angle_error_deg10;
            page.stuck_reason = log.stuck_reason;
            page.stuck_cell_x = log.stuck_cell_x;
            page.stuck_cell_y = log.stuck_cell_y;
            page.attitude = log.attitude;
            page.fusion_status_flags = log.fusion_status_flags;
            page.imu_age_ms = log.imu_age_ms;
            page.magnetic_age_ms = log.magnetic_age_ms;
            page.encoder_age_ms = log.encoder_age_ms;
            page.gps_age_ms = log.gps_age_ms;
            page.jog_remain_ms = log.jog_remain_ms;
            page.grid_map_update_count = log.grid_map_update_count;
            page.fusion_quality = log.fusion_quality;
            page.gps_health = log.gps_health;
            page.encoder_health = log.encoder_health;
            page.imu_health = log.imu_health;
            page.magnetic_health = log.magnetic_health;
            page.motion_anomaly_flags =
                log.motion_anomaly_flags;
            page.motion_anomaly_age_100ms =
                static_cast<uint8_t>(
                    log.motion_anomaly_age_ms / 100U > UINT8_MAX
                        ? UINT8_MAX
                        : log.motion_anomaly_age_ms / 100U);
        } else {
            auto& page = telemetry.data.gps;
            page.nmea_lat_1e7 =
                has_nmea && nmea_observation.location_valid
                    ? nmea_observation.latitude_e7 : 0;
            page.nmea_lng_1e7 =
                has_nmea && nmea_observation.location_valid
                    ? nmea_observation.longitude_e7 : 0;
            page.nmea_sentence_age_ms = has_nmea
                ? ageMs(now, nmea_observation.sentence_timestamp_ms)
                : UINT16_MAX;
            page.nmea_location_age_ms = has_nmea
                ? ageMs(now, nmea_observation.location_timestamp_ms)
                : UINT16_MAX;
            page.nmea_satellites_age_ms = has_nmea
                ? ageMs(now, nmea_observation.satellites_timestamp_ms)
                : UINT16_MAX;
            page.nmea_hdop_x100 = nmea_observation.hdop_valid
                ? nmea_observation.hdop_x100 : UINT16_MAX;
            page.nmea_satellites = nmea_observation.satellites_valid
                ? nmea_observation.satellites : 0;
            page.nmea_flags = 0;
            if (has_nmea) page.nmea_flags |= 1U << 0;
            if (nmea_observation.location_valid) {
                page.nmea_flags |= 1U << 1;
            }
            if (nmea_observation.satellites_valid) {
                page.nmea_flags |= 1U << 2;
            }
            if (nmea_observation.hdop_valid) {
                page.nmea_flags |= 1U << 3;
            }
            page.nav_pvt_lat_1e7 = has_gps
                ? gps_observation.latitude_e7 : 0;
            page.nav_pvt_lng_1e7 = has_gps
                ? gps_observation.longitude_e7 : 0;
            page.nav_pvt_hacc_mm = has_gps
                ? gps_observation.horizontal_accuracy_mm : UINT32_MAX;
            page.nav_pvt_measurement_age_ms = has_gps
                ? ageMs(now, gps_observation.timestamp_ms) : UINT16_MAX;
            page.nav_pvt_receive_age_ms = has_gps
                ? ageMs(now, gps_observation.received_ms) : UINT16_MAX;
            page.nav_pvt_fix_type = has_gps
                ? gps_observation.fix_type : 0;
            page.nav_pvt_satellites = has_gps
                ? gps_observation.satellites : 0;
            page.nav_pvt_flags = 0;
            if (has_gps) page.nav_pvt_flags |= 1U << 0;
            if (has_gps && gps_observation.fix_ok) {
                page.nav_pvt_flags |= 1U << 1;
            }
        }
        xQueueOverwrite(mbx_twe_telemetry, &telemetry);
    }
}
