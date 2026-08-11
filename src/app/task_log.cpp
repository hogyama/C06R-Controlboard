#include "tasks.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "domain/localization/localization_types.h"
#include "domain/sensor/sensor_freshness.h"

#include <esp_timer.h>
#include <math.h>

namespace {

constexpr uint32_t LOG_PERIOD_MS = 100;
constexpr uint32_t IMU_TIMEOUT_MS = 100;
constexpr uint32_t MAGNETIC_TIMEOUT_MS = 250;
constexpr uint32_t PRESSURE_TIMEOUT_MS = 500;
constexpr uint32_t ENCODER_TIMEOUT_MS = 150;
constexpr uint32_t GPS_TIMEOUT_MS = 2000;
constexpr uint32_t COORDINATE_TIMEOUT_MS = 500;
constexpr uint32_t CAMERA_TIMEOUT_MS = 500;
constexpr uint32_t STUCK_TIMEOUT_MS = 300;
constexpr float INVERSE_STANDARD_GRAVITY = 1.0f / 9.80665f;

enum ValidFlag : uint16_t {
    VALID_GPS = 1U << 0,
    VALID_BOARD_IMU = 1U << 1,
    VALID_CAN_IMU = 1U << 2,
    VALID_BOARD_MAGNETIC = 1U << 3,
    VALID_CAN_MAGNETIC = 1U << 4,
    VALID_PRESSURE = 1U << 5,
    VALID_ENCODER = 1U << 6,
    VALID_COORDINATE = 1U << 7,
    VALID_CAMERA = 1U << 8,
    VALID_JOG = 1U << 9,
    VALID_RASP_HEARTBEAT = 1U << 10,
    GPS_LOCALIZATION_ENABLED = 1U << 11,
    VALID_SELECTED_IMU = 1U << 12,
    VALID_SELECTED_MAGNETIC = 1U << 13,
    VALID_STUCK_DIAGNOSTICS = 1U << 14,
    VALID_FLASH = 1U << 15,
};

bool fresh(uint32_t now, uint32_t timestamp, uint32_t timeout)
{
    return timestamp != 0U && static_cast<uint32_t>(now - timestamp) <= timeout;
}

int16_t i16(float value)
{
    if (!isfinite(value)) return 0;
    if (value > 32767.0f) return 32767;
    if (value < -32768.0f) return -32768;
    return static_cast<int16_t>(lroundf(value));
}

uint16_t u16(uint32_t value)
{
    return value > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(value);
}

uint16_t yawDeg1e2(float radians)
{
    if (!isfinite(radians)) return 0;
    float degrees = fmodf(radians * 180.0f / static_cast<float>(M_PI), 360.0f);
    if (degrees < 0.0f) degrees += 360.0f;
    return static_cast<uint16_t>(lroundf(degrees * 100.0f)) % 36000U;
}

uint8_t sensorSources(
    Sensor::Source acceleration,
    Sensor::Source gyroscope,
    Sensor::Source magnetic,
    Sensor::Source pressure)
{
    return (static_cast<uint8_t>(acceleration) & 0x03U) |
        ((static_cast<uint8_t>(gyroscope) & 0x03U) << 2U) |
        ((static_cast<uint8_t>(magnetic) & 0x03U) << 4U) |
        ((static_cast<uint8_t>(pressure) & 0x03U) << 6U);
}

uint8_t flashState(bool available, const FlashStatus& status)
{
    if (!available) return 0;
    uint8_t value = status.initialized ? 1U : 0U;
    if (status.storage_full) value |= 1U << 1U;
    if (status.last_event == FlashLedEvent::InitError ||
        status.last_event == FlashLedEvent::WriteError) value |= 1U << 2U;
    value |= static_cast<uint8_t>((status.used_file_flags & 0x07U) << 3U);
    if (status.active_file_index >= 0 && status.active_file_index < 3) {
        value |= static_cast<uint8_t>((status.active_file_index + 1) << 6U);
    }
    return value;
}

void quantizeInertial(
    const Sensor::AccelerometerData& acceleration,
    bool acceleration_valid,
    const Sensor::GyroscopeData& gyroscope,
    bool gyroscope_valid,
    int16_t output[6])
{
    if (acceleration_valid) {
        output[0] = i16(acceleration.x_m_s2 * INVERSE_STANDARD_GRAVITY * 1000.0f);
        output[1] = i16(acceleration.y_m_s2 * INVERSE_STANDARD_GRAVITY * 1000.0f);
        output[2] = i16(acceleration.z_m_s2 * INVERSE_STANDARD_GRAVITY * 1000.0f);
    }
    if (gyroscope_valid) {
        output[3] = i16(gyroscope.x_rad_s * 1000.0f);
        output[4] = i16(gyroscope.y_rad_s * 1000.0f);
        output[5] = i16(gyroscope.z_rad_s * 1000.0f);
    }
}

void quantizeMagnetic(
    const Sensor::MagneticData& input,
    bool valid,
    int16_t output[3])
{
    if (!valid) return;
    output[0] = i16(input.x_uT * 10.0f);
    output[1] = i16(input.y_uT * 10.0f);
    output[2] = i16(input.z_uT * 10.0f);
}

int8_t i8(float value)
{
    if (!isfinite(value)) return 0;
    if (value > 127.0f) return 127;
    if (value < -128.0f) return -128;
    return static_cast<int8_t>(lroundf(value));
}

void quantizeTweSensors(
    const Sensor::AccelerometerData& acceleration,
    bool acceleration_valid,
    const Sensor::GyroscopeData& gyroscope,
    bool gyroscope_valid,
    const Sensor::MagneticData& magnetic,
    bool magnetic_valid,
    int8_t output[9])
{
    if (acceleration_valid) {
        output[0] = i8(acceleration.x_m_s2 * INVERSE_STANDARD_GRAVITY * 20.0f);
        output[1] = i8(acceleration.y_m_s2 * INVERSE_STANDARD_GRAVITY * 20.0f);
        output[2] = i8(acceleration.z_m_s2 * INVERSE_STANDARD_GRAVITY * 20.0f);
    }
    if (gyroscope_valid) {
        output[3] = i8(gyroscope.x_rad_s * 3.0f);
        output[4] = i8(gyroscope.y_rad_s * 3.0f);
        output[5] = i8(gyroscope.z_rad_s * 3.0f);
    }
    if (magnetic_valid) {
        output[6] = i8(magnetic.x_uT / 16.0f);
        output[7] = i8(magnetic.y_uT / 16.0f);
        output[8] = i8(magnetic.z_uT / 16.0f);
    }
}

} // namespace

void taskLog(void* pvParameters)
{
    (void)pvParameters;
    uint32_t message_number = 0;
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        xTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LOG_PERIOD_MS));
        const uint32_t now = millis();
        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

        SystemData system{};
        if (xQueuePeek(mbx_system_data, &system, 0) != pdTRUE ||
            system.boot_mode == BootMode::DEBUG) continue;

        Coordinate coordinate{};
        Gps::NavPvtObservation gps_data{};
        Sensor::AccelerometerData acceleration{};
        Sensor::AccelerometerData board_acceleration{};
        Sensor::AccelerometerData can_acceleration{};
        Sensor::GyroscopeData gyroscope{};
        Sensor::GyroscopeData board_gyroscope{};
        Sensor::GyroscopeData can_gyroscope{};
        Sensor::MagneticData magnetic{}, board_magnetic{}, can_magnetic{};
        Sensor::PressureData pressure{};
        Sensor::AcquisitionStats acquisition_stats{};
        Can::Data::Encoder encoder{};
        JogData jog{};
        Rasp::CameraData camera{};
        StuckStatus stuck{};
        StuckDiagnostics stuck_diagnostics{};
        FlashStatus flash_status{};

        const bool has_coordinate = xQueuePeek(mbx_coordinate, &coordinate, 0) == pdTRUE;
        const bool has_gps = xQueuePeek(mbx_gps_nav_pvt_observation, &gps_data, 0) == pdTRUE;
        const bool has_acceleration =
            xQueuePeek(mbx_acceleration, &acceleration, 0) == pdTRUE;
        const bool has_board_acceleration = xQueuePeek(
            mbx_board_acceleration, &board_acceleration, 0) == pdTRUE;
        const bool has_can_acceleration = xQueuePeek(
            mbx_can_acceleration, &can_acceleration, 0) == pdTRUE;
        const bool has_gyroscope =
            xQueuePeek(mbx_gyroscope, &gyroscope, 0) == pdTRUE;
        const bool has_board_gyroscope = xQueuePeek(
            mbx_board_gyroscope, &board_gyroscope, 0) == pdTRUE;
        const bool has_can_gyroscope = xQueuePeek(
            mbx_can_gyroscope, &can_gyroscope, 0) == pdTRUE;
        const bool has_magnetic = xQueuePeek(mbx_magnetic, &magnetic, 0) == pdTRUE;
        const bool has_board_magnetic =
            xQueuePeek(mbx_board_magnetic, &board_magnetic, 0) == pdTRUE;
        const bool has_can_magnetic =
            xQueuePeek(mbx_can_magnetic, &can_magnetic, 0) == pdTRUE;
        const bool has_pressure = xQueuePeek(mbx_pressure, &pressure, 0) == pdTRUE;
        const bool has_acquisition_stats =
            xQueuePeek(
                mbx_sensor_acquisition_stats,
                &acquisition_stats,
                0) == pdTRUE;
        const bool has_encoder = xQueuePeek(mbx_can_encoder, &encoder, 0) == pdTRUE;
        const bool has_jog = xQueuePeek(mbx_can_jog_cmd, &jog, 0) == pdTRUE;
        const bool has_camera = xQueuePeek(mbx_camera_data, &camera, 0) == pdTRUE;
        const bool has_stuck = xQueuePeek(mbx_stuck_status, &stuck, 0) == pdTRUE;
        const bool has_stuck_diagnostics =
            xQueuePeek(mbx_stuck_diagnostics, &stuck_diagnostics, 0) == pdTRUE;
        const bool has_flash = xQueuePeek(mbx_flash_status, &flash_status, 0) == pdTRUE;

        const bool gps_valid = has_gps && Sensor::sampleIsFresh(
            gps_data.metadata, now_us, GPS_TIMEOUT_MS * 1000ULL);
        const bool coordinate_valid = has_coordinate &&
            fresh(now, coordinate.timestamp_ms, COORDINATE_TIMEOUT_MS) &&
            (coordinate.localization_status_flags & Domain::Localization::STATUS_POSITION_USABLE) != 0U;
        const bool acceleration_valid = has_acceleration && Sensor::sampleIsFresh(
            acceleration.metadata, now_us, IMU_TIMEOUT_MS * 1000ULL);
        const bool board_acceleration_valid = has_board_acceleration &&
            Sensor::sampleIsFresh(
                board_acceleration.metadata, now_us, IMU_TIMEOUT_MS * 1000ULL);
        const bool can_acceleration_valid = has_can_acceleration &&
            Sensor::sampleIsFresh(
                can_acceleration.metadata, now_us, IMU_TIMEOUT_MS * 1000ULL);
        const bool gyroscope_valid = has_gyroscope && Sensor::sampleIsFresh(
            gyroscope.metadata, now_us, IMU_TIMEOUT_MS * 1000ULL);
        const bool board_gyroscope_valid = has_board_gyroscope &&
            Sensor::sampleIsFresh(
                board_gyroscope.metadata, now_us, IMU_TIMEOUT_MS * 1000ULL);
        const bool can_gyroscope_valid = has_can_gyroscope &&
            Sensor::sampleIsFresh(
                can_gyroscope.metadata, now_us, IMU_TIMEOUT_MS * 1000ULL);
        const bool magnetic_valid = has_magnetic && Sensor::sampleIsFresh(
            magnetic.metadata, now_us, MAGNETIC_TIMEOUT_MS * 1000ULL);
        const bool board_magnetic_valid = has_board_magnetic &&
            Sensor::sampleIsFresh(
                board_magnetic.metadata, now_us, MAGNETIC_TIMEOUT_MS * 1000ULL);
        const bool can_magnetic_valid = has_can_magnetic && Sensor::sampleIsFresh(
            can_magnetic.metadata, now_us, MAGNETIC_TIMEOUT_MS * 1000ULL);
        const bool pressure_valid = has_pressure && Sensor::sampleIsFresh(
            pressure.metadata, now_us, PRESSURE_TIMEOUT_MS * 1000ULL);
        const bool encoder_valid = has_encoder && Sensor::sampleIsFresh(
            encoder.metadata, now_us, ENCODER_TIMEOUT_MS * 1000ULL);
        const bool camera_valid =
            has_camera && fresh(now, camera.received_ms, CAMERA_TIMEOUT_MS);
        const bool jog_valid = has_jog &&
            static_cast<uint32_t>(now - jog.timestamp_ms) < jog.duration_ms;
        const bool stuck_diagnostics_valid = has_stuck_diagnostics &&
            fresh(now, stuck_diagnostics.timestamp_ms, STUCK_TIMEOUT_MS);
        const auto rasp_status = rasp.getStatus();

        uint16_t valid_flags = 0;
        if (gps_valid) valid_flags |= VALID_GPS;
        if (board_acceleration_valid || board_gyroscope_valid) {
            valid_flags |= VALID_BOARD_IMU;
        }
        if (can_acceleration_valid || can_gyroscope_valid) {
            valid_flags |= VALID_CAN_IMU;
        }
        if (board_magnetic_valid) valid_flags |= VALID_BOARD_MAGNETIC;
        if (can_magnetic_valid) valid_flags |= VALID_CAN_MAGNETIC;
        if (pressure_valid) valid_flags |= VALID_PRESSURE;
        if (encoder_valid) valid_flags |= VALID_ENCODER;
        if (coordinate_valid) valid_flags |= VALID_COORDINATE;
        if (camera_valid) valid_flags |= VALID_CAMERA;
        if (jog_valid) valid_flags |= VALID_JOG;
        if (rasp_status.heartbeat_alive) valid_flags |= VALID_RASP_HEARTBEAT;
        if (system.gps_localization_enabled) {
            valid_flags |= GPS_LOCALIZATION_ENABLED;
        }
        if (acceleration_valid || gyroscope_valid) {
            valid_flags |= VALID_SELECTED_IMU;
        }
        if (magnetic_valid) valid_flags |= VALID_SELECTED_MAGNETIC;
        if (stuck_diagnostics_valid) valid_flags |= VALID_STUCK_DIAGNOSTICS;
        if (has_flash && flash_status.initialized) valid_flags |= VALID_FLASH;

        const uint8_t camera_flags =
            (camera_valid ? 1U : 0U) |
            (camera_valid && camera.frame.target_found ? 2U : 0U);
        const uint8_t sources = sensorSources(
            acceleration_valid ? acceleration.metadata.source : Sensor::Source::None,
            gyroscope_valid ? gyroscope.metadata.source : Sensor::Source::None,
            magnetic_valid ? magnetic.metadata.source : Sensor::Source::None,
            pressure_valid ? pressure.metadata.source : Sensor::Source::None);
        const uint8_t flash_state = flashState(has_flash, flash_status);
        const uint8_t verification_result = stuck_diagnostics_valid
            ? stuck_diagnostics.verification_result : 0U;
        const uint8_t stuck_reason = has_stuck
            ? static_cast<uint8_t>(stuck.reason) : 0U;
        const uint16_t gps_hacc = gps_valid
            ? u16(gps_data.horizontal_accuracy_mm) : UINT16_MAX;
        const uint16_t pressure_div10 = pressure_valid && pressure.pressure_pa > 0
            ? u16(static_cast<uint32_t>(pressure.pressure_pa) / 10U) : 0U;

        int16_t board_imu_raw[6]{}, can_imu_raw[6]{};
        int16_t board_magnetic_raw[3]{}, can_magnetic_raw[3]{};
        quantizeInertial(
            board_acceleration,
            board_acceleration_valid,
            board_gyroscope,
            board_gyroscope_valid,
            board_imu_raw);
        quantizeInertial(
            can_acceleration,
            can_acceleration_valid,
            can_gyroscope,
            can_gyroscope_valid,
            can_imu_raw);
        quantizeMagnetic(board_magnetic, board_magnetic_valid, board_magnetic_raw);
        quantizeMagnetic(can_magnetic, can_magnetic_valid, can_magnetic_raw);
        int8_t board_twe_sensor[9]{}, can_twe_sensor[9]{};
        quantizeTweSensors(
            board_acceleration,
            board_acceleration_valid,
            board_gyroscope,
            board_gyroscope_valid,
            board_magnetic,
            board_magnetic_valid,
            board_twe_sensor);
        quantizeTweSensors(
            can_acceleration,
            can_acceleration_valid,
            can_gyroscope,
            can_gyroscope_valid,
            can_magnetic,
            can_magnetic_valid,
            can_twe_sensor);

        if (++message_number == 0U) message_number = 1U;
        Flash::LogFrame log{};
        log.format_version = Flash::LOG_FORMAT_VERSION;
        log.flash_file_index = has_flash ? flash_status.active_file_index : -1;
        log.mission_state = static_cast<uint8_t>(system.state);
        log.boot_mode = static_cast<uint8_t>(system.boot_mode);
        log.message_number = message_number;
        log.timestamp_ms = now;
        log.valid_flags = valid_flags;
        log.localization_status_flags = has_coordinate
            ? coordinate.localization_status_flags : 0U;
        log.x_mm = has_coordinate ? coordinate.x_mm : 0;
        log.y_mm = has_coordinate ? coordinate.y_mm : 0;
        log.yaw_deg_1e2 = has_coordinate ? yawDeg1e2(coordinate.heading_rad) : 0U;
        log.forward_velocity_mm_s =
            has_coordinate ? i16(coordinate.forward_velocity_mm_s) : 0;
        log.lat_1e7 = gps_valid ? gps_data.latitude_e7 : 0;
        log.lng_1e7 = gps_valid ? gps_data.longitude_e7 : 0;
        log.gps_fix_type = gps_valid ? gps_data.fix_type : 0U;
        log.gps_satellites = gps_valid ? gps_data.satellites : 0U;
        log.gps_horizontal_accuracy_mm = gps_hacc;
        log.sensor_sources = sources;
        log.camera_flags = camera_flags;
        log.camera_angle_error_deg10 = camera_valid ? camera.frame.angle_error_deg10 : 0;
        log.camera_occupancy_permille = camera_valid ? camera.frame.occupancy_permille : 0U;
        log.camera_confidence = camera_valid ? camera.frame.confidence : 0U;
        log.stuck_reason = stuck_reason;
        log.stuck_verification_result = verification_result;
        log.rasp_state = static_cast<uint8_t>(rasp.getState());
        log.gps_state = static_cast<uint8_t>(gps.getStatus());
        log.flash_state = flash_state;
        log.board_acc_x_mg = board_imu_raw[0];
        log.board_acc_y_mg = board_imu_raw[1];
        log.board_acc_z_mg = board_imu_raw[2];
        log.board_gyro_x_rad_s_x1000 = board_imu_raw[3];
        log.board_gyro_y_rad_s_x1000 = board_imu_raw[4];
        log.board_gyro_z_rad_s_x1000 = board_imu_raw[5];
        log.can_acc_x_mg = can_imu_raw[0];
        log.can_acc_y_mg = can_imu_raw[1];
        log.can_acc_z_mg = can_imu_raw[2];
        log.can_gyro_x_rad_s_x1000 = can_imu_raw[3];
        log.can_gyro_y_rad_s_x1000 = can_imu_raw[4];
        log.can_gyro_z_rad_s_x1000 = can_imu_raw[5];
        log.board_magnetic_x_uT_x10 = board_magnetic_raw[0];
        log.board_magnetic_y_uT_x10 = board_magnetic_raw[1];
        log.board_magnetic_z_uT_x10 = board_magnetic_raw[2];
        log.can_magnetic_x_uT_x10 = can_magnetic_raw[0];
        log.can_magnetic_y_uT_x10 = can_magnetic_raw[1];
        log.can_magnetic_z_uT_x10 = can_magnetic_raw[2];
        log.pressure_pa_div10 = pressure_div10;
        log.encoder_left_mm = encoder_valid ? encoder.left_mm : 0;
        log.encoder_right_mm = encoder_valid ? encoder.right_mm : 0;
        log.camera_scene_hash = camera_valid ? camera.frame.scene_hash : 0U;
        log.gyro_samples_100ms = has_acquisition_stats
            ? acquisition_stats.gyro_samples : 0U;
        log.accel_samples_100ms = has_acquisition_stats
            ? acquisition_stats.accel_samples : 0U;
        log.magnetic_samples_100ms = has_acquisition_stats
            ? acquisition_stats.magnetic_samples : 0U;
        log.imu_fifo_overflow_count = has_acquisition_stats
            ? acquisition_stats.fifo_overflow_count : 0U;
        xQueueOverwrite(mbx_flash_log, &log);

        Twe::TelemetryFrame telemetry{};
        telemetry.protocol_version = Twe::TELEMETRY_PROTOCOL_VERSION;
        telemetry.message_number = static_cast<uint16_t>(message_number);
        if (telemetry.message_number == 0U) telemetry.message_number = 1U;
        telemetry.timestamp_ms = now;
        telemetry.mission_state = log.mission_state;
        telemetry.boot_mode = log.boot_mode;
        telemetry.valid_flags = valid_flags;
        telemetry.localization_status_flags = log.localization_status_flags;
        telemetry.x_mm = log.x_mm;
        telemetry.y_mm = log.y_mm;
        telemetry.yaw_deg_1e2 = log.yaw_deg_1e2;
        telemetry.forward_velocity_mm_s = log.forward_velocity_mm_s;
        telemetry.lat_1e7 = log.lat_1e7;
        telemetry.lng_1e7 = log.lng_1e7;
        telemetry.gps_fix_type = log.gps_fix_type;
        telemetry.gps_satellites = log.gps_satellites;
        telemetry.gps_horizontal_accuracy_mm = gps_hacc;
        telemetry.sensor_sources = sources;
        telemetry.camera_flags = camera_flags;
        telemetry.camera_angle_error_deg10 = log.camera_angle_error_deg10;
        telemetry.camera_occupancy_permille = log.camera_occupancy_permille;
        telemetry.camera_confidence = log.camera_confidence;
        telemetry.stuck_reason = stuck_reason;
        telemetry.stuck_verification_result = verification_result;
        telemetry.rasp_state = log.rasp_state;
        telemetry.gps_state = log.gps_state;
        telemetry.flash_state = flash_state;
        telemetry.board_acc_x_g_x20 = board_twe_sensor[0];
        telemetry.board_acc_y_g_x20 = board_twe_sensor[1];
        telemetry.board_acc_z_g_x20 = board_twe_sensor[2];
        telemetry.board_gyro_x_rad_s_x3 = board_twe_sensor[3];
        telemetry.board_gyro_y_rad_s_x3 = board_twe_sensor[4];
        telemetry.board_gyro_z_rad_s_x3 = board_twe_sensor[5];
        telemetry.board_magnetic_x_uT_div16 = board_twe_sensor[6];
        telemetry.board_magnetic_y_uT_div16 = board_twe_sensor[7];
        telemetry.board_magnetic_z_uT_div16 = board_twe_sensor[8];
        telemetry.can_acc_x_g_x20 = can_twe_sensor[0];
        telemetry.can_acc_y_g_x20 = can_twe_sensor[1];
        telemetry.can_acc_z_g_x20 = can_twe_sensor[2];
        telemetry.can_gyro_x_rad_s_x3 = can_twe_sensor[3];
        telemetry.can_gyro_y_rad_s_x3 = can_twe_sensor[4];
        telemetry.can_gyro_z_rad_s_x3 = can_twe_sensor[5];
        telemetry.can_magnetic_x_uT_div16 = can_twe_sensor[6];
        telemetry.can_magnetic_y_uT_div16 = can_twe_sensor[7];
        telemetry.can_magnetic_z_uT_div16 = can_twe_sensor[8];
        telemetry.pressure_pa_div10 = pressure_div10;
        telemetry.encoder_left_mm = log.encoder_left_mm;
        telemetry.encoder_right_mm = log.encoder_right_mm;
        xQueueOverwrite(mbx_twe_telemetry, &telemetry);
    }
}
