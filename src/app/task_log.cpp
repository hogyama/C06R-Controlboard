#include "tasks.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "domain/localization/localization_types.h"
#include "domain/sensor/sensor_freshness.h"
#include "platform/field_config.h"
#include "platform/sensor_axis_transform.h"

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

enum LogValidFlag : uint32_t {
    VALID_GPS = 1U << 0,
    VALID_BOARD_ACCEL = 1U << 1,
    VALID_CAN_ACCEL = 1U << 2,
    VALID_BOARD_GYRO = 1U << 3,
    VALID_CAN_GYRO = 1U << 4,
    VALID_BOARD_MAGNETIC = 1U << 5,
    VALID_CAN_MAGNETIC = 1U << 6,
    VALID_PRESSURE = 1U << 7,
    VALID_ENCODER = 1U << 8,
    VALID_COORDINATE = 1U << 9,
    VALID_CAMERA = 1U << 10,
    VALID_JOG = 1U << 11,
    VALID_RASP_HEARTBEAT = 1U << 12,
    VALID_FLASH = 1U << 13,
    VALID_SELECTED_ACCEL = 1U << 14,
    VALID_SELECTED_GYRO = 1U << 15,
    VALID_SELECTED_MAGNETIC = 1U << 16,
    VALID_SELECTED_PRESSURE = 1U << 17,
    VALID_STUCK_DIAGNOSTICS = 1U << 18,
    GPS_LOCALIZATION_ENABLED = 1U << 19,
    VALID_GYRO_INTERVAL_STATS = 1U << 20,
    VALID_NAVIGATION_PROGRESS = 1U << 21,
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

uint8_t u8(uint32_t value)
{
    return value > UINT8_MAX ? UINT8_MAX : static_cast<uint8_t>(value);
}

int32_t i32(float value)
{
    if (!isfinite(value)) return 0;
    if (value > static_cast<float>(INT32_MAX)) return INT32_MAX;
    if (value < static_cast<float>(INT32_MIN)) return INT32_MIN;
    return static_cast<int32_t>(lroundf(value));
}

uint16_t sampleAgeMs(
    const Sensor::SampleMetadata& metadata, uint64_t now_us)
{
    if (metadata.received_us == 0U || now_us < metadata.received_us) {
        return UINT16_MAX;
    }
    return u16(static_cast<uint32_t>(
        (now_us - metadata.received_us) / 1000ULL));
}

uint8_t age100ms(uint16_t age_ms)
{
    if (age_ms == UINT16_MAX) return UINT8_MAX;
    return u8(age_ms / 100U);
}

uint8_t age100msNibble(uint16_t age_ms)
{
    if (age_ms == UINT16_MAX) return 0x0FU;
    const uint16_t units = age_ms / 100U;
    return static_cast<uint8_t>(units > 14U ? 14U : units);
}

int8_t correctedMagneticUT(float value, bool valid)
{
    if (!valid || !isfinite(value)) return Twe::INVALID_CORRECTED_MAGNETIC_UT;
    if (value > 127.0f) return 127;
    if (value < -127.0f) return -127;
    return static_cast<int8_t>(lroundf(value));
}

uint8_t magneticYawU8(float radians, bool valid)
{
    if (!valid || !isfinite(radians)) return Twe::INVALID_MAGNETIC_YAW;
    constexpr float MAGNETIC_TWO_PI = 2.0f * static_cast<float>(M_PI);
    float angle = fmodf(radians, MAGNETIC_TWO_PI);
    if (angle < 0.0f) angle += MAGNETIC_TWO_PI;
    const uint16_t encoded = static_cast<uint16_t>(lroundf(
        angle * 255.0f / MAGNETIC_TWO_PI));
    return encoded >= 255U ? 0U : static_cast<uint8_t>(encoded);
}

uint8_t packHealth(const Coordinate& coordinate, bool available)
{
    if (!available) return 0;
    return (static_cast<uint8_t>(coordinate.gps_health) & 0x03U) |
        ((static_cast<uint8_t>(coordinate.encoder_health) & 0x03U) << 2U) |
        ((static_cast<uint8_t>(coordinate.imu_health) & 0x03U) << 4U) |
        ((static_cast<uint8_t>(coordinate.magnetic_health) & 0x03U) << 6U);
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

uint8_t flashFlags(bool available, const FlashStatus& status)
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

int8_t activeFlashFile(bool available, const FlashStatus& status)
{
    if (!available || !status.initialized) return -2;
    if (status.storage_full) return -1;
    return status.active_file_index >= 0 && status.active_file_index < 3
        ? status.active_file_index : -2;
}

void quantizeInertial(
    const Sensor::AccelerometerData& acceleration,
    bool acceleration_valid,
    const Sensor::GyroscopeData& gyroscope,
    bool gyroscope_valid,
    int16_t output[6])
{
    if (acceleration_valid) {
        const auto body = SensorAxisTransform::imuToBody(
            acceleration.metadata.source,
            acceleration.x_m_s2,
            acceleration.y_m_s2,
            acceleration.z_m_s2);
        output[0] = i16(body.x * INVERSE_STANDARD_GRAVITY * 1000.0f);
        output[1] = i16(body.y * INVERSE_STANDARD_GRAVITY * 1000.0f);
        output[2] = i16(body.z * INVERSE_STANDARD_GRAVITY * 1000.0f);
    }
    if (gyroscope_valid) {
        const auto body = SensorAxisTransform::imuToBody(
            gyroscope.metadata.source,
            gyroscope.x_rad_s,
            gyroscope.y_rad_s,
            gyroscope.z_rad_s);
        output[3] = i16(body.x * 1000.0f);
        output[4] = i16(body.y * 1000.0f);
        output[5] = i16(body.z * 1000.0f);
    }
}

void quantizeMagnetic(
    const Sensor::MagneticData& input,
    bool valid,
    int16_t output[3])
{
    if (!valid) return;
    const auto body = SensorAxisTransform::magneticToBody(
        input.metadata.source, input.x_uT, input.y_uT, input.z_uT);
    output[0] = i16(body.x * 10.0f);
    output[1] = i16(body.y * 10.0f);
    output[2] = i16(body.z * 10.0f);
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
        NavigationProgress navigation_progress{};
        LocalizationDebugStatus localization_debug{};
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
        const bool has_navigation_progress = xQueuePeek(
            mbx_navigation_progress, &navigation_progress, 0) == pdTRUE;
        const bool has_localization_debug = xQueuePeek(
            mbx_localization_debug_status, &localization_debug, 0) == pdTRUE;
        const bool has_flash = xQueuePeek(mbx_flash_status, &flash_status, 0) == pdTRUE;
        const bool magnetic_diagnostic_fresh = has_localization_debug &&
            localization_debug.timestamp_us != 0U &&
            now_us >= localization_debug.timestamp_us &&
            now_us - localization_debug.timestamp_us <= 250000ULL;

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
        const bool navigation_progress_valid = has_navigation_progress &&
            fresh(now, navigation_progress.timestamp_ms, COORDINATE_TIMEOUT_MS);
        const bool acquisition_stats_valid = has_acquisition_stats &&
            Sensor::sampleIsFresh(
                acquisition_stats.metadata, now_us, 250000ULL);
        const auto rasp_status = rasp.getStatus();

        uint32_t valid_flags = 0;
        if (gps_valid) valid_flags |= VALID_GPS;
        if (board_acceleration_valid) valid_flags |= VALID_BOARD_ACCEL;
        if (can_acceleration_valid) valid_flags |= VALID_CAN_ACCEL;
        if (board_gyroscope_valid) valid_flags |= VALID_BOARD_GYRO;
        if (can_gyroscope_valid) valid_flags |= VALID_CAN_GYRO;
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
        if (acceleration_valid) valid_flags |= VALID_SELECTED_ACCEL;
        if (gyroscope_valid) valid_flags |= VALID_SELECTED_GYRO;
        if (magnetic_valid) valid_flags |= VALID_SELECTED_MAGNETIC;
        if (pressure_valid) valid_flags |= VALID_SELECTED_PRESSURE;
        if (stuck_diagnostics_valid) valid_flags |= VALID_STUCK_DIAGNOSTICS;
        if (has_flash && flash_status.initialized) valid_flags |= VALID_FLASH;
        if (acquisition_stats_valid) valid_flags |= VALID_GYRO_INTERVAL_STATS;
        if (navigation_progress_valid) valid_flags |= VALID_NAVIGATION_PROGRESS;

        const uint8_t camera_flags =
            (camera_valid ? 1U : 0U) |
            (camera_valid && camera.frame.target_found ? 2U : 0U);
        const uint8_t sources = sensorSources(
            acceleration_valid ? acceleration.metadata.source : Sensor::Source::None,
            gyroscope_valid ? gyroscope.metadata.source : Sensor::Source::None,
            magnetic_valid ? magnetic.metadata.source : Sensor::Source::None,
            pressure_valid ? pressure.metadata.source : Sensor::Source::None);
        const uint8_t flash_flags = flashFlags(has_flash, flash_status);
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
        if (++message_number == 0U) message_number = 1U;
        Flash::LogFrame log{};
        log.format_version = Flash::LOG_FORMAT_VERSION;
        log.mission_state = static_cast<uint8_t>(system.state);
        log.boot_mode = static_cast<uint8_t>(system.boot_mode);
        log.localization_quality = has_coordinate
            ? static_cast<uint8_t>(coordinate.localization_quality) : 0U;
        log.message_number = message_number;
        log.timestamp_ms = now;
        log.valid_flags = valid_flags;
        log.localization_status_flags = has_coordinate
            ? coordinate.localization_status_flags : 0U;
        log.health_states = packHealth(coordinate, has_coordinate);
        log.sensor_sources = sources;
        log.nav_hold_reason = jog_valid
            ? static_cast<uint8_t>(jog.nav_hold_reason) : 0U;
        log.recovery_phase = jog_valid
            ? static_cast<uint8_t>(jog.recovery_phase) : 0U;
        log.path_mode = navigation_progress_valid
            ? static_cast<uint8_t>(navigation_progress.path_mode) : 0U;
        log.rasp_state = static_cast<uint8_t>(rasp.getState());
        log.gps_state = static_cast<uint8_t>(gps.getStatus());
        log.stuck_reason = stuck_reason;
        log.stuck_verification_result = verification_result;
        log.stuck_verification_phase = stuck_diagnostics_valid
            ? static_cast<uint8_t>(stuck_diagnostics.verification_phase) : 0U;
        log.camera_flags = camera_flags;
        log.camera_confidence = camera_valid ? camera.frame.confidence : 0U;
        log.target_path_index = navigation_progress_valid
            ? navigation_progress.target_index : UINT16_MAX;
        log.gps_warp_count = has_coordinate ? coordinate.gps_warp_count : 0U;
        log.x_mm = has_coordinate ? coordinate.x_mm : 0;
        log.y_mm = has_coordinate ? coordinate.y_mm : 0;
        log.yaw_deg_1e2 = has_coordinate ? yawDeg1e2(coordinate.heading_rad) : 0U;
        log.forward_velocity_mm_s =
            has_coordinate ? i16(coordinate.forward_velocity_mm_s) : 0;
        log.yaw_rate_rad_s_x1000 = has_coordinate
            ? i16(coordinate.yaw_rate_rad_s * 1000.0f) : 0;
        log.position_std_mm = has_coordinate
            ? u16(coordinate.position_std_mm) : UINT16_MAX;
        log.yaw_std_mrad = has_coordinate
            ? u16(static_cast<uint32_t>(fmaxf(
                coordinate.yaw_std_rad * 1000.0f, 0.0f))) : UINT16_MAX;
        log.lat_1e7 = gps_valid ? gps_data.latitude_e7 : 0;
        log.lng_1e7 = gps_valid ? gps_data.longitude_e7 : 0;
        log.gps_fix_type = gps_valid ? gps_data.fix_type : 0U;
        log.gps_satellites = gps_valid ? gps_data.satellites : 0U;
        log.gps_horizontal_accuracy_mm = gps_hacc;
        log.camera_angle_error_deg10 = camera_valid ? camera.frame.angle_error_deg10 : 0;
        log.camera_occupancy_permille = camera_valid ? camera.frame.occupancy_permille : 0U;
        log.camera_message_number = camera_valid ? camera.frame.msg_number : 0U;
        log.camera_age_ms = camera_valid
            ? u16(static_cast<uint32_t>(now - camera.received_ms)) : UINT16_MAX;
        log.camera_scene_hash = camera_valid ? camera.frame.scene_hash : 0U;
        log.jog_velocity_mm_s = jog_valid ? i16(jog.velocity_mm_s) : 0;
        log.jog_omega_rad_s_x100 = jog_valid
            ? i16(jog.omega_rad_s * 100.0f) : 0;
        log.jog_remaining_ms = jog_valid
            ? u16(jog.duration_ms - static_cast<uint32_t>(now - jog.timestamp_ms))
            : 0U;
        log.jog_source = jog_valid ? static_cast<uint8_t>(jog.source) : 0U;
        log.jog_before_scale_mm_s = jog_valid ? jog.jog_before_scale_mm_s : 0;
        log.jog_after_scale_mm_s = jog_valid ? jog.jog_after_scale_mm_s : 0;
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
        log.board_accel_age_ms = has_board_acceleration
            ? sampleAgeMs(board_acceleration.metadata, now_us) : UINT16_MAX;
        log.board_gyro_age_ms = has_board_gyroscope
            ? sampleAgeMs(board_gyroscope.metadata, now_us) : UINT16_MAX;
        log.board_magnetic_age_ms = has_board_magnetic
            ? sampleAgeMs(board_magnetic.metadata, now_us) : UINT16_MAX;
        log.can_accel_age_ms = has_can_acceleration
            ? sampleAgeMs(can_acceleration.metadata, now_us) : UINT16_MAX;
        log.can_gyro_age_ms = has_can_gyroscope
            ? sampleAgeMs(can_gyroscope.metadata, now_us) : UINT16_MAX;
        log.can_magnetic_age_ms = has_can_magnetic
            ? sampleAgeMs(can_magnetic.metadata, now_us) : UINT16_MAX;
        log.pressure_age_ms = has_pressure
            ? sampleAgeMs(pressure.metadata, now_us) : UINT16_MAX;
        log.encoder_age_ms = has_encoder
            ? sampleAgeMs(encoder.metadata, now_us) : UINT16_MAX;
        log.gps_age_ms = has_gps
            ? sampleAgeMs(gps_data.metadata, now_us) : UINT16_MAX;
        log.coordinate_age_ms = has_coordinate
            ? u16(static_cast<uint32_t>(now - coordinate.timestamp_ms))
            : UINT16_MAX;
        if (gyroscope_valid) {
            const auto selected_body = SensorAxisTransform::imuToBody(
                gyroscope.metadata.source,
                gyroscope.x_rad_s,
                gyroscope.y_rad_s,
                gyroscope.z_rad_s);
            log.latest_gyro_x_rad_s_x1000 = i16(selected_body.x * 1000.0f);
            log.latest_gyro_y_rad_s_x1000 = i16(selected_body.y * 1000.0f);
            log.latest_gyro_z_rad_s_x1000 = i16(selected_body.z * 1000.0f);
        }
        log.gyro_integrated_z_urad_100ms = acquisition_stats_valid
            ? i32(acquisition_stats.integrated_gyro_z_rad * 1000000.0f) : 0;
        log.gyro_min_z_rad_s_x1000 = acquisition_stats_valid
            ? i16(acquisition_stats.minimum_gyro_z_rad_s * 1000.0f) : 0;
        log.gyro_max_z_rad_s_x1000 = acquisition_stats_valid
            ? i16(acquisition_stats.maximum_gyro_z_rad_s * 1000.0f) : 0;
        log.gyro_samples_100ms = acquisition_stats_valid
            ? acquisition_stats.gyro_samples : 0U;
        log.accel_samples_100ms = acquisition_stats_valid
            ? acquisition_stats.accel_samples : 0U;
        log.magnetic_samples_100ms = acquisition_stats_valid
            ? acquisition_stats.magnetic_samples : 0U;
        log.imu_fifo_overflow_count = acquisition_stats_valid
            ? acquisition_stats.fifo_overflow_count : 0U;
        log.gps_power_cycle_count = u16(gps.powerCycleCount());
        log.gps_configuration_retry_count = u16(gps.configurationRetryCount());
        log.gps_checksum_failure_count = u16(gps.uartChecksumFailureCount());
        log.stuck_hash_before = stuck_diagnostics_valid
            ? stuck_diagnostics.hash_before : 0U;
        log.stuck_hash_after = stuck_diagnostics_valid
            ? stuck_diagnostics.hash_after : 0U;
        log.stuck_hash_distance_bits = stuck_diagnostics_valid
            ? stuck_diagnostics.hash_distance_bits : UINT8_MAX;
        log.gps_warp_east_mm = has_coordinate
            ? i16(static_cast<float>(coordinate.gps_warp_east_mm)) : 0;
        log.gps_warp_north_mm = has_coordinate
            ? i16(static_cast<float>(coordinate.gps_warp_north_mm)) : 0;
        log.goal_latitude_e7 = FieldConfig::GOAL_LATITUDE_E7;
        log.goal_longitude_e7 = FieldConfig::GOAL_LONGITUDE_E7;
        log.goal_x_mm = FieldConfig::GOAL_X_MM;
        log.goal_y_mm = FieldConfig::GOAL_Y_MM;
        log.field_size_x_mm = FieldConfig::SIZE_X_MM;
        log.field_size_y_mm = FieldConfig::SIZE_Y_MM;
        xQueueOverwrite(mbx_flash_log, &log);

        Twe::TelemetrySnapshot telemetry{};
        auto& page0 = telemetry.navigation;
        page0.protocol_version = Twe::TELEMETRY_PROTOCOL_VERSION;
        page0.page_id = static_cast<uint8_t>(Twe::TelemetryPageId::Navigation);
        page0.message_number = static_cast<uint16_t>(message_number);
        if (page0.message_number == 0U) page0.message_number = 1U;
        page0.timestamp_ms = now;
        page0.mission_state = log.mission_state;
        page0.boot_mode = log.boot_mode;
        page0.valid_flags = log.valid_flags;
        page0.localization_status_flags = log.localization_status_flags;
        page0.localization_quality = log.localization_quality;
        page0.health_states = log.health_states;
        page0.sensor_sources = log.sensor_sources;
        page0.x_mm = log.x_mm;
        page0.y_mm = log.y_mm;
        page0.yaw_deg_1e2 = log.yaw_deg_1e2;
        page0.forward_velocity_mm_s = log.forward_velocity_mm_s;
        page0.position_std_mm = log.position_std_mm;
        page0.yaw_std_mrad = log.yaw_std_mrad;
        page0.lat_1e7 = log.lat_1e7;
        page0.lng_1e7 = log.lng_1e7;
        page0.gps_fix_type = log.gps_fix_type;
        page0.gps_satellites = log.gps_satellites;
        page0.gps_horizontal_accuracy_mm = log.gps_horizontal_accuracy_mm;
        page0.camera_flags = log.camera_flags;
        page0.camera_angle_error_deg10 = log.camera_angle_error_deg10;
        page0.camera_occupancy_permille = log.camera_occupancy_permille;
        page0.camera_confidence = log.camera_confidence;
        page0.stuck_reason = log.stuck_reason;
        page0.stuck_verification_result = log.stuck_verification_result;
        page0.stuck_verification_phase = log.stuck_verification_phase;
        page0.nav_hold_reason = log.nav_hold_reason;
        page0.recovery_phase = log.recovery_phase;
        page0.path_mode = log.path_mode;
        page0.target_path_index = log.target_path_index;
        page0.gps_warp_count = log.gps_warp_count;
        page0.rasp_state = log.rasp_state;
        page0.gps_state = log.gps_state;
        page0.flash_file_index = activeFlashFile(has_flash, flash_status);
        page0.flash_flags = flash_flags;
        page0.jog_velocity_mm_s = log.jog_velocity_mm_s;
        page0.jog_omega_rad_s_x100 = log.jog_omega_rad_s_x100;
        page0.jog_remaining_ms = log.jog_remaining_ms;
        page0.jog_source = log.jog_source;
        page0.board_magnetic_yaw_u8 = magneticYawU8(
            localization_debug.board_magnetic_yaw_rad,
            magnetic_diagnostic_fresh &&
                localization_debug.board_magnetic_yaw_valid);
        page0.can_magnetic_yaw_u8 = magneticYawU8(
            localization_debug.can_magnetic_yaw_rad,
            magnetic_diagnostic_fresh &&
                localization_debug.can_magnetic_yaw_valid);

        auto& page1 = telemetry.sensors;
        page1.protocol_version = Twe::TELEMETRY_PROTOCOL_VERSION;
        page1.page_id = static_cast<uint8_t>(Twe::TelemetryPageId::Sensors);
        page1.message_number = page0.message_number;
        page1.timestamp_ms = page0.timestamp_ms;
        page1.sensor_valid_flags = static_cast<uint16_t>(
            valid_flags & 0x000001FEUL);
        page1.board_acc_x_mg = log.board_acc_x_mg;
        page1.board_acc_y_mg = log.board_acc_y_mg;
        page1.board_acc_z_mg = log.board_acc_z_mg;
        page1.board_gyro_x_rad_s_x1000 = log.board_gyro_x_rad_s_x1000;
        page1.board_gyro_y_rad_s_x1000 = log.board_gyro_y_rad_s_x1000;
        page1.board_gyro_z_rad_s_x1000 = log.board_gyro_z_rad_s_x1000;
        page1.board_magnetic_x_uT_x10 = log.board_magnetic_x_uT_x10;
        page1.board_magnetic_y_uT_x10 = log.board_magnetic_y_uT_x10;
        page1.board_magnetic_z_uT_x10 = log.board_magnetic_z_uT_x10;
        page1.board_corrected_magnetic_x_uT = correctedMagneticUT(
            localization_debug.board_corrected_magnetic_uT[0],
            magnetic_diagnostic_fresh &&
                localization_debug.board_corrected_magnetic_valid);
        page1.board_corrected_magnetic_y_uT = correctedMagneticUT(
            localization_debug.board_corrected_magnetic_uT[1],
            magnetic_diagnostic_fresh &&
                localization_debug.board_corrected_magnetic_valid);
        page1.board_corrected_magnetic_z_uT = correctedMagneticUT(
            localization_debug.board_corrected_magnetic_uT[2],
            magnetic_diagnostic_fresh &&
                localization_debug.board_corrected_magnetic_valid);
        page1.can_acc_x_mg = log.can_acc_x_mg;
        page1.can_acc_y_mg = log.can_acc_y_mg;
        page1.can_acc_z_mg = log.can_acc_z_mg;
        page1.can_gyro_x_rad_s_x1000 = log.can_gyro_x_rad_s_x1000;
        page1.can_gyro_y_rad_s_x1000 = log.can_gyro_y_rad_s_x1000;
        page1.can_gyro_z_rad_s_x1000 = log.can_gyro_z_rad_s_x1000;
        page1.can_magnetic_x_uT_x10 = log.can_magnetic_x_uT_x10;
        page1.can_magnetic_y_uT_x10 = log.can_magnetic_y_uT_x10;
        page1.can_magnetic_z_uT_x10 = log.can_magnetic_z_uT_x10;
        page1.pressure_pa_div10 = log.pressure_pa_div10;
        page1.encoder_left_mm = log.encoder_left_mm;
        page1.encoder_right_mm = log.encoder_right_mm;
        page1.gyro_integrated_z_rad_x10000 = i16(
            static_cast<float>(log.gyro_integrated_z_urad_100ms) / 100.0f);
        page1.gyro_min_z_rad_s_x1000 = log.gyro_min_z_rad_s_x1000;
        page1.gyro_max_z_rad_s_x1000 = log.gyro_max_z_rad_s_x1000;
        page1.gyro_samples_100ms = u8(log.gyro_samples_100ms);
        page1.accel_samples_100ms = log.accel_samples_100ms;
        page1.magnetic_samples_100ms = log.magnetic_samples_100ms;
        page1.imu_fifo_overflow_count = u8(log.imu_fifo_overflow_count);
        page1.gps_recovery_counts = static_cast<uint8_t>(
            (log.gps_power_cycle_count > 15U ? 15U : log.gps_power_cycle_count) |
            ((log.gps_configuration_retry_count > 15U
                ? 15U : log.gps_configuration_retry_count) << 4U));
        page1.board_accel_age_100ms = age100ms(log.board_accel_age_ms);
        page1.board_gyro_age_100ms = age100ms(log.board_gyro_age_ms);
        page1.can_accel_age_100ms = age100ms(log.can_accel_age_ms);
        page1.can_gyro_age_100ms = age100ms(log.can_gyro_age_ms);
        page1.magnetic_ages_100ms = static_cast<uint8_t>(
            age100msNibble(log.board_magnetic_age_ms) |
            (age100msNibble(log.can_magnetic_age_ms) << 4U));
        page1.pressure_age_100ms = age100ms(log.pressure_age_ms);
        page1.gps_age_100ms = age100ms(log.gps_age_ms);
        page1.encoder_age_100ms = age100ms(log.encoder_age_ms);
        xQueueOverwrite(mbx_twe_telemetry, &telemetry);
    }
}
