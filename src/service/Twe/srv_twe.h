#pragma once

#include "hal_twe.h"

constexpr uint8_t TWE_MSG_TYPE_LEN = 1;
constexpr uint8_t TWE_DATA_LEN = 6;
constexpr uint8_t TWE_PACKET_LEN = TWE_MSG_TYPE_LEN + TWE_DATA_LEN + 1;
constexpr uint8_t TWE_BUFFER_SIZE = 8;

namespace Twe {

enum class MessageType : uint8_t {
    None = 0x00,
    Telemetry = 0x10,
    Command = 0x20,
    Jog = 0x21,
};

enum class CommandType : uint8_t {
    None = 0,
    ServoLock,
    ServoUnlock,
    StartSequence,
    Reset,
    InjectAscent,
    InjectLanding,
    InjectSeparationFinished,
    StartGpsNav,
    StartCameraNav,
    StartEscape,
    NotifyStuck,
    NotifyGoal,
    NotifySeparation,

    // 状態機械を変更せず、モーター・センサー基板へだけ通知する。
    CanReset,
    CanStart,

    // MANUAL試験用。GPS電源と受信は維持し、Localization入力だけを切り替える。
    DisableGpsLocalization,
    EnableGpsLocalization,
    MarkObstacle
};

union Data {
    struct {
        CommandType type;
        uint8_t padding[5];
    } cmd;
    struct {
        int16_t velocity_mm_s;
        int16_t omega_rad_s_x100;
        uint16_t duration_ms;
    } jog;
    uint8_t raw_payload[6];
};

constexpr uint8_t TELEMETRY_PROTOCOL_VERSION = 12;
constexpr int8_t INVALID_CORRECTED_MAGNETIC_UT = INT8_MIN;
constexpr uint8_t INVALID_MAGNETIC_YAW = UINT8_MAX;

enum class TelemetryPageId : uint8_t {
    Navigation = 0,
    Sensors = 1
};

struct TelemetryPage0 {
    uint8_t protocol_version;
    uint8_t page_id;
    uint16_t message_number;
    uint32_t timestamp_ms;
    uint8_t mission_state;
    uint8_t boot_mode;
    uint32_t valid_flags;
    uint16_t localization_status_flags;
    uint8_t localization_quality;
    uint8_t health_states;
    uint8_t sensor_sources;
    int32_t x_mm;
    int32_t y_mm;
    uint16_t yaw_deg_1e2;
    int16_t forward_velocity_mm_s;
    uint16_t position_std_mm;
    uint16_t yaw_std_mrad;
    int32_t lat_1e7;
    int32_t lng_1e7;
    uint8_t gps_fix_type;
    uint8_t gps_satellites;
    uint16_t gps_horizontal_accuracy_mm;
    uint8_t camera_flags;
    int16_t camera_angle_error_deg10;
    uint16_t camera_occupancy_permille;
    uint8_t camera_confidence;
    uint8_t stuck_reason;
    uint8_t stuck_verification_result;
    uint8_t stuck_verification_phase;
    uint8_t nav_hold_reason;
    uint8_t recovery_phase;
    uint8_t path_mode;
    uint16_t target_path_index;
    uint32_t gps_warp_count;
    uint8_t rasp_state;
    uint8_t gps_state;
    int8_t flash_file_index;
    uint8_t flash_flags;
    int16_t jog_velocity_mm_s;
    int16_t jog_omega_rad_s_x100;
    uint16_t jog_remaining_ms;
    uint8_t jog_source;
    uint8_t board_magnetic_yaw_u8;
    uint8_t can_magnetic_yaw_u8;
} __attribute__((packed));

struct TelemetryPage1 {
    uint8_t protocol_version;
    uint8_t page_id;
    uint16_t message_number;
    uint32_t timestamp_ms;
    uint16_t sensor_valid_flags;
    int16_t board_acc_x_mg;
    int16_t board_acc_y_mg;
    int16_t board_acc_z_mg;
    int16_t board_gyro_x_rad_s_x1000;
    int16_t board_gyro_y_rad_s_x1000;
    int16_t board_gyro_z_rad_s_x1000;
    int16_t board_magnetic_x_uT_x10;
    int16_t board_magnetic_y_uT_x10;
    int16_t board_magnetic_z_uT_x10;
    int8_t board_corrected_magnetic_x_uT;
    int8_t board_corrected_magnetic_y_uT;
    int8_t board_corrected_magnetic_z_uT;
    int16_t can_acc_x_mg;
    int16_t can_acc_y_mg;
    int16_t can_acc_z_mg;
    int16_t can_gyro_x_rad_s_x1000;
    int16_t can_gyro_y_rad_s_x1000;
    int16_t can_gyro_z_rad_s_x1000;
    int16_t can_magnetic_x_uT_x10;
    int16_t can_magnetic_y_uT_x10;
    int16_t can_magnetic_z_uT_x10;
    uint16_t pressure_pa_div10;
    int32_t encoder_left_mm;
    int32_t encoder_right_mm;
    int16_t gyro_integrated_z_rad_x10000;
    int16_t gyro_min_z_rad_s_x1000;
    int16_t gyro_max_z_rad_s_x1000;
    uint8_t gyro_samples_100ms;
    uint8_t accel_samples_100ms;
    uint8_t magnetic_samples_100ms;
    uint8_t imu_fifo_overflow_count;
    uint8_t gps_recovery_counts;
    uint8_t board_accel_age_100ms;
    uint8_t board_gyro_age_100ms;
    uint8_t can_accel_age_100ms;
    uint8_t can_gyro_age_100ms;
    uint8_t magnetic_ages_100ms;
    uint8_t pressure_age_100ms;
    uint8_t gps_age_100ms;
    uint8_t encoder_age_100ms;
} __attribute__((packed));

struct TelemetrySnapshot {
    TelemetryPage0 navigation;
    TelemetryPage1 sensors;
};

} // namespace Twe

static_assert(sizeof(Twe::TelemetryPage0) == 78, "Twe page 0 size mismatch");
static_assert(sizeof(Twe::TelemetryPage1) == 78, "Twe page 1 size mismatch");
static_assert(sizeof(Twe::TelemetryPage1) + 2 <= HAL_TWE_USER_DATA_MAX_LEN,
              "Twe telemetry page exceeds payload");

class SrvTwe {
public:
    SrvTwe();
    ~SrvTwe();
    bool init(int8_t esp_tx_pin, int8_t esp_rx_pin, int8_t twe_en_pin,
              uart_port_t uart_port, uint32_t baud = 115200);
    void end();
    bool powerOn();
    void powerOff();
    bool sendTelemetry(const Twe::TelemetryPage0& frame);
    bool sendTelemetry(const Twe::TelemetryPage1& frame);
    void poll();
    Twe::MessageType readMsg(Twe::Data* data_out);

private:
    int8_t esp_tx_pin_;
    int8_t esp_rx_pin_;
    int8_t twe_en_pin_;
    uart_port_t uart_port_;
    uint32_t baud_;
    bool initialized_;
    bool uart_active_;
    bool sendTelemetryRaw(
        const void* frame, size_t frame_size, uint16_t message_number);

    struct twe_packet_t {
        uint8_t msg_type;
        uint8_t msg_data[TWE_DATA_LEN];
    };

    HalTwe hal_twe;
    uint8_t rx_head;
    uint8_t rx_tail;
    uint8_t rx_count;
    twe_packet_t rx_buffer[TWE_BUFFER_SIZE];
};
