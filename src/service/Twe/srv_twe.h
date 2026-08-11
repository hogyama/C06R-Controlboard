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

constexpr uint8_t TELEMETRY_PROTOCOL_VERSION = 9;

// One complete live snapshot. Fixed-point raw values keep it in one packet.
struct TelemetryFrame {
    uint8_t protocol_version;
    uint16_t message_number;
    uint32_t timestamp_ms;
    uint8_t mission_state;
    uint8_t boot_mode;
    uint16_t valid_flags;
    uint16_t localization_status_flags;
    int32_t x_mm;
    int32_t y_mm;
    uint16_t yaw_deg_1e2;
    int16_t forward_velocity_mm_s;
    int32_t lat_1e7;
    int32_t lng_1e7;
    uint8_t gps_fix_type;
    uint8_t gps_satellites;
    uint16_t gps_horizontal_accuracy_mm;
    uint8_t sensor_sources;
    uint8_t camera_flags;
    int16_t camera_angle_error_deg10;
    uint16_t camera_occupancy_permille;
    uint8_t camera_confidence;
    uint8_t stuck_reason;
    uint8_t stuck_verification_result;
    uint8_t rasp_state;
    uint8_t gps_state;
    uint8_t flash_state;
    int8_t board_acc_x_g_x20;
    int8_t board_acc_y_g_x20;
    int8_t board_acc_z_g_x20;
    int8_t board_gyro_x_rad_s_x3;
    int8_t board_gyro_y_rad_s_x3;
    int8_t board_gyro_z_rad_s_x3;
    int8_t board_magnetic_x_uT_div16;
    int8_t board_magnetic_y_uT_div16;
    int8_t board_magnetic_z_uT_div16;
    int8_t can_acc_x_g_x20;
    int8_t can_acc_y_g_x20;
    int8_t can_acc_z_g_x20;
    int8_t can_gyro_x_rad_s_x3;
    int8_t can_gyro_y_rad_s_x3;
    int8_t can_gyro_z_rad_s_x3;
    int8_t can_magnetic_x_uT_div16;
    int8_t can_magnetic_y_uT_div16;
    int8_t can_magnetic_z_uT_div16;
    uint16_t pressure_pa_div10;
    int32_t encoder_left_mm;
    int32_t encoder_right_mm;
} __attribute__((packed));

} // namespace Twe

static_assert(sizeof(Twe::TelemetryFrame) == 77, "Twe telemetry size mismatch");
static_assert(sizeof(Twe::TelemetryFrame) + 2 <= HAL_TWE_USER_DATA_MAX_LEN,
              "Twe telemetry exceeds payload");

class SrvTwe {
public:
    SrvTwe();
    ~SrvTwe();
    bool init(int8_t esp_tx_pin, int8_t esp_rx_pin, int8_t twe_en_pin,
              uart_port_t uart_port, uint32_t baud = 115200);
    void end();
    bool powerOn();
    void powerOff();
    bool sendTelemetry(const Twe::TelemetryFrame& frame);
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
