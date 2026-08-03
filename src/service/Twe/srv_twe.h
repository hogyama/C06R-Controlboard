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
    CanStart
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

constexpr uint8_t TELEMETRY_PROTOCOL_VERSION = 4;

enum class TelemetryPage : uint8_t {
    Navigation = 0,
    Sensors = 1,
    Health = 2,
    Gps = 3,
    Count
};

// 共通ヘッダ12 byteと詳細40 byte。packedなのでPC側がpaddingに依存しない。
struct TelemetryFrame {
    uint8_t protocol_version;
    TelemetryPage page;
    uint16_t message_number;
    uint32_t timestamp_ms;
    uint8_t mission_state;
    uint8_t boot_mode;
    uint16_t valid_flags;

    union {
        struct {
            int32_t lat_1e7;
            int32_t lng_1e7;
            int32_t gps_x_mm;
            int32_t gps_y_mm;
            int32_t x_mm;
            int32_t y_mm;
            uint16_t yaw_deg_1e2;
            int16_t forward_velocity_mm_s;
            int16_t yaw_rate_rad_s_x1000;
            uint32_t position_std_mm;
            uint16_t yaw_std_mrad;
            int16_t jog_velocity_mm_s;
            int16_t jog_omega_rad_s_x100;
        } __attribute__((packed)) navigation;
        struct {
            int16_t acc_x_mg;
            int16_t acc_y_mg;
            int16_t acc_z_mg;
            int32_t pressure_pa;
            int32_t encoder_left_mm;
            int32_t encoder_right_mm;
            int16_t gyro_z_rad_s_x1000;
            uint16_t magnetic_yaw_deg_1e2;
            uint8_t gps_fix_type;
            uint8_t gps_satellites;
            uint32_t gps_horizontal_accuracy_mm;
            int32_t gps_velocity_east_mm_s;
            int32_t gps_velocity_north_mm_s;
            uint32_t gps_speed_accuracy_mm_s;
        } __attribute__((packed)) sensors;
        struct {
            int8_t flash_file_index;
            uint8_t flash_used_flags;
            uint8_t flash_storage_full;
            uint8_t rasp_state;
            uint8_t gps_state;
            uint8_t camera_valid;
            uint8_t camera_target_found;
            uint8_t camera_confidence;
            uint16_t camera_occupancy_permille;
            int16_t camera_angle_error_deg10;
            uint8_t stuck_reason;
            uint8_t stuck_cell_x;
            uint8_t stuck_cell_y;
            uint8_t attitude;
            uint16_t fusion_status_flags;
            uint16_t imu_age_ms;
            uint16_t magnetic_age_ms;
            uint16_t encoder_age_ms;
            uint16_t gps_age_ms;
            uint16_t jog_remain_ms;
            uint32_t grid_map_update_count;
            uint8_t fusion_quality;
            uint8_t gps_health;
            uint8_t encoder_health;
            uint8_t imu_health;
            uint8_t magnetic_health;
            uint16_t motion_anomaly_flags;
            uint8_t motion_anomaly_age_100ms;
        } __attribute__((packed)) health;
        struct {
            int32_t nmea_lat_1e7;
            int32_t nmea_lng_1e7;
            uint16_t nmea_sentence_age_ms;
            uint16_t nmea_location_age_ms;
            uint16_t nmea_satellites_age_ms;
            uint16_t nmea_hdop_x100;
            uint8_t nmea_satellites;
            uint8_t nmea_flags;
            int32_t nav_pvt_lat_1e7;
            int32_t nav_pvt_lng_1e7;
            uint32_t nav_pvt_hacc_mm;
            uint16_t nav_pvt_measurement_age_ms;
            uint16_t nav_pvt_receive_age_ms;
            uint8_t nav_pvt_fix_type;
            uint8_t nav_pvt_satellites;
            uint8_t nav_pvt_flags;
            uint8_t reserved[3];
        } __attribute__((packed)) gps;
        uint8_t raw[40];
    } data;
} __attribute__((packed));

} // namespace Twe

static_assert(sizeof(Twe::TelemetryFrame) == 52, "Twe telemetry size mismatch");
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
