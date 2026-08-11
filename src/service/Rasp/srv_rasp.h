#pragma once

#include <Arduino.h>
#include "driver/uart.h"

namespace Rasp {
struct __attribute__((packed)) Frame {
    uint8_t magic;
    uint32_t msg_number;
    uint8_t target_found;
    int16_t angle_error_deg10;
    uint16_t occupancy_permille;
    uint8_t confidence;
    uint16_t capture_age_ms;
    uint64_t scene_hash;
    uint8_t checksum;
};
static_assert(sizeof(Frame) == 22, "Rasp::Frame must be 22 bytes");

enum class CommandType : uint8_t { Start = 1, Stop = 2, Request = 3 };

struct __attribute__((packed)) CommandHeader {
    uint8_t magic;
    uint8_t command;
    uint16_t sequence;
    uint8_t payload_length;
};
static_assert(sizeof(CommandHeader) == 5, "Rasp::CommandHeader must be 5 bytes");

struct __attribute__((packed)) CameraParameters {
    uint8_t version;
    uint8_t image_mode;
    uint16_t min_target_area_px;
    uint16_t confidence_full_area_px;
    uint16_t horizontal_fov_deg_x100;
    uint8_t hsv_low1[3];
    uint8_t hsv_high1[3];
    uint8_t hsv_low2[3];
    uint8_t hsv_high2[3];
    uint8_t blur_kernel_size;
    uint8_t morph_kernel_size;
    uint8_t transform_flags;
};
static_assert(sizeof(CameraParameters) == 23, "Rasp::CameraParameters must be 23 bytes");

constexpr uint8_t CAMERA_TRANSFORM_ROTATE_180 = 1U << 0;
constexpr uint8_t CAMERA_TRANSFORM_MIRROR_HORIZONTAL = 1U << 1;
constexpr uint8_t CAMERA_TRANSFORM_MASK =
    CAMERA_TRANSFORM_ROTATE_180 |
    CAMERA_TRANSFORM_MIRROR_HORIZONTAL;

struct CameraData {
    Frame frame;
    uint32_t requested_ms;
    uint32_t received_ms;
};
}

class SrvRasp {
public:
    enum class RaspState : uint8_t { NotInitialized, PowerOff, Starting, Requesting, Ready };
    struct RaspStatus {
        bool initialized;
        bool power_enabled;
        bool heartbeat_alive;
        bool camera_ready;
        bool camera_reception_started;
        bool waiting_response;
        bool link_valid;
        uint32_t last_heartbeat_age_ms;
        uint32_t last_frame_age_ms;
    };

    bool init(int8_t uart_rx_pin, int8_t uart_tx_pin,
        int8_t heartbeat_pin, int8_t camera_ready_pin, int8_t rasp_en_pin,
        uart_port_t uart_port, uint32_t baud_rate = 115200);
    void update();
    bool getLatestFrame(Rasp::Frame& frame, uint32_t& requested_ms) const;
    bool isCameraLinkValid() const;
    RaspStatus getStatus() const;
    RaspState getState() const;
    bool startCameraReception(const Rasp::CameraParameters& parameters);
    void stopCameraReception();
    bool powerOn();
    bool powerOff();

private:
    static constexpr uint8_t FRAME_MAGIC = 0x43;
    static constexpr uint8_t COMMAND_MAGIC = 0x45;
    static constexpr size_t RX_BUFFER_SIZE = 64;
    // UART transfer takes only a few milliseconds at 115200 baud.
    static constexpr uint32_t REQUEST_INTERVAL_MS = 50;
    static constexpr uint32_t RESPONSE_TIMEOUT_MS = 200;
    static constexpr uint32_t COMM_TIMEOUT_MS = 500;
    static constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 2000;

    bool startUart();
    void stopUart();
    void pollUart();
    bool sendCommand(Rasp::CommandType command, const void* payload = nullptr,
        uint8_t payload_length = 0);
    bool decodeAndValidate(const uint8_t* input, size_t length, Rasp::Frame& output) const;

    uart_port_t uart_port_ = UART_NUM_MAX;
    int8_t uart_rx_pin_ = -1;
    int8_t uart_tx_pin_ = -1;
    int8_t heartbeat_pin_ = -1;
    int8_t camera_ready_pin_ = -1;
    int8_t rasp_en_pin_ = -1;
    uint32_t baud_rate_ = 0;
    uint8_t rx_buffer_[RX_BUFFER_SIZE]{};
    size_t rx_size_ = 0;
    Rasp::Frame latest_frame_{};
    uint32_t last_request_ms_ = 0;
    uint32_t last_frame_ms_ = 0;
    uint32_t active_request_started_ms_ = 0;
    uint32_t last_frame_request_started_ms_ = 0;
    uint32_t last_heartbeat_ms_ = 0;
    int heartbeat_level_ = LOW;
    uint16_t command_sequence_ = 0;
    bool initialized_ = false;
    bool uart_active_ = false;
    bool waiting_response_ = false;
    bool have_frame_ = false;
    bool heartbeat_seen_ = false;
    bool camera_reception_started_ = false;
    bool image_mode_ = false;
    Rasp::CameraParameters camera_parameters_{};
    bool power_enabled_ = false;
};
