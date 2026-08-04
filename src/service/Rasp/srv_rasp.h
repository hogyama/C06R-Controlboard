#pragma once

#include <Arduino.h>
#include "driver/uart.h"

namespace Rasp
{
    // COBS decoding result received from Raspberry Pi.
    struct __attribute__((packed)) Frame
    {
        uint8_t magic;
        uint32_t msg_number;
        uint8_t target_found;
        int16_t angle_error_deg10;
        uint16_t occupancy_permille;
        uint8_t confidence;
        uint64_t scene_hash;
        uint8_t checksum;
    };

    static_assert(
        sizeof(Frame) == 20,
        "Rasp::Frame must be 20 bytes"
    );

    // Raspberry Piから受信したフレームにESP32側の受信時刻を付加する。
    struct CameraData
    {
        Frame frame;
        // ESP32がこの応答を要求してREQをHIGHにした時刻。
        uint32_t requested_ms;
        uint32_t received_ms;
    };
}

class SrvRasp
{
public:
    // Simplified Raspberry Pi operating state.
    enum class RaspState : uint8_t
    {
        NotInitialized,
        PowerOff,
        Starting,
        Requesting,
        Ready
    };

    // Snapshot of Raspberry Pi power and communication state.
    struct RaspStatus
    {
        bool initialized;
        bool power_enabled;
        bool camera_ready;
        bool camera_reception_started;
        bool waiting_response;
        bool link_valid;
        uint32_t last_frame_age_ms;
    };

    // 設定を保存し、UARTは開始せずRaspberry Pi電源をOFFにする。
    bool init(
        int8_t uart_rx_pin,
        int8_t request_pin,
        int8_t camera_ready_pin,
        int8_t rasp_en_pin,
        uart_port_t uart_port,
        uint32_t baud_rate = 1200
    );

    // Advances UART reception, requests, and shutdown processing.
    void update();

    // Copies the latest valid frame; returns false before first reception.
    bool getLatestFrame(
        Rasp::Frame& frame,
        uint32_t& requested_ms) const;

    // Returns true while READY and recent UART data are both valid.
    bool isCameraLinkValid() const;

    // Returns a snapshot of all externally useful state.
    RaspStatus getStatus() const;

    // Returns the current simplified Raspberry Pi state.
    RaspState getState() const;

    // Starts periodic camera-frame requests after initial readiness.
    bool startCameraReception();

    // Stops periodic requests without powering the Raspberry Pi off.
    void stopCameraReception();

    // Powers the Pi on and resets communication state.
    bool powerOn();

    // Stops requests and directly removes Raspberry Pi power.
    bool powerOff();

private:
    static constexpr uint8_t MAGIC = 0x43;
    static constexpr size_t RX_BUFFER_SIZE = 64;
    // 応答完了後100ms待って次を要求する。要求の重複はwaiting_response_で防ぐ。
    static constexpr uint32_t REQUEST_INTERVAL_MS = 100;
    // REQからCOBSフレーム受信完了までを待つ上限。通常の1200baud転送より十分長い。
    static constexpr uint32_t RESPONSE_TIMEOUT_MS = 400;
    static constexpr uint32_t COMM_TIMEOUT_MS = 2000;

    bool startUart();
    void stopUart();

    // Reads UART bytes and completes one 0x00-delimited COBS frame.
    void pollUart();

    // COBS-decodes one packet and validates its fields and checksum.
    bool decodeAndValidate(
        const uint8_t* input,
        size_t length,
        Rasp::Frame& output
    ) const;

    uart_port_t uart_port_ = UART_NUM_MAX;
    int8_t uart_rx_pin_ = -1;
    uint32_t baud_rate_ = 0;
    int8_t request_pin_ = -1;
    int8_t camera_ready_pin_ = -1;
    int8_t rasp_en_pin_ = -1;

    uint8_t rx_buffer_[RX_BUFFER_SIZE]{};
    size_t rx_size_ = 0;
    Rasp::Frame latest_frame_{};

    uint32_t last_request_ms_ = 0;
    uint32_t last_frame_ms_ = 0;
    uint32_t active_request_started_ms_ = 0;
    uint32_t last_frame_request_started_ms_ = 0;

    bool initialized_ = false;
    bool uart_active_ = false;
    bool waiting_response_ = false;
    bool have_frame_ = false;
    bool camera_reception_started_ = false;
    bool power_enabled_ = false;
};
