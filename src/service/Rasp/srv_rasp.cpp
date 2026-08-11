#include "srv_rasp.h"
#include <cstring>

bool SrvRasp::init(int8_t rx, int8_t tx, int8_t heartbeat, int8_t ready,
    int8_t en, uart_port_t port, uint32_t baud)
{
    if (initialized_) return true;
    if (rx < 0 || tx < 0 || heartbeat < 0 || ready < 0 || en < 0 ||
        port < 0 || port >= UART_NUM_MAX || baud == 0) return false;
    uart_rx_pin_ = rx; uart_tx_pin_ = tx; heartbeat_pin_ = heartbeat;
    camera_ready_pin_ = ready; rasp_en_pin_ = en; uart_port_ = port; baud_rate_ = baud;
    pinMode(rasp_en_pin_, OUTPUT);
    pinMode(heartbeat_pin_, INPUT_PULLDOWN);
    pinMode(camera_ready_pin_, INPUT_PULLDOWN);
    pinMode(uart_rx_pin_, INPUT);
    pinMode(uart_tx_pin_, INPUT);
    digitalWrite(rasp_en_pin_, LOW);
    initialized_ = true;
    return true;
}

bool SrvRasp::startUart()
{
    if (!initialized_) return false;
    if (uart_active_) return true;
    uart_config_t config{};
    config.baud_rate = static_cast<int>(baud_rate_);
    config.data_bits = UART_DATA_8_BITS;
    config.parity = UART_PARITY_DISABLE;
    config.stop_bits = UART_STOP_BITS_1;
    config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    if (uart_param_config(uart_port_, &config) != ESP_OK ||
        uart_set_pin(uart_port_, uart_tx_pin_, uart_rx_pin_,
            UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) return false;
    if (uart_driver_install(uart_port_, 1024, 0, 0, nullptr, 0) != ESP_OK) return false;
    uart_active_ = true;
    return true;
}

void SrvRasp::stopUart()
{
    if (uart_active_) uart_driver_delete(uart_port_);
    uart_active_ = false;
    pinMode(uart_rx_pin_, INPUT);
    pinMode(uart_tx_pin_, INPUT);
}

bool SrvRasp::sendCommand(Rasp::CommandType type, const void* payload,
    uint8_t payload_length)
{
    if (!uart_active_ || !power_enabled_) return false;
    if (payload_length > sizeof(Rasp::CameraParameters) ||
        (payload_length != 0U && payload == nullptr)) return false;
    uint8_t raw[sizeof(Rasp::CommandHeader) + sizeof(Rasp::CameraParameters) + 1]{};
    Rasp::CommandHeader header{};
    header.magic = COMMAND_MAGIC;
    header.command = static_cast<uint8_t>(type);
    header.sequence = ++command_sequence_;
    header.payload_length = payload_length;
    memcpy(raw, &header, sizeof(header));
    if (payload_length != 0U) memcpy(raw + sizeof(header), payload, payload_length);
    const size_t raw_size = sizeof(header) + payload_length + 1U;
    for (size_t i = 0; i < raw_size - 1U; ++i) raw[raw_size - 1U] ^= raw[i];
    uint8_t encoded[sizeof(raw) + 2]{};
    size_t read = 0, write = 1, code_index = 0;
    uint8_t code = 1;
    while (read < raw_size) {
        if (raw[read] == 0) {
            encoded[code_index] = code; code = 1; code_index = write++;
        } else {
            encoded[write++] = raw[read];
            if (++code == 0xFF) { encoded[code_index] = code; code = 1; code_index = write++; }
        }
        ++read;
    }
    encoded[code_index] = code;
    encoded[write++] = 0;
    return uart_write_bytes(uart_port_, encoded, write) == static_cast<int>(write);
}

void SrvRasp::update()
{
    if (!initialized_ || !power_enabled_ || !uart_active_) return;
    const uint32_t now = millis();
    const int level = digitalRead(heartbeat_pin_);
    if (level != heartbeat_level_) {
        heartbeat_level_ = level;
        heartbeat_seen_ = true;
        last_heartbeat_ms_ = now;
    }
    pollUart();
    if (digitalRead(camera_ready_pin_) != HIGH) {
        have_frame_ = false;
    }
    if (waiting_response_) {
        if (now - last_request_ms_ >= RESPONSE_TIMEOUT_MS) {
            waiting_response_ = false; rx_size_ = 0; active_request_started_ms_ = 0;
            last_request_ms_ = now;
        }
        return;
    }
    if (camera_reception_started_ && digitalRead(camera_ready_pin_) == HIGH &&
        now - last_request_ms_ >= REQUEST_INTERVAL_MS &&
        sendCommand(Rasp::CommandType::Request)) {
        uart_flush_input(uart_port_); rx_size_ = 0; waiting_response_ = true;
        active_request_started_ms_ = now; last_request_ms_ = now;
    }
}

bool SrvRasp::getLatestFrame(Rasp::Frame& frame, uint32_t& requested_ms) const
{
    if (!have_frame_) return false;
    frame = latest_frame_; requested_ms = last_frame_request_started_ms_; return true;
}

SrvRasp::RaspStatus SrvRasp::getStatus() const
{
    RaspStatus s{}; const uint32_t now = millis();
    s.initialized = initialized_; s.power_enabled = power_enabled_;
    s.heartbeat_alive = heartbeat_seen_ && now - last_heartbeat_ms_ < HEARTBEAT_TIMEOUT_MS;
    s.camera_ready = initialized_ && digitalRead(camera_ready_pin_) == HIGH;
    s.camera_reception_started = camera_reception_started_;
    s.waiting_response = waiting_response_; s.link_valid = isCameraLinkValid();
    s.last_heartbeat_age_ms = heartbeat_seen_ ? now - last_heartbeat_ms_ : UINT32_MAX;
    s.last_frame_age_ms = have_frame_ ? now - last_frame_ms_ : UINT32_MAX;
    return s;
}

bool SrvRasp::isCameraLinkValid() const
{
    return initialized_ && uart_active_ && power_enabled_ && heartbeat_seen_ &&
        millis() - last_heartbeat_ms_ < HEARTBEAT_TIMEOUT_MS && have_frame_ &&
        digitalRead(camera_ready_pin_) == HIGH && millis() - last_frame_ms_ < COMM_TIMEOUT_MS;
}

SrvRasp::RaspState SrvRasp::getState() const
{
    if (!initialized_) return RaspState::NotInitialized;
    if (!power_enabled_) return RaspState::PowerOff;
    if (waiting_response_) return RaspState::Requesting;
    return isCameraLinkValid() ? RaspState::Ready : RaspState::Starting;
}

bool SrvRasp::startCameraReception(const Rasp::CameraParameters& parameters)
{
    if (!initialized_ || !uart_active_ || !power_enabled_) return false;
    camera_parameters_ = parameters;
    image_mode_ = parameters.image_mode != 0U;
    if (!camera_reception_started_) {
        if (!sendCommand(Rasp::CommandType::Start, &camera_parameters_,
                sizeof(camera_parameters_))) return false;
        camera_reception_started_ = true;
        last_request_ms_ = millis();
    }
    return true;
}

void SrvRasp::stopCameraReception()
{
    if (uart_active_ && power_enabled_ && camera_reception_started_)
        sendCommand(Rasp::CommandType::Stop);
    waiting_response_ = false; camera_reception_started_ = false;
    active_request_started_ms_ = 0; rx_size_ = 0;
    if (uart_active_) uart_flush_input(uart_port_);
}

bool SrvRasp::powerOn()
{
    if (!initialized_) return false;
    if (power_enabled_) return true;
    rx_size_ = 0; latest_frame_ = {}; waiting_response_ = false; have_frame_ = false;
    heartbeat_seen_ = false; camera_reception_started_ = false; last_frame_ms_ = 0;
    active_request_started_ms_ = 0; last_frame_request_started_ms_ = 0;
    last_request_ms_ = millis() - REQUEST_INTERVAL_MS;
    heartbeat_level_ = digitalRead(heartbeat_pin_);
    digitalWrite(rasp_en_pin_, HIGH);
    if (!startUart()) { stopUart(); digitalWrite(rasp_en_pin_, LOW); return false; }
    uart_flush_input(uart_port_); power_enabled_ = true; return true;
}

bool SrvRasp::powerOff()
{
    if (!initialized_) return false;
    waiting_response_ = false; camera_reception_started_ = false; power_enabled_ = false;
    have_frame_ = false; heartbeat_seen_ = false; rx_size_ = 0;
    stopUart(); digitalWrite(rasp_en_pin_, LOW); return true;
}

void SrvRasp::pollUart()
{
    uint8_t bytes[64];
    while (true) {
        const int count = uart_read_bytes(uart_port_, bytes, sizeof(bytes), 0);
        if (count <= 0) return;
        for (int i = 0; i < count; ++i) {
            if (!waiting_response_) continue;
            const uint8_t byte = bytes[i];
            if (byte == 0) {
                Rasp::Frame frame{};
                const bool valid = decodeAndValidate(rx_buffer_, rx_size_, frame);
                rx_size_ = 0; waiting_response_ = false; last_request_ms_ = millis();
                if (valid) { latest_frame_ = frame; last_frame_request_started_ms_ = active_request_started_ms_;
                    last_frame_ms_ = millis(); have_frame_ = true; }
                active_request_started_ms_ = 0; continue;
            }
            if (rx_size_ >= RX_BUFFER_SIZE) { rx_size_ = 0; waiting_response_ = false;
                active_request_started_ms_ = 0; last_request_ms_ = millis(); continue; }
            rx_buffer_[rx_size_++] = byte;
        }
    }
}

bool SrvRasp::decodeAndValidate(const uint8_t* input, size_t length, Rasp::Frame& output) const
{
    if (!input || length == 0) return false;
    uint8_t decoded[sizeof(Rasp::Frame)]{}; size_t read = 0, write = 0;
    while (read < length) {
        const uint8_t code = input[read++]; if (code == 0) return false;
        const size_t block = code - 1U;
        if (read + block > length || write + block > sizeof(decoded)) return false;
        for (size_t i = 0; i < block; ++i) decoded[write++] = input[read++];
        if (code != 0xFF && read < length) { if (write >= sizeof(decoded)) return false; decoded[write++] = 0; }
    }
    if (write != sizeof(Rasp::Frame)) return false;
    Rasp::Frame frame{}; memcpy(&frame, decoded, sizeof(frame));
    if (frame.magic != FRAME_MAGIC || frame.target_found > 1U ||
        frame.angle_error_deg10 < -1800 || frame.angle_error_deg10 > 1800 ||
        frame.occupancy_permille > 1000U) return false;
    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(decoded) - 1U; ++i) checksum ^= decoded[i];
    if (frame.checksum != checksum) return false;
    output = frame; return true;
}
