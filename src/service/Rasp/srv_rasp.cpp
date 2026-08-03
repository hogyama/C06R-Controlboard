#include "srv_rasp.h"

#include <cstring>

// 設定値とGPIOだけを初期化し、UART driverはpowerOn()まで作らない。
bool SrvRasp::init(
    int8_t uart_rx_pin,
    int8_t request_pin,
    int8_t camera_ready_pin,
    int8_t rasp_en_pin,
    uart_port_t uart_port,
    uint32_t baud_rate
)
{
    if (initialized_) {
        return true;
    }

    if (
        uart_rx_pin < 0 ||
        request_pin < 0 ||
        camera_ready_pin < 0 ||
        rasp_en_pin < 0 ||
        static_cast<int>(uart_port) < 0 ||
        uart_port >= UART_NUM_MAX ||
        baud_rate == 0
    ) {
        return false;
    }

    request_pin_ = request_pin;
    camera_ready_pin_ = camera_ready_pin;
    rasp_en_pin_ = rasp_en_pin;
    uart_port_ = uart_port;
    uart_rx_pin_ = uart_rx_pin;
    baud_rate_ = baud_rate;

    pinMode(request_pin_, OUTPUT);
    pinMode(rasp_en_pin_, OUTPUT);
    pinMode(camera_ready_pin_, INPUT_PULLDOWN);

    digitalWrite(request_pin_, LOW);
    digitalWrite(rasp_en_pin_, LOW);
    pinMode(uart_rx_pin_, INPUT);

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
        uart_set_pin(
            uart_port_,
            UART_PIN_NO_CHANGE,
            uart_rx_pin_,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
        ) != ESP_OK) {
        pinMode(uart_rx_pin_, INPUT);
        return false;
    }

    const esp_err_t install_result = uart_driver_install(
        uart_port_,
        1024,
        0,
        0,
        nullptr,
        0
    );

    if (install_result != ESP_OK) {
        pinMode(uart_rx_pin_, INPUT);
        return false;
    }

    uart_active_ = true;
    return true;
}

void SrvRasp::stopUart()
{
    if (uart_active_) {
        uart_driver_delete(uart_port_);
        uart_active_ = false;
    }
    if (uart_rx_pin_ >= 0) {
        pinMode(uart_rx_pin_, INPUT);
    }
}

// Advances UART request and reception while Raspberry Pi power is enabled.
void SrvRasp::update()
{
    if (!initialized_ || !power_enabled_ || !uart_active_) {
        return;
    }

    const uint32_t now = millis();

    pollUart();

    const bool camera_ready =
        digitalRead(camera_ready_pin_) == HIGH;

    if (!camera_ready) {
        have_frame_ = false;
        camera_reception_started_ = false;
    }

    if (waiting_response_) {
        if (
            !camera_ready ||
            static_cast<uint32_t>(now - last_request_ms_) >=
                RESPONSE_TIMEOUT_MS
        ) {
            digitalWrite(request_pin_, LOW);
            waiting_response_ = false;
            rx_size_ = 0;
            active_request_started_ms_ = 0;
            last_request_ms_ = now;
        }

        return;
    }

    if (
        camera_ready &&
        (!have_frame_ || camera_reception_started_) &&
        static_cast<uint32_t>(now - last_request_ms_) >=
            REQUEST_INTERVAL_MS
    ) {
        uart_flush_input(uart_port_);
        rx_size_ = 0;
        digitalWrite(request_pin_, HIGH);
        waiting_response_ = true;
        active_request_started_ms_ = now;
        last_request_ms_ = now;
    }
}

// Copies the most recently validated camera frame.
bool SrvRasp::getLatestFrame(
    Rasp::Frame& frame,
    uint32_t& requested_ms) const
{
    if (!have_frame_) {
        return false;
    }

    frame = latest_frame_;
    requested_ms = last_frame_request_started_ms_;
    return true;
}

// Checks Pi power, READY level, and age of the latest frame.
bool SrvRasp::isCameraLinkValid() const
{
    const bool frame_is_current =
        !camera_reception_started_ ||
        static_cast<uint32_t>(millis() - last_frame_ms_) <
            COMM_TIMEOUT_MS;

    return
        initialized_ &&
        uart_active_ &&
        power_enabled_ &&
        have_frame_ &&
        digitalRead(camera_ready_pin_) == HIGH &&
        frame_is_current;
}

// Builds a status snapshot without adding duplicated state variables.
SrvRasp::RaspStatus SrvRasp::getStatus() const
{
    RaspStatus status{};
    status.initialized = initialized_;
    status.power_enabled = power_enabled_;
    status.camera_ready =
        initialized_ &&
        digitalRead(camera_ready_pin_) == HIGH;
    status.camera_reception_started =
        camera_reception_started_;
    status.waiting_response = waiting_response_;
    status.link_valid = isCameraLinkValid();
    status.last_frame_age_ms = have_frame_
        ? static_cast<uint32_t>(millis() - last_frame_ms_)
        : UINT32_MAX;
    return status;
}

// Derives one simple state from the existing power and link flags.
SrvRasp::RaspState SrvRasp::getState() const
{
    if (!initialized_) {
        return RaspState::NotInitialized;
    }

    if (!power_enabled_) {
        return RaspState::PowerOff;
    }

    if (waiting_response_) {
        return RaspState::Requesting;
    }

    if (isCameraLinkValid()) {
        return RaspState::Ready;
    }

    return RaspState::Starting;
}

// Enables periodic requests and immediately starts the first one.
bool SrvRasp::startCameraReception()
{
    if (
        !initialized_ ||
        !uart_active_ ||
        !power_enabled_ ||
        !have_frame_ ||
        digitalRead(camera_ready_pin_) != HIGH
    ) {
        return false;
    }

    if (camera_reception_started_) {
        return true;
    }

    uart_flush_input(uart_port_);
    rx_size_ = 0;
    digitalWrite(request_pin_, HIGH);
    waiting_response_ = true;
    camera_reception_started_ = true;
    active_request_started_ms_ = millis();
    last_request_ms_ = active_request_started_ms_;
    return true;
}

// Powers the Pi on and clears the previous communication session.
bool SrvRasp::powerOn()
{
    if (!initialized_) {
        return false;
    }

    if (power_enabled_) {
        return true;
    }

    rx_size_ = 0;
    latest_frame_ = {};
    waiting_response_ = false;
    have_frame_ = false;
    camera_reception_started_ = false;
    last_frame_ms_ = 0;
    active_request_started_ms_ = 0;
    last_frame_request_started_ms_ = 0;
    last_request_ms_ = millis() - REQUEST_INTERVAL_MS;

    // 周辺基板へ先に給電し、その後でUART driverをinstallする。
    digitalWrite(rasp_en_pin_, HIGH);
    if (!startUart()) {
        stopUart();
        digitalWrite(rasp_en_pin_, LOW);
        return false;
    }
    uart_flush_input(uart_port_);
    power_enabled_ = true;
    return true;
}

// Stops communication and directly removes power (safe with read-only root).
bool SrvRasp::powerOff()
{
    if (!initialized_) {
        return false;
    }

    digitalWrite(request_pin_, LOW);
    waiting_response_ = false;
    active_request_started_ms_ = 0;
    camera_reception_started_ = false;
    power_enabled_ = false;
    have_frame_ = false;
    rx_size_ = 0;
    stopUart();
    digitalWrite(rasp_en_pin_, LOW);
    return true;
}

// Buffers UART bytes until 0x00, then validates one COBS frame.
void SrvRasp::pollUart()
{
    if (!uart_active_) return;

    uint8_t bytes[64];

    while (true) {
        const int count = uart_read_bytes(
            uart_port_,
            bytes,
            sizeof(bytes),
            0
        );

        if (count <= 0) {
            return;
        }

        for (int i = 0; i < count; ++i) {
            if (!waiting_response_) {
                continue;
            }

            const uint8_t byte = bytes[i];

            if (byte == 0x00) {
                Rasp::Frame frame{};
                const bool valid = decodeAndValidate(
                    rx_buffer_,
                    rx_size_,
                    frame
                );

                rx_size_ = 0;
                digitalWrite(request_pin_, LOW);
                waiting_response_ = false;
                last_request_ms_ = millis();

                if (valid) {
                    latest_frame_ = frame;
                    last_frame_request_started_ms_ =
                        active_request_started_ms_;
                    last_frame_ms_ = millis();
                    have_frame_ = true;
                }
                active_request_started_ms_ = 0;

                continue;
            }

            if (rx_size_ >= RX_BUFFER_SIZE) {
                rx_size_ = 0;
                digitalWrite(request_pin_, LOW);
                waiting_response_ = false;
                active_request_started_ms_ = 0;
                last_request_ms_ = millis();
                continue;
            }

            rx_buffer_[rx_size_++] = byte;
        }
    }
}

// Decodes COBS and checks the 12-byte camera-result packet.
bool SrvRasp::decodeAndValidate(
    const uint8_t* input,
    size_t length,
    Rasp::Frame& output
) const
{
    if (input == nullptr || length == 0) {
        return false;
    }

    uint8_t decoded[sizeof(Rasp::Frame)]{};
    size_t read_index = 0;
    size_t write_index = 0;

    while (read_index < length) {
        const uint8_t code = input[read_index++];

        if (code == 0) {
            return false;
        }

        const size_t block_size = code - 1U;

        if (
            read_index + block_size > length ||
            write_index + block_size > sizeof(decoded)
        ) {
            return false;
        }

        for (size_t i = 0; i < block_size; ++i) {
            decoded[write_index++] = input[read_index++];
        }

        if (code != 0xFF && read_index < length) {
            if (write_index >= sizeof(decoded)) {
                return false;
            }

            decoded[write_index++] = 0x00;
        }
    }

    if (write_index != sizeof(Rasp::Frame)) {
        return false;
    }

    Rasp::Frame frame{};
    memcpy(&frame, decoded, sizeof(frame));

    if (
        frame.magic != MAGIC ||
        frame.target_found > 1U ||
        frame.angle_error_deg10 < -1800 ||
        frame.angle_error_deg10 > 1800 ||
        frame.occupancy_permille > 1000U
    ) {
        return false;
    }

    uint8_t checksum = 0;
    for (size_t i = 0; i < sizeof(decoded) - 1U; ++i) {
        checksum ^= decoded[i];
    }

    if (frame.checksum != checksum) {
        return false;
    }

    output = frame;
    return true;
}
