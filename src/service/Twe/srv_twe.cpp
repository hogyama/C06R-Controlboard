#include "srv_twe.h"
SrvTwe::SrvTwe() {
    esp_tx_pin_ = -1;
    esp_rx_pin_ = -1;
    twe_en_pin_ = -1;
    uart_port_ = UART_NUM_MAX;
    baud_ = 115200;
    initialized_ = false;
    uart_active_ = false;
    rx_head = 0;
    rx_tail = 0;
    rx_count = 0;
}

SrvTwe::~SrvTwe() {
    end();
}


bool SrvTwe::init(int8_t esp_tx_pin, int8_t esp_rx_pin, int8_t twe_en_pin, uart_port_t uart_port, uint32_t baud) {
    if (initialized_) {
        return true;
    }
    if (esp_tx_pin < 0 || esp_rx_pin < 0 || twe_en_pin < 0 ||
        uart_port < 0 || uart_port >= UART_NUM_MAX || baud == 0) {
        return false;
    }

    pinMode(twe_en_pin, OUTPUT);
    // initでは設定保存だけ行い、UART線は高インピーダンスに保つ。
    digitalWrite(twe_en_pin, HIGH);
    pinMode(esp_tx_pin, INPUT);
    pinMode(esp_rx_pin, INPUT);
    esp_tx_pin_ = esp_tx_pin;
    esp_rx_pin_ = esp_rx_pin;
    twe_en_pin_ = twe_en_pin;
    uart_port_ = uart_port;
    baud_ = baud;
    initialized_ = true;
    return true;
}

bool SrvTwe::powerOn() {
    if (!initialized_) {
        return false;
    }
    if (uart_active_) {
        return true;
    }

    // 先にTWELITEへ給電し、その後でUART TXを有効にする。
    digitalWrite(twe_en_pin_, LOW);
    if (!hal_twe.begin(esp_tx_pin_, esp_rx_pin_, uart_port_, baud_)) {
        pinMode(esp_tx_pin_, INPUT);
        pinMode(esp_rx_pin_, INPUT);
        digitalWrite(twe_en_pin_, HIGH);
        return false;
    }
    uart_active_ = true;
    return true;
}

void SrvTwe::powerOff() {
    if (!initialized_) {
        return;
    }

    // UART driverを削除して両線を入力へ戻してからTWELITE電源を切る。
    if (uart_active_) {
        hal_twe.end();
        uart_active_ = false;
    }
    pinMode(esp_tx_pin_, INPUT);
    pinMode(esp_rx_pin_, INPUT);
    digitalWrite(twe_en_pin_, HIGH);
}

void SrvTwe::end() {
    if (!initialized_) {
        return;
    }
    powerOff();
    initialized_ = false;
}

bool SrvTwe::sendTelemetry(const Twe::TelemetryFrame& frame)
{
    if (!initialized_ || !uart_active_) {
        return false;
    }
    // message_numberが0の場合は送信しない(1から65535までの範囲で送信する) 
    if (frame.message_number == 0) {
        return false;
    }
    uint8_t payload[1 + sizeof(Twe::TelemetryFrame)] = {0};
    payload[0] = static_cast<uint8_t>(Twe::MessageType::Telemetry); // msg_type
    memcpy(&payload[1], &frame, sizeof(Twe::TelemetryFrame));
    return hal_twe.sendto(HAL_TWE_LID_PARENT, payload, sizeof(Twe::TelemetryFrame) + 1);
}

static bool isValidTweCommandType(uint8_t msg_type)
{
    // 受信するコマンドタイプはCommand(0x20)とJog(0x21)のみ
    switch (msg_type)
    {
        case static_cast<uint8_t>(Twe::MessageType::Command):
        case static_cast<uint8_t>(Twe::MessageType::Jog):
            return true;
        default:
            return false;
    }
}

void SrvTwe::poll()
{
    if (!initialized_ || !uart_active_) {
        return;
    }
    hal_twe.poll(millis());
    HalTwePacket packet = {};
    while (hal_twe.available())
    {
        if (!hal_twe.read(&packet)) {
            continue;
        }
        // 受信データの解析
        // [msg_type:1byte][msg_data:6byte][0x00:1byte]
        if (packet.len < 8) {
            continue;
        }
        uint8_t msg_type = packet.data[0];
        if (!isValidTweCommandType(msg_type)) {
            continue;
        }
        if (rx_count >= TWE_BUFFER_SIZE) {
            rx_head = (rx_head + 1) % TWE_BUFFER_SIZE; // 読み出し位置を進めて古いパケットを破棄
            rx_count--;
        }
        rx_buffer[rx_tail].msg_type = msg_type; // 
        memcpy(rx_buffer[rx_tail].msg_data, &packet.data[1], TWE_DATA_LEN);
        rx_tail = (rx_tail + 1) % TWE_BUFFER_SIZE; // 書き込み位置を更新
        rx_count++; 
    }
}

 Twe::MessageType SrvTwe::readMsg(Twe::Data* data_out) {
    if (rx_count == 0) {
        return Twe::MessageType::None;
    }
    uint8_t msg_type = rx_buffer[rx_head].msg_type;
    memcpy(data_out, rx_buffer[rx_head].msg_data, TWE_DATA_LEN);
    rx_head = (rx_head + 1) % TWE_BUFFER_SIZE;
    rx_count--;
    return static_cast<Twe::MessageType>(msg_type);
}
