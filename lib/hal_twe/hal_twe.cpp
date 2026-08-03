#include "hal_twe.h"

uint8_t HalTwe::high_nibble = 0xFF;

bool HalTwe::begin(uint8_t esp_tx_pin, uint8_t esp_rx_pin, uart_port_t uart_port, uint32_t baud)
{
    if (is_begin) {
        // 同じ設定なら再installせず、異なる設定なら一度完全に停止する。
        if (this->esp_tx_pin == esp_tx_pin &&
            this->esp_rx_pin == esp_rx_pin &&
            this->port == uart_port &&
            this->baud == baud) {
            return true;
        }
        end();
    }

    // UARTの初期化
    uart_config_t uart_config = {
        .baud_rate = (int)baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    if (uart_param_config(uart_port, &uart_config) != ESP_OK) {
        return false;
    }
    if (uart_set_pin(uart_port, esp_tx_pin, esp_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        return false;
    }
    if (uart_driver_install(uart_port, DRIVER_BUFFER_SIZE, 0, 0, NULL, 0) != ESP_OK) {
        return false;
    }

    this->port = uart_port;
    this->esp_tx_pin = esp_tx_pin;
    this->esp_rx_pin = esp_rx_pin;
    this->baud = baud;

    // 内部状態の初期化
    this->config.link_timeout_ms = HAL_TWE_LINK_TIMEOUT_MS;
    this->config.default_id = HAL_TWE_LID_PARENT;
    this->status.first_valid = false;
    this->status.last_rx_81_ms = 0;
    this->is_begin = true;

    // バッファ初期化
    this->rx_head = 0;
    this->rx_tail = 0;
    this->rx_count = 0;
    this->high_nibble = 0xFF;
    
    // パーサ状態初期化
    this->parse_state = WAIT_COLON;
    this->frame_len = 0;

    return true;
}

void HalTwe::end()
{
    if(is_begin) {
        uart_driver_delete(port);
        is_begin = false;
    }
}

bool HalTwe::setConfig(HalTweConfig config)
{
    this->config = config;
    return true;
}

bool HalTwe::getConfig(HalTweConfig* config) const
{
    if(config == nullptr){
        return false;
    }
    *config = this->config;
    return true;
}

void HalTwe::poll(uint32_t now_ms)
{
    if(!is_begin){
        return;
    }
    uint8_t ch;
    while(uart_read_bytes(port, &ch, 1, 0) > 0){
        switch(parse_state){
            case WAIT_COLON:
                if (ch == ':') {
                    frame_len = 0;
                    high_nibble = 0xFF;
                    parse_state = READ_HEX;
                }
                break;
            case READ_HEX:
                if (ch == '\r') {
                    if (high_nibble == 0xFF){
                        parse_state = WAIT_LF;
                    }else{
                        // 不正なフレームなので破棄
                        parse_state = WAIT_COLON;
                        frame_len = 0;
                        high_nibble = 0xFF;
                    }
                } else {
                    // フレーム長がバッファサイズを超えないようにする
                    if(frame_len < FRAME_BUF_SIZE){
                        int8_t nibble = HexToNibble(ch);
                        if(nibble >= 0){
                            if(high_nibble == 0xFF){
                                high_nibble = nibble;
                            } else {
                                frame_buf[frame_len++] = (high_nibble << 4) | nibble;
                                high_nibble = 0xFF;
                            }
                        } else {
                            // 不正な文字なのでフレームを破棄
                            parse_state = WAIT_COLON;
                            frame_len = 0;
                            high_nibble = 0xFF;
                        }
                    }
                }
                break;
            case WAIT_LF:
                if(ch == '\n'){
                    if(frame_len >= 6){ 
                        uint8_t lrc = CalcLrc(frame_buf, frame_len - 1);
                        if(lrc == frame_buf[frame_len - 1]){
                            uint8_t cmd = frame_buf[1];
                            const uint8_t* payload = &frame_buf[2];
                            uint8_t payload_len = frame_len - 3; // ID + CMD + LRCを除く
                            switch(cmd){
                                case APP_TWE_CMD_USER_DATA:
                                    Handle01(payload, payload_len);
                                    break;
                                case APP_TWE_CMD_STATUS:
                                    Handle81(payload, payload_len, now_ms);
                                    break;
                                default:
                                    break;
                            }
                        }
                    }
                }
                parse_state = WAIT_COLON;
                frame_len = 0;
                high_nibble = 0xFF;
                break;
        }
    }
}

void HalTwe::Handle01(const uint8_t* payload, uint8_t len)
{
    if (payload == nullptr || len == 0) {
        return;
    }
    HalTwePacket packet = {};
    if (len > HAL_TWE_USER_DATA_MAX_LEN) {
        len = HAL_TWE_USER_DATA_MAX_LEN;
    }
    packet.len = len;
    memcpy(packet.data, payload, len);
#ifdef DEBUG_HAL_TWE
    // 受信した長さ
    Serial.print("[HAL 0x01] len=");
    Serial.print(packet.len);
    // 受信したデータの16進数表示
    Serial.print(" raw=");
    for (uint8_t i = 0; i < packet.len; i++) {
        if (packet.data[i] < 0x10) Serial.print("0");
        Serial.print(packet.data[i], HEX);
        Serial.print(" ");
    }
    Serial.println();
#endif
    PushRxPacket(packet);
}


void HalTwe::Handle81(const uint8_t* payload, uint8_t len, uint32_t now_ms)
{
    if(len < 22 || payload == nullptr){
        return;
    }
    status.first_valid = true;
    status.lqi = payload[2];  // LQIはpayloadの3バイト目
    status.last_rx_81_ms = now_ms;
}

void HalTwe::PushRxPacket(const HalTwePacket& packet)
{
    if (rx_count >= RX_QUEUE_SIZE) {
        rx_head = (rx_head + 1) % RX_QUEUE_SIZE; //　読み出し位置を進めて古いパケットを破棄
        rx_count--;
    }
    rx_ring[rx_tail] = packet; // 新しいパケットを末尾に追加
    rx_tail = (rx_tail + 1) % RX_QUEUE_SIZE; // 書き込み位置を進める 
    rx_count++;
}

bool HalTwe::PopRxPacket(HalTwePacket* packet)
{
    if(packet == nullptr || rx_count == 0){
        return false;
    }
    *packet = rx_ring[rx_head]; // 読み出し位置のパケットを返す
    rx_head = (rx_head + 1) % RX_QUEUE_SIZE; // 読み出し位置を進める
    rx_count--;
    return true;
}

bool HalTwe::send(const uint8_t* data, uint8_t len)
{
    return sendto(config.default_id, data, len);
}

bool HalTwe::sendto(uint8_t dest_id, const uint8_t* data, uint8_t len)
{
    if(!is_begin || data == nullptr || len == 0 || len > HAL_TWE_USER_DATA_MAX_LEN){
        return false;
    }
    uint8_t payload[2 + HAL_TWE_USER_DATA_MAX_LEN] = {0};
    payload[0] = dest_id;
    payload[1] = APP_TWE_CMD_USER_DATA;
    memcpy(&payload[2], data, len);
    uint8_t lrc = CalcLrc(payload, len + 2);
    char ascii[1 + (1 + 1 + HAL_TWE_USER_DATA_MAX_LEN + 1) * 2 + 2 + 1] = {}; //コロン + (ID + CMD + DATA + LRC) * 2 + CRLF + NULL
    uint16_t idx = 0;
    ascii[idx++] = ':';
    for(uint8_t i = 0; i < len + 2; i++){
        ascii[idx++] = NibbleToHex((payload[i] >> 4) & 0x0F);
        ascii[idx++] = NibbleToHex(payload[i] & 0x0F);
    }
    ascii[idx++] = NibbleToHex(lrc >> 4);
    ascii[idx++] = NibbleToHex(lrc & 0x0F);
    ascii[idx++] = '\r';
    ascii[idx++] = '\n';
    ascii[idx] = '\0';
    int written = uart_write_bytes(port, ascii, idx);
    return written == idx;
}

uint8_t HalTwe::available() const
{
    return rx_count;
}

bool HalTwe::read(HalTwePacket* packet)
{
    if(packet == nullptr){
        return false;
    }
    if(PopRxPacket(packet)){
        return true;
    }
    return false;
}

bool HalTwe::getStatus(HalTweStatus* status) const
{
    if(status == nullptr){
        return false;
    }
    *status = this->status;
    return true;
}

bool HalTwe::islinkalive(uint32_t now_ms) const
{
    if(!status.first_valid){
        return false;
    }
    return (now_ms - status.last_rx_81_ms) < config.link_timeout_ms;
}

int8_t HalTwe::HexToNibble(uint8_t ch)
{
    if(ch >= '0' && ch <= '9'){
        return ch - '0';
    } else if(ch >= 'A' && ch <= 'F'){
        return ch - 'A' + 10;
    } else if(ch >= 'a' && ch <= 'f'){
        return ch - 'a' + 10;
    } else {
        return -1;
    }
}

char HalTwe::NibbleToHex(uint8_t nibble)
{
    if(nibble < 10){
        return '0' + nibble;
    } else if(nibble < 16){
        return 'A' + (nibble - 10);
    } else {
        return '?';
    }
}

uint8_t HalTwe::CalcLrc(const uint8_t* data, uint8_t len)
{
    if(data == nullptr || len == 0){
        return 0;
    }
    uint8_t sum = 0;
    for(uint8_t i = 0; i < len; i++){
        sum += (uint8_t)data[i];
    }
    return (uint8_t)(0 - sum);
}
