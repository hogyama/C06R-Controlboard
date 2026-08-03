#pragma once
#include <Arduino.h>
#include <driver/uart.h>

// DEBUG
// #define DEBUG_HAL_TWE

/**
 * App_Twelite HAL
 * 0x01 任意データ送受信、0x81 状態通知受信を行う。
 * このクラスでは、0x01 payload の意味は解釈しない。
 * 子機での使用を想定している。  
 */
#define HAL_TWE_USER_DATA_MAX_LEN   80
#define HAL_TWE_LINK_TIMEOUT_MS     5000

#define HAL_TWE_LID_PARENT          0x00
#define HAL_TWE_LID_BROADCAST       0x78
#define HAL_TWE_LID_UNKNOWN         0xFF

#define APP_TWE_CMD_USER_DATA       0x01
#define APP_TWE_CMD_STATUS          0x81

#define RX_QUEUE_SIZE 3
#define FRAME_BUF_SIZE 128
#define DRIVER_BUFFER_SIZE 512

//保持する設定
struct HalTweConfig{
    uint32_t link_timeout_ms; //リンクタイムアウト時間
    uint8_t default_id; //デフォルト送信先ID
};

//0x01受信データ構造体
struct HalTwePacket{
    uint8_t data[HAL_TWE_USER_DATA_MAX_LEN];
    uint8_t len;
};

//0x81受信データ構造体
struct HalTweStatus{
    bool first_valid;
    uint8_t lqi;
    uint32_t last_rx_81_ms;
};

class HalTwe{
    public:
    /**
     * UARTのport、pin設定、baud設定,および内部状態を初期化する.
     * すでにbegin()済みの場合は UART driverを削除して再初期化する.
     */
    bool begin(uint8_t esp_tx_pin, uint8_t esp_rx_pin, uart_port_t uart_port, uint32_t baud = 115200);
    void end();
    
    bool setConfig(HalTweConfig config);
    bool getConfig(HalTweConfig* config) const;

    void poll(uint32_t now_ms);

    bool send(const uint8_t* data, 
              uint8_t len);

    bool sendto(uint8_t dest_id, const uint8_t* data, uint8_t len);

    uint8_t available() const;

    bool read(HalTwePacket* packet);

    bool getStatus(HalTweStatus* status) const; 
    
    bool islinkalive(uint32_t now_ms) const;

    private:

    bool is_begin = false;
    uart_port_t port;
    int8_t esp_tx_pin;
    int8_t esp_rx_pin;
    uint32_t baud;

    HalTweConfig config;
    HalTweStatus status;

    HalTwePacket rx_ring[RX_QUEUE_SIZE];
    uint8_t rx_head; // 受信リングバッファの先頭インデックス(読み出し位置)
    uint8_t rx_tail; // 受信リングバッファの末尾インデックス(書き込み位置)
    uint8_t rx_count; // 受信リングバッファの要素数
    
    void Handle01(const uint8_t* payload, uint8_t len);
    void Handle81(const uint8_t* payload, uint8_t len, uint32_t now_ms);
    void PushRxPacket(const HalTwePacket& packet);
    bool PopRxPacket(HalTwePacket* packet);
    enum ParseState{
        WAIT_COLON,
        READ_HEX,
        WAIT_LF
    };
    ParseState parse_state;
    uint8_t frame_buf[FRAME_BUF_SIZE];
    uint8_t frame_len;

    static uint8_t high_nibble;
    static int8_t HexToNibble(uint8_t ch);
    static char NibbleToHex(uint8_t nibble);
    static uint8_t CalcLrc(const uint8_t* data, uint8_t len);
};
