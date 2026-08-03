#pragma once
#include "TinyGPS++.h"
#include <driver/uart.h>

namespace Gps {
    // NMEAから得た診断用観測値。NAV-PVTとは混ぜず、独立した鮮度を持つ。
    struct NmeaObservation {
        uint32_t sentence_timestamp_ms;
        uint32_t location_timestamp_ms;
        uint32_t satellites_timestamp_ms;
        int32_t latitude_e7;
        int32_t longitude_e7;
        uint16_t hdop_x100;
        uint8_t satellites;
        bool location_valid;
        bool satellites_valid;
        bool hdop_valid;
    };

    // MAX-M10SのUBX-NAV-PVTをそのまま表す観測値。
    struct NavPvtObservation {
        uint32_t timestamp_ms;
        uint32_t received_ms;
        int32_t latitude_e7;
        int32_t longitude_e7;
        uint8_t fix_type;
        uint8_t satellites;
        uint32_t horizontal_accuracy_mm;
        int32_t velocity_north_mm_s;
        int32_t velocity_east_mm_s;
        uint32_t speed_accuracy_mm_s;
        bool fix_ok;
    };
    
} // namespace Gps


class SrvGps {
public:
    enum class Status : uint8_t {
        Dead = 0, 
        Searching,
        Fix
    };

    SrvGps();

    bool init(int rx, int tx, int gps_enable_pin, uart_port_t uart_num, int baud);
    bool end();

    bool powerOn();
    void powerOff();

    void poll();

    Status getStatus() const { return status; }

    bool getNmeaObservation(Gps::NmeaObservation* observation_out) const;
    bool getNavPvtObservation(Gps::NavPvtObservation* observation_out) const;

private:

    bool is_gps_initialized;
    bool gps_power_on;
    int gps_enable_pin;
    int gps_rx_pin;
    int gps_tx_pin;
    int gps_baud;
    uart_port_t uart_num;
    bool uart_active;
    
    TinyGPSPlus gps;

    Status status;
    bool is_valid;

    // NMEA
    uint32_t last_rx_time;
    uint32_t last_passed_sentences;

    // NMEAと同じUARTに混在するUBXフレームを逐次解析する。
    enum class UbxParseState : uint8_t {
        Sync1,
        Sync2,
        Class,
        Id,
        Length1,
        Length2,
        Payload,
        ChecksumA,
        ChecksumB
    };

    static constexpr uint16_t UBX_MAX_PAYLOAD_SIZE = 128;
    UbxParseState ubx_state;
    uint8_t ubx_class;
    uint8_t ubx_id;
    uint16_t ubx_length;
    uint16_t ubx_payload_index;
    uint8_t ubx_payload[UBX_MAX_PAYLOAD_SIZE];
    uint8_t ubx_checksum_a;
    uint8_t ubx_checksum_b;
    uint8_t ubx_received_checksum_a;

    Gps::NmeaObservation nmea_observation;
    bool nmea_observation_available;
    Gps::NavPvtObservation nav_pvt_observation;
    bool nav_pvt_observation_available;
    bool ubx_configuration_sent;
    uint32_t gps_power_on_time_ms;
    uint32_t last_ubx_configuration_ms;
    bool navigation_time_reference_valid;
    uint32_t previous_navigation_itow_ms;
    uint32_t previous_navigation_timestamp_ms;

    static constexpr uint32_t VALID_TIMEOUT_MS = 2000;
    static constexpr uint32_t SENTENCE_TIMEOUT_MS = 2000;
    static constexpr uint32_t GPS_WEEK_MS = 604800000;
    // NAV-PVT is timestamped at the navigation epoch. At 9600 baud, parsing
    // the complete UBX frame and the 100 ms polling cadence add about 120 ms.
    static constexpr uint32_t NAV_PVT_TRANSPORT_DELAY_MS = 120;

    bool startUart();
    void stopUart();
    void parseUbxByte(uint8_t value);
    void resetUbxParser();
    void handleUbxFrame();
    bool configureMaxM10s();
    bool sendUbx(uint8_t message_class,
                 uint8_t message_id,
                 const uint8_t* payload,
                 uint16_t payload_size);
};
