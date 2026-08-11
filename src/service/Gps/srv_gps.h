#pragma once

#include <Arduino.h>
#include <driver/uart.h>
#include "domain/sensor/sensor_types.h"

namespace Gps {

struct NavPvtObservation {
    Sensor::SampleMetadata metadata;
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
    bool getNavPvtObservation(Gps::NavPvtObservation* observation_out) const;

private:
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
    static constexpr uint32_t VALID_TIMEOUT_MS = 2000;
    static constexpr uint32_t GPS_WEEK_MS = 604800000;
    static constexpr uint32_t NAV_PVT_TRANSPORT_DELAY_MS = 120;

    bool is_gps_initialized;
    bool gps_power_on;
    int gps_enable_pin;
    int gps_rx_pin;
    int gps_tx_pin;
    int gps_baud;
    uart_port_t uart_num;
    bool uart_active;
    Status status;
    bool is_valid;

    UbxParseState ubx_state;
    uint8_t ubx_class;
    uint8_t ubx_id;
    uint16_t ubx_length;
    uint16_t ubx_payload_index;
    uint8_t ubx_payload[UBX_MAX_PAYLOAD_SIZE];
    uint8_t ubx_checksum_a;
    uint8_t ubx_checksum_b;
    uint8_t ubx_received_checksum_a;

    Gps::NavPvtObservation nav_pvt_observation;
    bool nav_pvt_observation_available;
    bool ubx_configuration_sent;
    uint32_t gps_power_on_time_ms;
    uint32_t last_ubx_configuration_ms;
    bool navigation_time_reference_valid;
    uint32_t previous_navigation_itow_ms;
    uint64_t previous_navigation_timestamp_us;

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
