#include "srv_gps.h"

#include <esp_timer.h>

namespace {

uint32_t receivedAgeMs(
    const Gps::NavPvtObservation& observation,
    uint64_t now_us)
{
    if (observation.metadata.received_us == 0U ||
        now_us < observation.metadata.received_us) {
        return UINT32_MAX;
    }
    const uint64_t age_ms =
        (now_us - observation.metadata.received_us) / 1000ULL;
    return age_ms > UINT32_MAX ? UINT32_MAX : static_cast<uint32_t>(age_ms);
}

} // namespace

SrvGps::SrvGps()
    : is_gps_initialized(false),
      gps_power_on(false),
      gps_enable_pin(-1),
      gps_rx_pin(-1),
      gps_tx_pin(-1),
      gps_baud(0),
      uart_num(UART_NUM_MAX),
      uart_active(false),
      status(Status::Dead),
      is_valid(false),
      ubx_state(UbxParseState::Sync1),
      ubx_class(0),
      ubx_id(0),
      ubx_length(0),
      ubx_payload_index(0),
      ubx_checksum_a(0),
      ubx_checksum_b(0),
      ubx_received_checksum_a(0),
      nav_pvt_observation{},
      nav_pvt_observation_available(false),
      ubx_configuration_sent(false),
      gps_power_on_time_ms(0),
      last_ubx_configuration_ms(0),
      navigation_time_reference_valid(false),
      previous_navigation_itow_ms(0),
       previous_navigation_timestamp_us(0)
{
}

bool SrvGps::init(int rx, int tx, int gps_enable_pin, uart_port_t uart_num, int baud)
{
    if (is_gps_initialized || rx < 0 || tx < 0 || gps_enable_pin < 0 ||
        uart_num < 0 || uart_num >= UART_NUM_MAX || baud <= 0) {
        return false;
    }

    // initでは設定を保存するだけにし、UART線は高インピーダンスに保つ。
    pinMode(gps_enable_pin, OUTPUT);
    digitalWrite(gps_enable_pin, LOW);
    pinMode(tx, INPUT);
    pinMode(rx, INPUT);

    this->gps_enable_pin = gps_enable_pin;
    this->gps_rx_pin = rx;
    this->gps_tx_pin = tx;
    this->gps_baud = baud;
    this->uart_num = uart_num;
    is_gps_initialized = true;
    return true;
}

bool SrvGps::startUart()
{
    if (!is_gps_initialized) return false;
    if (uart_active) return true;

    uart_config_t uart_config = {
        .baud_rate = gps_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    if (uart_param_config(uart_num, &uart_config) != ESP_OK) return false;
    if (uart_set_pin(
            uart_num,
            gps_tx_pin,
            gps_rx_pin,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE) != ESP_OK) {
        return false;
    }
    if (uart_driver_install(uart_num, 1024, 0, 0, nullptr, 0) != ESP_OK) {
        pinMode(gps_tx_pin, INPUT);
        pinMode(gps_rx_pin, INPUT);
        return false;
    }

    uart_active = true;
    return true;
}

void SrvGps::stopUart()
{
    if (uart_active) {
        uart_driver_delete(uart_num);
        uart_active = false;
    }

    if (gps_tx_pin >= 0) {
        pinMode(gps_tx_pin, INPUT);
    }
    if (gps_rx_pin >= 0) {
        pinMode(gps_rx_pin, INPUT);
    }
}

bool SrvGps::end()
{
    if (!is_gps_initialized) return false;
    stopUart();
    digitalWrite(gps_enable_pin, LOW);
    gps_power_on = false;
    is_valid = false;
    status = Status::Dead;
    is_gps_initialized = false;
    return true;
}

bool SrvGps::powerOn()
{
    if (!is_gps_initialized) return false;
    if (gps_power_on) return true;

    // 先にGPSへ給電し、その後でUART TXを有効にする。
    digitalWrite(gps_enable_pin, HIGH);
    if (!startUart()) {
        stopUart();
        digitalWrite(gps_enable_pin, LOW);
        status = Status::Dead;
        return false;
    }

    gps_power_on = true;
    is_valid = false;
    status = Status::Searching;
    nav_pvt_observation = {};
    nav_pvt_observation_available = false;
    ubx_configuration_sent = false;
    gps_power_on_time_ms = millis();
    last_ubx_configuration_ms = 0;
    navigation_time_reference_valid = false;
    previous_navigation_itow_ms = 0;
    previous_navigation_timestamp_us = 0;
    resetUbxParser();
    return true;
}   

void SrvGps::powerOff()
{
    if (!is_gps_initialized) return;

    // UART driverを削除して両線を入力へ戻してからGPS電源を切る。
    stopUart();
    digitalWrite(gps_enable_pin, LOW);
    gps_power_on = false;
    is_valid = false;
    nav_pvt_observation_available = false;
    ubx_configuration_sent = false;
    last_ubx_configuration_ms = 0;
    navigation_time_reference_valid = false;
    previous_navigation_itow_ms = 0;
    previous_navigation_timestamp_us = 0;
    status = Status::Dead;
}

void SrvGps::poll()
{
    if (!is_gps_initialized || !gps_power_on){
        // GPSが初期化されていない、または電源が入っていない場合は、ステータスをDeadにする
        status = Status::Dead;
        return;
    }

    // MAX-M10Sの起動完了を待ち、RAM設定だけを毎回適用する。
    // UART出力をUBX-NAV-PVTだけに設定する。
    const uint32_t now_ms = millis();
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    const bool nav_pvt_missing_or_stale =
        !nav_pvt_observation_available ||
        receivedAgeMs(nav_pvt_observation, now_us) >= VALID_TIMEOUT_MS;
    const bool configuration_retry_due =
        nav_pvt_missing_or_stale &&
        (last_ubx_configuration_ms == 0U ||
         static_cast<uint32_t>(now_ms - last_ubx_configuration_ms) >= 2000U);
    if (static_cast<uint32_t>(now_ms - gps_power_on_time_ms) >= 500U &&
        (!ubx_configuration_sent || configuration_retry_due)) {
        ubx_configuration_sent = configureMaxM10s();
        last_ubx_configuration_ms = now_ms;
    }

    uint8_t gps_buffer[128];
    while (true) {
        const int len = uart_read_bytes(
            uart_num, gps_buffer, sizeof(gps_buffer), 0);
        if (len <= 0) {
            break; // 読み切ったのでループを抜ける
        }
        for (int i = 0; i < len; i++) {
            parseUbxByte(gps_buffer[i]);
        }
    }
    const uint64_t status_now_us =
        static_cast<uint64_t>(esp_timer_get_time());
    is_valid =
        nav_pvt_observation_available &&
        nav_pvt_observation.fix_ok &&
        receivedAgeMs(nav_pvt_observation, status_now_us) <= VALID_TIMEOUT_MS;

    // ステータスの更新
    const bool nav_pvt_fresh =
        nav_pvt_observation_available &&
        receivedAgeMs(nav_pvt_observation, status_now_us) < VALID_TIMEOUT_MS;
    if (!nav_pvt_fresh) {
        status = Status::Dead;
    } else if (is_valid) {
        status = Status::Fix;
    } else {
        status = Status::Searching;
    }
}

bool SrvGps::getNavPvtObservation(
    Gps::NavPvtObservation* observation_out) const
{
    if (observation_out == nullptr ||
        !is_gps_initialized ||
        !gps_power_on ||
        !nav_pvt_observation_available) {
        return false;
    }
    *observation_out = nav_pvt_observation;
    return true;
}

namespace {

template <typename T>
T readLittleEndian(const uint8_t* data)
{
    T value{};
    memcpy(&value, data, sizeof(T));
    return value;
}

} // namespace

void SrvGps::resetUbxParser()
{
    ubx_state = UbxParseState::Sync1;
    ubx_class = 0;
    ubx_id = 0;
    ubx_length = 0;
    ubx_payload_index = 0;
    ubx_checksum_a = 0;
    ubx_checksum_b = 0;
    ubx_received_checksum_a = 0;
}

void SrvGps::parseUbxByte(uint8_t value)
{
    auto add_checksum = [this](uint8_t byte) {
        ubx_checksum_a = static_cast<uint8_t>(ubx_checksum_a + byte);
        ubx_checksum_b = static_cast<uint8_t>(ubx_checksum_b + ubx_checksum_a);
    };

    switch (ubx_state) {
        case UbxParseState::Sync1:
            if (value == 0xB5) ubx_state = UbxParseState::Sync2;
            break;

        case UbxParseState::Sync2:
            if (value == 0x62) {
                ubx_checksum_a = 0;
                ubx_checksum_b = 0;
                ubx_state = UbxParseState::Class;
            } else {
                ubx_state = value == 0xB5
                    ? UbxParseState::Sync2
                    : UbxParseState::Sync1;
            }
            break;

        case UbxParseState::Class:
            ubx_class = value;
            add_checksum(value);
            ubx_state = UbxParseState::Id;
            break;

        case UbxParseState::Id:
            ubx_id = value;
            add_checksum(value);
            ubx_state = UbxParseState::Length1;
            break;

        case UbxParseState::Length1:
            ubx_length = value;
            add_checksum(value);
            ubx_state = UbxParseState::Length2;
            break;

        case UbxParseState::Length2:
            ubx_length |= static_cast<uint16_t>(value) << 8;
            add_checksum(value);
            ubx_payload_index = 0;
            if (ubx_length > UBX_MAX_PAYLOAD_SIZE) {
                resetUbxParser();
            } else {
                ubx_state = ubx_length == 0
                    ? UbxParseState::ChecksumA
                    : UbxParseState::Payload;
            }
            break;

        case UbxParseState::Payload:
            ubx_payload[ubx_payload_index++] = value;
            add_checksum(value);
            if (ubx_payload_index >= ubx_length) {
                ubx_state = UbxParseState::ChecksumA;
            }
            break;

        case UbxParseState::ChecksumA:
            ubx_received_checksum_a = value;
            ubx_state = UbxParseState::ChecksumB;
            break;

        case UbxParseState::ChecksumB:
            if (ubx_received_checksum_a == ubx_checksum_a &&
                value == ubx_checksum_b) {
                handleUbxFrame();
            }
            resetUbxParser();
            break;
    }
}

void SrvGps::handleUbxFrame()
{
    // UBX-NAV-PVT (class 0x01, id 0x07)。
    // M10 SPGのNAV-PVTは92 byteで、使用フィールドはoffset 68までに収まる。
    if (ubx_class != 0x01 || ubx_id != 0x07 || ubx_length < 72) {
        return;
    }

    const uint8_t fix_type = ubx_payload[20];
    const uint8_t flags = ubx_payload[21];

    const uint64_t arrival_timestamp_us =
        static_cast<uint64_t>(esp_timer_get_time());
    const uint32_t navigation_itow_ms =
        readLittleEndian<uint32_t>(&ubx_payload[0]);
    uint64_t navigation_timestamp_us =
        arrival_timestamp_us > NAV_PVT_TRANSPORT_DELAY_MS * 1000ULL
            ? arrival_timestamp_us - NAV_PVT_TRANSPORT_DELAY_MS * 1000ULL
            : 1U;

    if (navigation_time_reference_valid) {
        int64_t itow_delta_ms =
            static_cast<int64_t>(navigation_itow_ms) -
            static_cast<int64_t>(previous_navigation_itow_ms);
        if (itow_delta_ms < -static_cast<int64_t>(GPS_WEEK_MS / 2U)) {
            itow_delta_ms += GPS_WEEK_MS;
        } else if (
            itow_delta_ms > static_cast<int64_t>(GPS_WEEK_MS / 2U)) {
            itow_delta_ms -= GPS_WEEK_MS;
        }

        if (itow_delta_ms > 0 && itow_delta_ms <= 2000) {
            const uint64_t predicted_timestamp_us =
                previous_navigation_timestamp_us +
                static_cast<uint64_t>(itow_delta_ms) * 1000ULL;
            const int64_t anchor_error_us =
                static_cast<int64_t>(navigation_timestamp_us) -
                static_cast<int64_t>(predicted_timestamp_us);
            if (anchor_error_us >= -500000 && anchor_error_us <= 500000) {
                // Preserve the GNSS navigation interval and remove task/UART
                // scheduling jitter from the measurement time.
                navigation_timestamp_us = predicted_timestamp_us;
            }
        }
    }

    navigation_time_reference_valid = true;
    previous_navigation_itow_ms = navigation_itow_ms;
    previous_navigation_timestamp_us = navigation_timestamp_us;
    nav_pvt_observation.metadata.timestamp_us = navigation_timestamp_us;
    nav_pvt_observation.metadata.received_us = arrival_timestamp_us;
    nav_pvt_observation.metadata.source = Sensor::Source::Gps;
    nav_pvt_observation.longitude_e7 =
        readLittleEndian<int32_t>(&ubx_payload[24]);
    nav_pvt_observation.latitude_e7 =
        readLittleEndian<int32_t>(&ubx_payload[28]);
    nav_pvt_observation.fix_type = fix_type;
    nav_pvt_observation.satellites = ubx_payload[23];
    nav_pvt_observation.horizontal_accuracy_mm =
        readLittleEndian<uint32_t>(&ubx_payload[40]);
    nav_pvt_observation.velocity_north_mm_s =
        readLittleEndian<int32_t>(&ubx_payload[48]);
    nav_pvt_observation.velocity_east_mm_s =
        readLittleEndian<int32_t>(&ubx_payload[52]);
    nav_pvt_observation.speed_accuracy_mm_s =
        readLittleEndian<uint32_t>(&ubx_payload[68]);
    nav_pvt_observation.fix_ok =
        (flags & 0x01U) != 0U && fix_type >= 2U;
    nav_pvt_observation.metadata.valid = nav_pvt_observation.fix_ok;
    nav_pvt_observation_available = true;
}

bool SrvGps::sendUbx(uint8_t message_class,
                     uint8_t message_id,
                     const uint8_t* payload,
                     uint16_t payload_size)
{
    if (!uart_active ||
        payload == nullptr ||
        payload_size > UBX_MAX_PAYLOAD_SIZE) {
        return false;
    }

    uint8_t frame[UBX_MAX_PAYLOAD_SIZE + 8]{};
    frame[0] = 0xB5;
    frame[1] = 0x62;
    frame[2] = message_class;
    frame[3] = message_id;
    frame[4] = static_cast<uint8_t>(payload_size & 0xFFU);
    frame[5] = static_cast<uint8_t>((payload_size >> 8) & 0xFFU);
    memcpy(&frame[6], payload, payload_size);

    uint8_t checksum_a = 0;
    uint8_t checksum_b = 0;
    for (uint16_t i = 2; i < payload_size + 6; ++i) {
        checksum_a = static_cast<uint8_t>(checksum_a + frame[i]);
        checksum_b = static_cast<uint8_t>(checksum_b + checksum_a);
    }
    frame[payload_size + 6] = checksum_a;
    frame[payload_size + 7] = checksum_b;

    const int written = uart_write_bytes(
        uart_num,
        reinterpret_cast<const char*>(frame),
        payload_size + 8
    );
    return written == static_cast<int>(payload_size + 8);
}

bool SrvGps::configureMaxM10s()
{
    // UBX-CFG-VALSET version 0、RAM layer。
    // CFG-UART1OUTPROT-UBX  = true
    // CFG-UART1OUTPROT-NMEA = false
    // CFG-MSGOUT-UBX_NAV_PVT_UART1 = every navigation solution
    constexpr uint32_t KEY_UART1_OUT_UBX = 0x10740001UL;
    constexpr uint32_t KEY_UART1_OUT_NMEA = 0x10740002UL;
    // 地上を低速走行するCanSatに合わせ、測位フィルタを歩行者モデルにする。
    constexpr uint32_t KEY_NAVSPG_DYNMODEL = 0x20110021UL;
    constexpr uint8_t DYNMODEL_PEDESTRIAN = 3U;
    constexpr uint32_t KEY_NAV_PVT_UART1 = 0x20910007UL;
    constexpr uint32_t KEY_NMEA_GGA_UART1 = 0x209100BBUL;
    constexpr uint32_t KEY_NMEA_GLL_UART1 = 0x209100CAUL;
    constexpr uint32_t KEY_NMEA_GSA_UART1 = 0x209100C0UL;
    constexpr uint32_t KEY_NMEA_GSV_UART1 = 0x209100C5UL;
    constexpr uint32_t KEY_NMEA_RMC_UART1 = 0x209100ACUL;
    constexpr uint32_t KEY_NMEA_VTG_UART1 = 0x209100B1UL;

    // NMEA各メッセージも明示的に無効化する。
    uint8_t payload[54]{};
    payload[0] = 0x00; // version
    payload[1] = 0x01; // RAM layer

    size_t offset = 4;
    auto appendU1 = [&payload, &offset](uint32_t key, uint8_t value) {
        memcpy(&payload[offset], &key, sizeof(key));
        offset += sizeof(key);
        payload[offset++] = value;
    };
    appendU1(KEY_UART1_OUT_UBX, 1);
    appendU1(KEY_UART1_OUT_NMEA, 0);
    appendU1(KEY_NAVSPG_DYNMODEL, DYNMODEL_PEDESTRIAN);
    appendU1(KEY_NAV_PVT_UART1, 1);
    appendU1(KEY_NMEA_GGA_UART1, 0);
    appendU1(KEY_NMEA_GLL_UART1, 0);
    appendU1(KEY_NMEA_GSA_UART1, 0);
    appendU1(KEY_NMEA_GSV_UART1, 0);
    appendU1(KEY_NMEA_RMC_UART1, 0);
    appendU1(KEY_NMEA_VTG_UART1, 0);

    return offset == sizeof(payload) &&
        sendUbx(0x06, 0x8A, payload, sizeof(payload));
}
