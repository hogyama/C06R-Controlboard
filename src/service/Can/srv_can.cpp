#include "srv_can.h"

#include <esp_timer.h>
#include <cstring>

namespace {

constexpr float MILLIG_TO_M_S2 = 0.00980665f;

template <typename T, size_t N>
void pushNewest(
    T (&buffer)[N],
    uint8_t& head,
    uint8_t& tail,
    uint8_t& count,
    const T& value)
{
    if (count >= N) {
        head = static_cast<uint8_t>((head + 1U) % N);
        --count;
    }
    buffer[tail] = value;
    tail = static_cast<uint8_t>((tail + 1U) % N);
    ++count;
}

template <typename T, size_t N>
bool popOldest(
    T (&buffer)[N],
    uint8_t& head,
    uint8_t& count,
    T* output)
{
    if (output == nullptr || count == 0U) return false;
    *output = buffer[head];
    head = static_cast<uint8_t>((head + 1U) % N);
    --count;
    return true;
}

} // namespace

bool SrvCan::begin(int rx, int tx)
{
    can_setting_t setting{};
    setting.baudRate = static_cast<long>(500E3);
    setting.multiData_send = true;
    setting.filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    return !can_create.begin(setting, rx, tx);
}

bool SrvCan::send(Can::Command::Velocity velocity)
{
    const uint32_t id = MAKE_CAN_ID(
        CAN_PRIO_HIGH, CAN_ADDR_MOTOR, CAN_SRC_CONTROL, CAN_TYPE_NORMAL);
    uint8_t data[8]{};
    const int32_t velocity_m_s_x100 = static_cast<int32_t>(
        velocity.velocity_mm_s * 0.1f);
    const int32_t omega_rad_s_x100 = static_cast<int32_t>(
        velocity.omega_rad_s * 100.0f);
    memcpy(data, &velocity_m_s_x100, sizeof(velocity_m_s_x100));
    memcpy(data + sizeof(velocity_m_s_x100),
           &omega_rad_s_x100,
           sizeof(omega_rad_s_x100));
    return !can_create.sendData(id, data, sizeof(data));
}

bool SrvCan::send(Can::Command::Action command)
{
    const char value = static_cast<char>(command.type);
    uint8_t address = CAN_ADDR_MOTOR;
    if (value == 'r' || value == 'k' || value == 'g') {
        address = CAN_ADDR_BROADCAST;
    } else if (
        value != 't' && value != 's' && value != 'S' && value != 'U' &&
        value != 'b' && value != 'd') {
        return false;
    }

    const uint32_t id = MAKE_CAN_ID(
        CAN_PRIO_CRITICAL, address, CAN_SRC_CONTROL, CAN_TYPE_NORMAL);
    uint8_t data[2] = {command.type_id, static_cast<uint8_t>(value)};
    return !can_create.sendData(id, data, sizeof(data));
}

void SrvCan::poll()
{
    can_return_t frame{};
    while (can_create.available()) {
        if (can_create.readWithDetail(&frame)) continue;

        uint8_t priority = 0, address = 0, source = 0, type = 0;
        PARSE_CAN_ID(frame.id, priority, address, source, type);
        (void)priority;
        const uint32_t now_ms = millis();
        const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());

        if (address == CAN_ADDR_CONTROL && source == CAN_SRC_SENSOR) {
            if (type == CAN_TYPE_ACCELERATION_XYZ &&
                frame.size == sizeof(int16_t) * 3U) {
                int16_t raw_mg[3]{};
                memcpy(raw_mg, frame.data, sizeof(raw_mg));
                Sensor::AccelerometerData sample{};
                sample.x_m_s2 = raw_mg[0] * MILLIG_TO_M_S2;
                sample.y_m_s2 = raw_mg[1] * MILLIG_TO_M_S2;
                sample.z_m_s2 = raw_mg[2] * MILLIG_TO_M_S2;
                sample.metadata = {
                    now_us, now_us, Sensor::Source::Can, true};
                pushNewest(
                    acceleration_buffer_, acceleration_head_,
                    acceleration_tail_, acceleration_count_, sample);
            } else if (type == CAN_TYPE_ANGULAR_VELOCITY_XYZ &&
                       frame.size == sizeof(int16_t) * 3U) {
                int16_t raw_x500[3]{};
                memcpy(raw_x500, frame.data, sizeof(raw_x500));
                Sensor::GyroscopeData sample{};
                sample.x_rad_s = raw_x500[0] * 0.002f;
                sample.y_rad_s = raw_x500[1] * 0.002f;
                sample.z_rad_s = raw_x500[2] * 0.002f;
                sample.metadata = {
                    now_us, now_us, Sensor::Source::Can, true};
                pushNewest(
                    gyroscope_buffer_, gyroscope_head_,
                    gyroscope_tail_, gyroscope_count_, sample);
            } else if (type == CAN_TYPE_MAGNETIC_XYZ &&
                       frame.size == sizeof(int16_t) * 3U) {
                int16_t raw_uT_x10[3]{};
                memcpy(raw_uT_x10, frame.data, sizeof(raw_uT_x10));
                Sensor::MagneticData sample{};
                sample.x_uT = raw_uT_x10[0] * 0.1f;
                sample.y_uT = raw_uT_x10[1] * 0.1f;
                sample.z_uT = raw_uT_x10[2] * 0.1f;
                sample.metadata = {
                    now_us, now_us, Sensor::Source::Can, true};
                pushNewest(
                    magnetic_buffer_, magnetic_head_,
                    magnetic_tail_, magnetic_count_, sample);
            } else if (type == CAN_TYPE_PRESSURE &&
                       frame.size == sizeof(int32_t)) {
                Sensor::PressureData sample{};
                memcpy(&sample.pressure_pa, frame.data, sizeof(sample.pressure_pa));
                sample.metadata = {
                    now_us, now_us, Sensor::Source::Can, true};
                pushNewest(
                    pressure_buffer_, pressure_head_,
                    pressure_tail_, pressure_count_, sample);
            } else if (type == CAN_TYPE_NORMAL && frame.size == 1U) {
                Can::Data::Event event{};
                if (frame.data[0] == Can::Data::AscendDetected) {
                    event.bytes = Can::Data::AscendDetected;
                } else if (frame.data[0] == Can::Data::LandingDetected) {
                    event.bytes = Can::Data::LandingDetected;
                } else {
                    continue;
                }
                event.ts_ms = now_ms;
                pushNewest(
                    event_buffer_, event_head_, event_tail_, event_count_, event);
            }
        } else if (address == CAN_ADDR_CONTROL && source == CAN_SRC_MOTOR) {
            if (type == CAN_TYPE_ENCODER &&
                frame.size == sizeof(int32_t) * 2U) {
                Can::Data::Encoder sample{};
                memcpy(&sample.left_mm, frame.data, sizeof(sample.left_mm));
                memcpy(&sample.right_mm,
                       frame.data + sizeof(sample.left_mm),
                       sizeof(sample.right_mm));
                sample.sequence = ++encoder_sequence_;
                if (sample.sequence == 0U) sample.sequence = ++encoder_sequence_;
                sample.metadata = {
                    now_us, now_us, Sensor::Source::Can, true};
                sample.left_valid = true;
                sample.right_valid = true;
                pushNewest(
                    encoder_buffer_, encoder_head_,
                    encoder_tail_, encoder_count_, sample);
            } else if (type == CAN_TYPE_NORMAL && frame.size == 1U) {
                Can::Data::Event event{};
                if (frame.data[0] == Can::Data::StuckResolved) {
                    event.bytes = Can::Data::StuckResolved;
                } else if (frame.data[0] == Can::Data::SeparationFinished) {
                    event.bytes = Can::Data::SeparationFinished;
                } else if (frame.data[0] == 0x04U) {
                    event.bytes = Can::Data::UprightRecoveryFailed;
                } else {
                    continue;
                }
                event.ts_ms = now_ms;
                pushNewest(
                    event_buffer_, event_head_, event_tail_, event_count_, event);
            }
        }
    }
}

bool SrvCan::read(Sensor::AccelerometerData* acceleration)
{
    return popOldest(
        acceleration_buffer_, acceleration_head_,
        acceleration_count_, acceleration);
}

bool SrvCan::read(Sensor::GyroscopeData* gyroscope)
{
    return popOldest(
        gyroscope_buffer_, gyroscope_head_, gyroscope_count_, gyroscope);
}

bool SrvCan::read(Sensor::MagneticData* magnetic)
{
    return popOldest(
        magnetic_buffer_, magnetic_head_, magnetic_count_, magnetic);
}

bool SrvCan::read(Sensor::PressureData* pressure)
{
    return popOldest(
        pressure_buffer_, pressure_head_, pressure_count_, pressure);
}

bool SrvCan::read(Can::Data::Encoder* encoder)
{
    return popOldest(
        encoder_buffer_, encoder_head_, encoder_count_, encoder);
}

Can::Data::Event SrvCan::readEvent()
{
    Can::Data::Event event{};
    if (!popOldest(event_buffer_, event_head_, event_count_, &event)) {
        event.bytes = Can::Data::None;
    }
    return event;
}
