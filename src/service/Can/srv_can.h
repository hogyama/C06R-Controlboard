#pragma once

#include <Arduino.h>
#include <CANCREATE.h>

#include "domain/sensor/sensor_types.h"

#define CAN_PRIO_CRITICAL 0b00
#define CAN_PRIO_HIGH 0b01
#define CAN_PRIO_NORMAL 0b11

#define CAN_ADDR_BROADCAST 0b011
#define CAN_ADDR_CONTROL 0b100
#define CAN_ADDR_SENSOR 0b010
#define CAN_ADDR_MOTOR 0b001

#define CAN_SRC_CONTROL 0b00
#define CAN_SRC_SENSOR 0b01
#define CAN_SRC_MOTOR 0b10

#define CAN_TYPE_NORMAL 0b0000
#define CAN_TYPE_ENCODER 0b0001
#define CAN_TYPE_ACCELERATION_XYZ 0b0010
#define CAN_TYPE_MAGNETIC_XYZ 0b0011
#define CAN_TYPE_PRESSURE 0b0100
#define CAN_TYPE_ANGULAR_VELOCITY_XYZ 0b0110

#define MAKE_CAN_ID(prio, addr, src, type) \
    ((((prio) & 0b11) << 9) | (((addr) & 0b111) << 6) | \
     (((src) & 0b11) << 4) | ((type) & 0b1111))

#define PARSE_CAN_ID(id, prio, addr, src, type) \
    do { \
        (prio) = ((id) >> 9) & 0b11; \
        (addr) = ((id) >> 6) & 0b111; \
        (src) = ((id) >> 4) & 0b11; \
        (type) = (id) & 0b1111; \
    } while (0)

namespace Can::Data {

enum EventBytes : uint8_t {
    None = 0x00,
    StuckResolved = 0x01,
    SeparationFinished = 0x02,
    AscendDetected = 0x04,
    LandingDetected = 0x05,
    UprightRecoveryFailed = 0x06,
};

struct Encoder {
    // Motor board sends cumulative wheel travel in millimetres.
    int32_t left_mm;
    int32_t right_mm;
    uint32_t sequence;
    Sensor::SampleMetadata metadata;
    bool left_valid;
    bool right_valid;
};

struct Event {
    EventBytes bytes;
    uint32_t ts_ms;
};

} // namespace Can::Data

namespace Can::Command {

enum ActionType : char {
    Reset = 'r',
    SequenceStart = 'k',
    NotifyGoal = 'g',
    NotifySeparation = 't',
    NotifyStuck = 's',
    NotifyFlipped = 'S',
    ConfirmUpright = 'U',
    ServoUnlock = 'b',
    ServoLock = 'd'
};

struct Action {
    static constexpr uint8_t TYPE_ID = 0x01;
    uint8_t type_id = TYPE_ID;
    ActionType type;
};

struct Velocity {
    float velocity_mm_s;
    float omega_rad_s;
};

} // namespace Can::Command

class SrvCan {
public:
    bool begin(int rx, int tx);
    bool send(Can::Command::Velocity velocity);
    bool send(Can::Command::Action command);
    void poll();

    bool read(Sensor::AccelerometerData* acceleration);
    bool read(Sensor::GyroscopeData* gyroscope);
    bool read(Sensor::MagneticData* magnetic);
    bool read(Sensor::PressureData* pressure);
    bool read(Can::Data::Encoder* encoder);
    Can::Data::Event readEvent();

private:
    static constexpr uint8_t SENSOR_BUFFER_SIZE = 4;
    static constexpr uint8_t ENCODER_BUFFER_SIZE = 8;
    static constexpr uint8_t EVENT_BUFFER_SIZE = 3;

    CAN_CREATE can_create{true};

    Sensor::AccelerometerData acceleration_buffer_[SENSOR_BUFFER_SIZE]{};
    Sensor::GyroscopeData gyroscope_buffer_[SENSOR_BUFFER_SIZE]{};
    Sensor::MagneticData magnetic_buffer_[SENSOR_BUFFER_SIZE]{};
    Sensor::PressureData pressure_buffer_[SENSOR_BUFFER_SIZE]{};
    Can::Data::Encoder encoder_buffer_[ENCODER_BUFFER_SIZE]{};
    Can::Data::Event event_buffer_[EVENT_BUFFER_SIZE]{};

    uint8_t acceleration_head_ = 0, acceleration_tail_ = 0, acceleration_count_ = 0;
    uint8_t gyroscope_head_ = 0, gyroscope_tail_ = 0, gyroscope_count_ = 0;
    uint8_t magnetic_head_ = 0, magnetic_tail_ = 0, magnetic_count_ = 0;
    uint8_t pressure_head_ = 0, pressure_tail_ = 0, pressure_count_ = 0;
    uint8_t encoder_head_ = 0, encoder_tail_ = 0, encoder_count_ = 0;
    uint8_t event_head_ = 0, event_tail_ = 0, event_count_ = 0;
    uint32_t encoder_sequence_ = 0;
};
