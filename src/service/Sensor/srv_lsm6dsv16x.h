#pragma once

#include "domain/sensor/sensor_types.h"
#include "service/Sensor/srv_sensor_i2c.h"

#include <lsm6dsv16x_reg.h>

namespace SensorService {

constexpr uint8_t LSM_MAX_GYROSCOPE_SAMPLES_PER_DRAIN = 40;

struct LsmDrainResult {
    Sensor::GyroscopeData
        gyroscope[LSM_MAX_GYROSCOPE_SAMPLES_PER_DRAIN]{};
    Sensor::AccelerometerData acceleration{};
    uint8_t gyroscope_count = 0;
    uint16_t gyroscope_fifo_count = 0;
    uint16_t accelerometer_fifo_count = 0;
    bool has_acceleration = false;
};

class Lsm6dsv16xService {
public:
    bool begin(I2cBus& bus);
    bool drain(LsmDrainResult& result, uint16_t max_frames = 32);
    uint16_t fifoOverflowCount() const { return fifo_overflow_count_; }

private:
    struct PendingVector {
        float x;
        float y;
        float z;
    };

    static int32_t readRegisters(
        void* handle,
        uint8_t reg,
        uint8_t* data,
        uint16_t length);
    static int32_t writeRegisters(
        void* handle,
        uint8_t reg,
        const uint8_t* data,
        uint16_t length);
    static void delayMs(uint32_t milliseconds);

    uint64_t hardwareTimestampUs(uint32_t raw_timestamp);
    void resetPending();
    void resolvePending(uint64_t tag_timestamp_us, LsmDrainResult& result);
    void appendAcceleration(
        const PendingVector& sample,
        uint64_t timestamp_us,
        LsmDrainResult& result);
    static uint64_t reconstructedTimestamp(
        uint64_t previous_tag_us,
        uint64_t tag_us,
        uint8_t index,
        uint8_t count,
        uint16_t sample_rate_hz);

    I2cDevice device_{};
    lsm6dsv16x_ctx_t context_{};
    PendingVector pending_gyroscope_[16]{};
    PendingVector pending_acceleration_[4]{};
    uint8_t pending_gyroscope_count_ = 0;
    uint8_t pending_acceleration_count_ = 0;
    float acceleration_sum_[3]{};
    uint8_t acceleration_average_count_ = 0;
    uint64_t acceleration_average_first_timestamp_us_ = 0;
    uint32_t timestamp_anchor_raw_ = 0;
    uint64_t timestamp_anchor_esp_us_ = 0;
    uint32_t last_timestamp_raw_ = 0;
    int64_t extended_timestamp_ticks_ = 0;
    bool timestamp_tag_seen_ = false;
    uint64_t previous_timestamp_tag_us_ = 0;
    uint16_t fifo_overflow_count_ = 0;
    bool overflow_active_ = false;
    uint64_t current_drain_received_us_ = 0;
};

} // namespace SensorService
