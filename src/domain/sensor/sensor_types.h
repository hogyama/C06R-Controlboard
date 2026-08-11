#pragma once

#include <stdint.h>

namespace Sensor {

enum class Source : uint8_t {
    None = 0,
    BoardI2c,
    Can,
    Gps
};

struct SampleMetadata {
    uint64_t timestamp_us;
    uint64_t received_us;
    Source source;
    bool valid;
};

struct GyroscopeData {
    float x_rad_s;
    float y_rad_s;
    float z_rad_s;
    SampleMetadata metadata;
};

struct AccelerometerData {
    float x_m_s2;
    float y_m_s2;
    float z_m_s2;
    SampleMetadata metadata;
};

struct MagneticData {
    float x_uT;
    float y_uT;
    float z_uT;
    SampleMetadata metadata;
};

struct PressureData {
    int32_t pressure_pa;
    SampleMetadata metadata;
};

struct AcquisitionStats {
    uint16_t gyro_samples;
    uint8_t accel_samples;
    uint8_t magnetic_samples;
    uint16_t fifo_overflow_count;
    SampleMetadata metadata;
};

} // namespace Sensor
