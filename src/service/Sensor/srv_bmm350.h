#pragma once

#include "domain/sensor/sensor_types.h"
#include "service/Sensor/srv_sensor_i2c.h"

#include <bmm350.h>

namespace SensorService {

enum class MagneticReadResult : uint8_t {
    Sample,
    NoData,
    Error
};

class Bmm350Service {
public:
    bool begin(I2cBus& bus, uint8_t address);
    MagneticReadResult read(Sensor::MagneticData& sample);

private:
    static BMM350_INTF_RET_TYPE readRegisters(
        uint8_t reg,
        uint8_t* data,
        uint32_t length,
        void* handle);
    static BMM350_INTF_RET_TYPE writeRegisters(
        uint8_t reg,
        const uint8_t* data,
        uint32_t length,
        void* handle);
    static void delayUs(uint32_t microseconds, void* handle);

    I2cDevice device_{};
    bmm350_dev sensor_{};
};

} // namespace SensorService
