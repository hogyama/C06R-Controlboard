#pragma once

#include <Arduino.h>
#include <driver/i2c.h>

namespace SensorService {

class I2cBus {
public:
    bool begin(
        i2c_port_t port,
        int sda_pin,
        int scl_pin,
        uint32_t clock_hz);
    bool probe(uint8_t address) const;
    int32_t read(
        uint8_t address,
        uint8_t reg,
        uint8_t* data,
        size_t length) const;
    int32_t write(
        uint8_t address,
        uint8_t reg,
        const uint8_t* data,
        size_t length) const;

private:
    i2c_port_t port_ = I2C_NUM_0;
    bool initialized_ = false;
};

struct I2cDevice {
    I2cBus* bus = nullptr;
    uint8_t address = 0;
};

} // namespace SensorService
