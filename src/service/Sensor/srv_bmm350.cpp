#include "srv_bmm350.h"

#include <esp_timer.h>

namespace SensorService {

BMM350_INTF_RET_TYPE Bmm350Service::readRegisters(
    uint8_t reg,
    uint8_t* data,
    uint32_t length,
    void* handle)
{
    const auto* device = static_cast<I2cDevice*>(handle);
    return device->bus->read(device->address, reg, data, length) == 0
        ? BMM350_INTF_RET_SUCCESS : static_cast<BMM350_INTF_RET_TYPE>(-1);
}

BMM350_INTF_RET_TYPE Bmm350Service::writeRegisters(
    uint8_t reg,
    const uint8_t* data,
    uint32_t length,
    void* handle)
{
    const auto* device = static_cast<I2cDevice*>(handle);
    return device->bus->write(device->address, reg, data, length) == 0
        ? BMM350_INTF_RET_SUCCESS : static_cast<BMM350_INTF_RET_TYPE>(-1);
}

void Bmm350Service::delayUs(uint32_t microseconds, void*)
{
    if (microseconds >= 2000U) {
        vTaskDelay(pdMS_TO_TICKS(microseconds / 1000U));
        microseconds %= 1000U;
    }
    if (microseconds > 0U) delayMicroseconds(microseconds);
}

bool Bmm350Service::begin(I2cBus& bus, uint8_t address)
{
    device_.bus = &bus;
    device_.address = address;
    if (!bus.probe(address)) return false;

    sensor_ = {};
    sensor_.intfPtr = &device_;
    sensor_.read = readRegisters;
    sensor_.write = writeRegisters;
    sensor_.delayUs = delayUs;
    return
        bmm350Init(&sensor_) == BMM350_OK &&
        bmm350_enable_axes(
            BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, &sensor_) == BMM350_OK &&
        bmm350SetOdrPerformance(
            BMM350_DATA_RATE_100HZ,
            BMM350_AVERAGING_4,
            &sensor_) == BMM350_OK &&
        bmm350_enable_interrupt(
            BMM350_ENABLE_INTERRUPT,
            &sensor_) == BMM350_OK &&
        bmm350SetPowerMode(eBmm350NormalMode, &sensor_) == BMM350_OK;
}

MagneticReadResult Bmm350Service::read(Sensor::MagneticData& sample)
{
    uint8_t ready = 0;
    if (bmm350GetInterruptStatus(&ready, &sensor_) != BMM350_OK) {
        return MagneticReadResult::Error;
    }
    if (ready == 0U) return MagneticReadResult::NoData;

    sBmm350MagTempData_t value{};
    if (bmm350GetCompensatedMagXYZTempData(&value, &sensor_) != BMM350_OK) {
        return MagneticReadResult::Error;
    }
    sample.x_uT = value.x;
    sample.y_uT = value.y;
    sample.z_uT = value.z;
    const uint64_t now_us = static_cast<uint64_t>(esp_timer_get_time());
    sample.metadata.timestamp_us = now_us;
    sample.metadata.received_us = now_us;
    sample.metadata.source = Sensor::Source::BoardI2c;
    sample.metadata.valid = true;
    return MagneticReadResult::Sample;
}

} // namespace SensorService
