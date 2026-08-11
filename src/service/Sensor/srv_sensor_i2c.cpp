#include "srv_sensor_i2c.h"

#include <cstring>

namespace SensorService {
namespace {

constexpr TickType_t I2C_TIMEOUT = pdMS_TO_TICKS(20);
constexpr size_t MAX_WRITE_LENGTH = 64;

} // namespace

bool I2cBus::begin(
    i2c_port_t port,
    int sda_pin,
    int scl_pin,
    uint32_t clock_hz)
{
    port_ = port;
    i2c_config_t config{};
    config.mode = I2C_MODE_MASTER;
    config.sda_io_num = sda_pin;
    config.scl_io_num = scl_pin;
    config.sda_pullup_en = GPIO_PULLUP_DISABLE;
    config.scl_pullup_en = GPIO_PULLUP_DISABLE;
    config.master.clk_speed = clock_hz;
    config.clk_flags = 0;

    initialized_ =
        i2c_param_config(port_, &config) == ESP_OK &&
        i2c_driver_install(port_, I2C_MODE_MASTER, 0, 0, 0) == ESP_OK;
    return initialized_;
}

bool I2cBus::probe(uint8_t address) const
{
    if (!initialized_) return false;
    i2c_cmd_handle_t command = i2c_cmd_link_create();
    if (command == nullptr) return false;
    i2c_master_start(command);
    i2c_master_write_byte(command, static_cast<uint8_t>(address << 1U), true);
    i2c_master_stop(command);
    const bool ok =
        i2c_master_cmd_begin(port_, command, I2C_TIMEOUT) == ESP_OK;
    i2c_cmd_link_delete(command);
    return ok;
}

int32_t I2cBus::read(
    uint8_t address,
    uint8_t reg,
    uint8_t* data,
    size_t length) const
{
    if (!initialized_ || data == nullptr || length == 0U) return -1;
    return i2c_master_write_read_device(
        port_, address, &reg, 1, data, length, I2C_TIMEOUT) == ESP_OK
        ? 0 : -1;
}

int32_t I2cBus::write(
    uint8_t address,
    uint8_t reg,
    const uint8_t* data,
    size_t length) const
{
    if (!initialized_ || length > MAX_WRITE_LENGTH ||
        (length > 0U && data == nullptr)) return -1;
    uint8_t buffer[MAX_WRITE_LENGTH + 1U];
    buffer[0] = reg;
    if (length > 0U) memcpy(buffer + 1U, data, length);
    return i2c_master_write_to_device(
        port_, address, buffer, length + 1U, I2C_TIMEOUT) == ESP_OK
        ? 0 : -1;
}

} // namespace SensorService
