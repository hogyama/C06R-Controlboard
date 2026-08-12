#include "tasks.h"
#include "app_queue.h"
#include "platform/board_config.h"
#include "platform/pin_config.h"
#include "service/Sensor/srv_bmm350.h"
#include "service/Sensor/srv_lsm6dsv16x.h"
#include "service/Sensor/srv_sensor_i2c.h"
#include "domain/sensor/sensor_freshness.h"

#include <Arduino.h>
#include <driver/i2c.h>
#include <esp_timer.h>
#include <math.h>

namespace {

SensorService::I2cBus sensor_bus;
SensorService::Lsm6dsv16xService imu_sensor;
SensorService::Bmm350Service magnetic_sensor;

class BoardPrimarySelector {
public:
    Sensor::Source select(bool board_valid, bool can_valid)
    {
        if (active_ == Sensor::Source::BoardI2c) {
            if (board_valid) return active_;
            recovery_count_ = 0;
            active_ = can_valid ? Sensor::Source::Can : Sensor::Source::None;
            return active_;
        }
        if (active_ == Sensor::Source::Can) {
            if (board_valid) {
                if (recovery_count_ < 10U) ++recovery_count_;
                if (!can_valid || recovery_count_ >= 10U) {
                    active_ = Sensor::Source::BoardI2c;
                    recovery_count_ = 0;
                }
            } else {
                recovery_count_ = 0;
            }
            if (active_ == Sensor::Source::Can && !can_valid) {
                active_ = Sensor::Source::None;
            }
            return active_;
        }
        recovery_count_ = 0;
        active_ = board_valid
            ? Sensor::Source::BoardI2c
            : (can_valid ? Sensor::Source::Can : Sensor::Source::None);
        return active_;
    }

private:
    Sensor::Source active_ = Sensor::Source::None;
    uint8_t recovery_count_ = 0;
};

bool finiteAcceleration(const Sensor::AccelerometerData& value)
{
    return isfinite(value.x_m_s2) && isfinite(value.y_m_s2) &&
        isfinite(value.z_m_s2);
}

bool finiteGyroscope(const Sensor::GyroscopeData& value)
{
    return isfinite(value.x_rad_s) && isfinite(value.y_rad_s) &&
        isfinite(value.z_rad_s);
}

bool finiteMagnetic(const Sensor::MagneticData& value)
{
    return isfinite(value.x_uT) && isfinite(value.y_uT) &&
        isfinite(value.z_uT);
}

uint16_t addU16(uint16_t value, uint16_t increment)
{
    const uint32_t sum = static_cast<uint32_t>(value) + increment;
    return sum > UINT16_MAX ? UINT16_MAX : static_cast<uint16_t>(sum);
}

uint8_t addU8(uint8_t value, uint16_t increment)
{
    const uint16_t sum = static_cast<uint16_t>(value) + increment;
    return sum > UINT8_MAX ? UINT8_MAX : static_cast<uint8_t>(sum);
}

void publishSelectedSensors(uint64_t now_us)
{
    static BoardPrimarySelector acceleration_selector;
    static BoardPrimarySelector gyroscope_selector;
    static BoardPrimarySelector magnetic_selector;
    Sensor::AccelerometerData board_acceleration{};
    Sensor::AccelerometerData can_acceleration{};
    const bool board_acceleration_valid =
        xQueuePeek(
            mbx_board_acceleration, &board_acceleration, 0) == pdTRUE &&
        Sensor::sampleIsFresh(
            board_acceleration.metadata, now_us, IMU_TIMEOUT_MS * 1000ULL) &&
        finiteAcceleration(board_acceleration);
    const bool can_acceleration_valid =
        xQueuePeek(mbx_can_acceleration, &can_acceleration, 0) == pdTRUE &&
        Sensor::sampleIsFresh(
            can_acceleration.metadata, now_us, IMU_TIMEOUT_MS * 1000ULL) &&
        finiteAcceleration(can_acceleration);
    const Sensor::Source acceleration_source = acceleration_selector.select(
        board_acceleration_valid, can_acceleration_valid);
    if (acceleration_source == Sensor::Source::BoardI2c) {
        xQueueOverwrite(mbx_acceleration, &board_acceleration);
    } else if (acceleration_source == Sensor::Source::Can) {
        xQueueOverwrite(mbx_acceleration, &can_acceleration);
    }

    Sensor::GyroscopeData board_gyroscope{};
    Sensor::GyroscopeData can_gyroscope{};
    const bool board_gyroscope_valid =
        xQueuePeek(mbx_board_gyroscope, &board_gyroscope, 0) == pdTRUE &&
        Sensor::sampleIsFresh(
            board_gyroscope.metadata, now_us, IMU_TIMEOUT_MS * 1000ULL) &&
        finiteGyroscope(board_gyroscope);
    const bool can_gyroscope_valid =
        xQueuePeek(mbx_can_gyroscope, &can_gyroscope, 0) == pdTRUE &&
        Sensor::sampleIsFresh(
            can_gyroscope.metadata, now_us, IMU_TIMEOUT_MS * 1000ULL) &&
        finiteGyroscope(can_gyroscope);
    const Sensor::Source gyroscope_source = gyroscope_selector.select(
        board_gyroscope_valid, can_gyroscope_valid);
    if (gyroscope_source == Sensor::Source::BoardI2c) {
        xQueueOverwrite(mbx_gyroscope, &board_gyroscope);
    } else if (gyroscope_source == Sensor::Source::Can) {
        xQueueOverwrite(mbx_gyroscope, &can_gyroscope);
    }

    Sensor::MagneticData board_magnetic{};
    Sensor::MagneticData can_magnetic{};
    const bool board_magnetic_valid =
        xQueuePeek(mbx_board_magnetic, &board_magnetic, 0) == pdTRUE &&
        Sensor::sampleIsFresh(
            board_magnetic.metadata, now_us, MAGNETIC_TIMEOUT_MS * 1000ULL) &&
        finiteMagnetic(board_magnetic);
    const bool can_magnetic_valid =
        xQueuePeek(mbx_can_magnetic, &can_magnetic, 0) == pdTRUE &&
        Sensor::sampleIsFresh(
            can_magnetic.metadata, now_us, MAGNETIC_TIMEOUT_MS * 1000ULL) &&
        finiteMagnetic(can_magnetic);
    const Sensor::Source magnetic_source = magnetic_selector.select(
        board_magnetic_valid, can_magnetic_valid);
    if (magnetic_source == Sensor::Source::BoardI2c) {
        xQueueOverwrite(mbx_magnetic, &board_magnetic);
    } else if (magnetic_source == Sensor::Source::Can) {
        xQueueOverwrite(mbx_magnetic, &can_magnetic);
    }
}

} // namespace

void taskSensor(void* pvParameters)
{
    (void)pvParameters;
    const bool bus_ready = sensor_bus.begin(
        I2C_NUM_0, I2C_SDA, I2C_SCL, I2C_CLOCK_HZ);
    bool imu_ready = bus_ready && imu_sensor.begin(sensor_bus);
    if (imu_ready) xQueueReset(fifo_board_gyroscope);
    bool magnetic_ready =
        bus_ready && magnetic_sensor.begin(sensor_bus, BMM350_I2C_ADDRESS);

    Sensor::AcquisitionStats stats{};
    Sensor::GyroscopeData previous_stats_gyro{};
    bool have_previous_stats_gyro = false;
    bool have_stats_gyro = false;
    uint32_t last_imu_retry_ms = millis();
    uint32_t last_magnetic_retry_ms = millis();
    uint32_t last_magnetic_poll_ms = millis();
    uint32_t last_stats_ms = millis();
    TickType_t last_wake = xTaskGetTickCount();

    while (true) {
        xTaskDelayUntil(
            &last_wake, pdMS_TO_TICKS(SENSOR_POLL_PERIOD_MS));
        const uint32_t now_ms = millis();

        if (!imu_ready && bus_ready &&
            static_cast<uint32_t>(now_ms - last_imu_retry_ms) >= 1000U) {
            last_imu_retry_ms = now_ms;
            imu_ready = imu_sensor.begin(sensor_bus);
            if (imu_ready) xQueueReset(fifo_board_gyroscope);
        }
        if (imu_ready) {
            SensorService::LsmDrainResult result{};
            if (!imu_sensor.drain(result)) {
                imu_ready = false;
                last_imu_retry_ms = now_ms;
            } else {
                stats.gyro_samples = addU16(
                    stats.gyro_samples, result.gyroscope_fifo_count);
                stats.accel_samples = addU8(
                    stats.accel_samples, result.accelerometer_fifo_count);
                for (uint8_t index = 0;
                     index < result.gyroscope_count;
                     ++index) {
                    const Sensor::GyroscopeData& sample =
                        result.gyroscope[index];
                    if (!finiteGyroscope(sample)) continue;
                    stats.latest_gyro_x_rad_s = sample.x_rad_s;
                    stats.latest_gyro_y_rad_s = sample.y_rad_s;
                    stats.latest_gyro_z_rad_s = sample.z_rad_s;
                    if (!have_stats_gyro) {
                        stats.minimum_gyro_z_rad_s = sample.z_rad_s;
                        stats.maximum_gyro_z_rad_s = sample.z_rad_s;
                        have_stats_gyro = true;
                    } else {
                        stats.minimum_gyro_z_rad_s = fminf(
                            stats.minimum_gyro_z_rad_s, sample.z_rad_s);
                        stats.maximum_gyro_z_rad_s = fmaxf(
                            stats.maximum_gyro_z_rad_s, sample.z_rad_s);
                    }
                    if (have_previous_stats_gyro &&
                        sample.metadata.timestamp_us >
                            previous_stats_gyro.metadata.timestamp_us) {
                        const float dt_s = static_cast<float>(
                            sample.metadata.timestamp_us -
                            previous_stats_gyro.metadata.timestamp_us) * 1.0e-6f;
                        stats.integrated_gyro_z_rad += 0.5f *
                            (previous_stats_gyro.z_rad_s + sample.z_rad_s) * dt_s;
                    }
                    previous_stats_gyro = sample;
                    have_previous_stats_gyro = true;
                    pushGyroscopeRing(fifo_board_gyroscope, sample);
                    pushGyroscopeRing(fifo_stuck_board_gyroscope, sample);
                    xQueueOverwrite(mbx_board_gyroscope, &sample);
                }
                if (result.has_acceleration &&
                    finiteAcceleration(result.acceleration)) {
                    xQueueOverwrite(
                        mbx_board_acceleration,
                        &result.acceleration);
                }
            }
        }

        if (!magnetic_ready && bus_ready &&
            static_cast<uint32_t>(now_ms - last_magnetic_retry_ms) >= 1000U) {
            last_magnetic_retry_ms = now_ms;
            magnetic_ready =
                magnetic_sensor.begin(sensor_bus, BMM350_I2C_ADDRESS);
        }
        if (magnetic_ready &&
            static_cast<uint32_t>(now_ms - last_magnetic_poll_ms) >=
                MAGNETIC_POLL_PERIOD_MS) {
            last_magnetic_poll_ms = now_ms;
            Sensor::MagneticData sample{};
            const auto result = magnetic_sensor.read(sample);
            if (result == SensorService::MagneticReadResult::Sample) {
                stats.magnetic_samples = addU8(stats.magnetic_samples, 1U);
                if (finiteMagnetic(sample)) {
                    xQueueOverwrite(mbx_board_magnetic, &sample);
                }
            } else if (result == SensorService::MagneticReadResult::Error) {
                magnetic_ready = false;
                last_magnetic_retry_ms = now_ms;
            }
        }

        if (static_cast<uint32_t>(now_ms - last_stats_ms) >=
            SENSOR_STATS_PERIOD_MS) {
            last_stats_ms = now_ms;
            stats.fifo_overflow_count = imu_sensor.fifoOverflowCount();
            const uint64_t now_us =
                static_cast<uint64_t>(esp_timer_get_time());
            stats.metadata = {
                now_us, now_us, Sensor::Source::BoardI2c, true};
            xQueueOverwrite(mbx_sensor_acquisition_stats, &stats);
            stats.gyro_samples = 0;
            stats.accel_samples = 0;
            stats.magnetic_samples = 0;
            stats.integrated_gyro_z_rad = 0.0f;
            stats.minimum_gyro_z_rad_s = 0.0f;
            stats.maximum_gyro_z_rad_s = 0.0f;
            have_previous_stats_gyro = false;
            have_stats_gyro = false;
        }

        publishSelectedSensors(static_cast<uint64_t>(esp_timer_get_time()));
    }
}
