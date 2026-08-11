#include "srv_lsm6dsv16x.h"

#include <esp_timer.h>
#include <math.h>

namespace SensorService {
namespace {

constexpr uint8_t ADDRESS_LOW = 0x6A;
constexpr uint8_t ADDRESS_HIGH = 0x6B;
constexpr uint8_t FIFO_WATERMARK = 8;
constexpr uint16_t GYROSCOPE_RATE_HZ = 1920;
constexpr uint16_t ACCELEROMETER_RATE_HZ = 240;
constexpr int64_t TIMESTAMP_TICK_NS = 21750;
constexpr float MDPS_TO_RAD_S = static_cast<float>(M_PI) / 180000.0f;
constexpr float STANDARD_GRAVITY_M_S2 = 9.80665f;

int16_t i16(const uint8_t* data)
{
    return static_cast<int16_t>(
        static_cast<uint16_t>(data[0]) |
        (static_cast<uint16_t>(data[1]) << 8U));
}

uint32_t u32(const uint8_t* data)
{
    return static_cast<uint32_t>(data[0]) |
        (static_cast<uint32_t>(data[1]) << 8U) |
        (static_cast<uint32_t>(data[2]) << 16U) |
        (static_cast<uint32_t>(data[3]) << 24U);
}

} // namespace

int32_t Lsm6dsv16xService::readRegisters(
    void* handle,
    uint8_t reg,
    uint8_t* data,
    uint16_t length)
{
    const auto* device = static_cast<I2cDevice*>(handle);
    return device->bus->read(device->address, reg, data, length);
}

int32_t Lsm6dsv16xService::writeRegisters(
    void* handle,
    uint8_t reg,
    const uint8_t* data,
    uint16_t length)
{
    const auto* device = static_cast<I2cDevice*>(handle);
    return device->bus->write(device->address, reg, data, length);
}

void Lsm6dsv16xService::delayMs(uint32_t milliseconds)
{
    vTaskDelay(pdMS_TO_TICKS(milliseconds));
}

bool Lsm6dsv16xService::begin(I2cBus& bus)
{
    device_.bus = &bus;
    if (bus.probe(ADDRESS_LOW)) {
        device_.address = ADDRESS_LOW;
    } else if (bus.probe(ADDRESS_HIGH)) {
        device_.address = ADDRESS_HIGH;
    } else {
        return false;
    }

    context_.read_reg = readRegisters;
    context_.write_reg = writeRegisters;
    context_.mdelay = delayMs;
    context_.handle = &device_;

    uint8_t id = 0;
    if (lsm6dsv16x_device_id_get(&context_, &id) != 0 ||
        id != LSM6DSV16X_ID ||
        lsm6dsv16x_reset_set(&context_, LSM6DSV16X_GLOBAL_RST) != 0) {
        return false;
    }

    lsm6dsv16x_reset_t reset = LSM6DSV16X_GLOBAL_RST;
    for (uint8_t retry = 0; retry < 100 && reset != LSM6DSV16X_READY; ++retry) {
        delayMs(1);
        if (lsm6dsv16x_reset_get(&context_, &reset) != 0) return false;
    }
    if (reset != LSM6DSV16X_READY) return false;

    const bool ok =
        lsm6dsv16x_auto_increment_set(&context_, PROPERTY_ENABLE) == 0 &&
        lsm6dsv16x_block_data_update_set(&context_, PROPERTY_ENABLE) == 0 &&
        lsm6dsv16x_xl_full_scale_set(&context_, LSM6DSV16X_4g) == 0 &&
        lsm6dsv16x_gy_full_scale_set(&context_, LSM6DSV16X_2000dps) == 0 &&
        lsm6dsv16x_xl_data_rate_set(&context_, LSM6DSV16X_ODR_AT_240Hz) == 0 &&
        lsm6dsv16x_gy_data_rate_set(&context_, LSM6DSV16X_ODR_AT_1920Hz) == 0 &&
        lsm6dsv16x_timestamp_set(&context_, PROPERTY_ENABLE) == 0 &&
        lsm6dsv16x_fifo_mode_set(&context_, LSM6DSV16X_BYPASS_MODE) == 0 &&
        lsm6dsv16x_fifo_watermark_set(&context_, FIFO_WATERMARK) == 0 &&
        lsm6dsv16x_fifo_xl_batch_set(
            &context_, LSM6DSV16X_XL_BATCHED_AT_240Hz) == 0 &&
        lsm6dsv16x_fifo_gy_batch_set(
            &context_, LSM6DSV16X_GY_BATCHED_AT_1920Hz) == 0 &&
        lsm6dsv16x_fifo_timestamp_batch_set(
            &context_, LSM6DSV16X_TMSTMP_DEC_8) == 0 &&
        lsm6dsv16x_fifo_mode_set(&context_, LSM6DSV16X_STREAM_MODE) == 0;
    if (!ok) return false;

    const int64_t before_us = esp_timer_get_time();
    uint32_t timestamp_raw = 0;
    if (lsm6dsv16x_timestamp_raw_get(&context_, &timestamp_raw) != 0) {
        return false;
    }
    const int64_t after_us = esp_timer_get_time();
    timestamp_anchor_raw_ = timestamp_raw;
    timestamp_anchor_esp_us_ = static_cast<uint64_t>(
        before_us + (after_us - before_us) / 2);
    last_timestamp_raw_ = timestamp_raw;
    extended_timestamp_ticks_ = 0;
    timestamp_tag_seen_ = false;
    resetPending();
    overflow_active_ = false;
    return true;
}

uint64_t Lsm6dsv16xService::hardwareTimestampUs(
    uint32_t raw_timestamp)
{
    if (!timestamp_tag_seen_) {
        extended_timestamp_ticks_ = static_cast<int32_t>(
            raw_timestamp - timestamp_anchor_raw_);
        timestamp_tag_seen_ = true;
    } else {
        extended_timestamp_ticks_ += static_cast<uint32_t>(
            raw_timestamp - last_timestamp_raw_);
    }
    last_timestamp_raw_ = raw_timestamp;
    const int64_t timestamp_us =
        static_cast<int64_t>(timestamp_anchor_esp_us_) +
        extended_timestamp_ticks_ * TIMESTAMP_TICK_NS / 1000LL;
    return timestamp_us > 0 ? static_cast<uint64_t>(timestamp_us) : 0U;
}

void Lsm6dsv16xService::resetPending()
{
    pending_gyroscope_count_ = 0;
    pending_acceleration_count_ = 0;
    acceleration_sum_[0] = 0.0f;
    acceleration_sum_[1] = 0.0f;
    acceleration_sum_[2] = 0.0f;
    acceleration_average_count_ = 0;
    acceleration_average_first_timestamp_us_ = 0;
    previous_timestamp_tag_us_ = 0;
}

uint64_t Lsm6dsv16xService::reconstructedTimestamp(
    uint64_t previous_tag_us,
    uint64_t tag_us,
    uint8_t index,
    uint8_t count,
    uint16_t sample_rate_hz)
{
    if (count == 0U) return tag_us;
    if (previous_tag_us != 0U && tag_us > previous_tag_us) {
        return previous_tag_us +
            (tag_us - previous_tag_us) * (index + 1U) / count;
    }
    const uint64_t samples_after = count - index - 1U;
    const uint64_t offset_us =
        samples_after * 1000000ULL / sample_rate_hz;
    return tag_us > offset_us ? tag_us - offset_us : 0U;
}

void Lsm6dsv16xService::appendAcceleration(
    const PendingVector& sample,
    uint64_t timestamp_us,
    LsmDrainResult& result)
{
    if (acceleration_average_count_ == 0U) {
        acceleration_average_first_timestamp_us_ = timestamp_us;
    }
    acceleration_sum_[0] += sample.x;
    acceleration_sum_[1] += sample.y;
    acceleration_sum_[2] += sample.z;
    ++acceleration_average_count_;
    if (acceleration_average_count_ < 4U) return;

    result.acceleration.x_m_s2 =
        acceleration_sum_[0] * 0.25f * STANDARD_GRAVITY_M_S2;
    result.acceleration.y_m_s2 =
        acceleration_sum_[1] * 0.25f * STANDARD_GRAVITY_M_S2;
    result.acceleration.z_m_s2 =
        acceleration_sum_[2] * 0.25f * STANDARD_GRAVITY_M_S2;
    result.acceleration.metadata.timestamp_us =
        acceleration_average_first_timestamp_us_ +
        (timestamp_us - acceleration_average_first_timestamp_us_) / 2ULL;
    result.acceleration.metadata.received_us = current_drain_received_us_;
    result.acceleration.metadata.source = Sensor::Source::BoardI2c;
    result.acceleration.metadata.valid = true;
    result.has_acceleration = true;

    acceleration_sum_[0] = 0.0f;
    acceleration_sum_[1] = 0.0f;
    acceleration_sum_[2] = 0.0f;
    acceleration_average_count_ = 0;
    acceleration_average_first_timestamp_us_ = 0;
}

void Lsm6dsv16xService::resolvePending(
    uint64_t tag_timestamp_us,
    LsmDrainResult& result)
{
    for (uint8_t index = 0; index < pending_gyroscope_count_; ++index) {
        if (result.gyroscope_count >=
            LSM_MAX_GYROSCOPE_SAMPLES_PER_DRAIN) break;
        const PendingVector& pending = pending_gyroscope_[index];
        Sensor::GyroscopeData& sample =
            result.gyroscope[result.gyroscope_count++];
        sample.x_rad_s = pending.x;
        sample.y_rad_s = pending.y;
        sample.z_rad_s = pending.z;
        sample.metadata.timestamp_us = reconstructedTimestamp(
            previous_timestamp_tag_us_,
            tag_timestamp_us,
            index,
            pending_gyroscope_count_,
            GYROSCOPE_RATE_HZ);
        sample.metadata.received_us = current_drain_received_us_;
        sample.metadata.source = Sensor::Source::BoardI2c;
        sample.metadata.valid = true;
    }

    for (uint8_t index = 0; index < pending_acceleration_count_; ++index) {
        appendAcceleration(
            pending_acceleration_[index],
            reconstructedTimestamp(
                previous_timestamp_tag_us_,
                tag_timestamp_us,
                index,
                pending_acceleration_count_,
                ACCELEROMETER_RATE_HZ),
            result);
    }

    pending_gyroscope_count_ = 0;
    pending_acceleration_count_ = 0;
    previous_timestamp_tag_us_ = tag_timestamp_us;
}

bool Lsm6dsv16xService::drain(
    LsmDrainResult& result,
    uint16_t max_frames)
{
    result = {};
    current_drain_received_us_ =
        static_cast<uint64_t>(esp_timer_get_time());
    lsm6dsv16x_fifo_status_t status{};
    if (lsm6dsv16x_fifo_status_get(&context_, &status) != 0) return false;

    if (status.fifo_ovr && !overflow_active_ && fifo_overflow_count_ != UINT16_MAX) {
        ++fifo_overflow_count_;
        resetPending();
    }
    overflow_active_ = status.fifo_ovr;

    const uint16_t count = status.fifo_level < max_frames
        ? status.fifo_level : max_frames;
    lsm6dsv16x_fifo_out_raw_t raw{};
    for (uint16_t index = 0; index < count; ++index) {
        if (lsm6dsv16x_fifo_out_raw_get(&context_, &raw) != 0) return false;
        if (raw.tag == lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_TIMESTAMP_TAG) {
            resolvePending(hardwareTimestampUs(u32(raw.data)), result);
            continue;
        }

        const int16_t x = i16(&raw.data[0]);
        const int16_t y = i16(&raw.data[2]);
        const int16_t z = i16(&raw.data[4]);

        if (raw.tag == lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_GY_NC_TAG) {
            ++result.gyroscope_fifo_count;
            if (pending_gyroscope_count_ >= 16U) {
                resetPending();
            }
            PendingVector& sample =
                pending_gyroscope_[pending_gyroscope_count_++];
            sample.x =
                lsm6dsv16x_from_fs2000_to_mdps(x) * MDPS_TO_RAD_S;
            sample.y =
                lsm6dsv16x_from_fs2000_to_mdps(y) * MDPS_TO_RAD_S;
            sample.z =
                lsm6dsv16x_from_fs2000_to_mdps(z) * MDPS_TO_RAD_S;
        } else if (
            raw.tag == lsm6dsv16x_fifo_out_raw_t::LSM6DSV16X_XL_NC_TAG) {
            ++result.accelerometer_fifo_count;
            if (pending_acceleration_count_ >= 4U) {
                resetPending();
            }
            PendingVector& sample =
                pending_acceleration_[pending_acceleration_count_++];
            sample.x =
                lsm6dsv16x_from_fs4_to_mg(x) * 0.001f;
            sample.y =
                lsm6dsv16x_from_fs4_to_mg(y) * 0.001f;
            sample.z =
                lsm6dsv16x_from_fs4_to_mg(z) * 0.001f;
        }
    }
    return true;
}

} // namespace SensorService
