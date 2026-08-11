#include "flip_detector.h"
#include "domain/sensor/sensor_freshness.h"

#include <math.h>

namespace Domain::Motion {

FlipDetector::FlipDetector(const FlipConfig& config)
    : config_(config)
{
}

void FlipDetector::reset()
{
    source_ = Sensor::Source::None;
    stable_state_ = FlipState::Unknown;
    flipped_since_us_ = 0;
    upright_since_us_ = 0;
}

FlipResult FlipDetector::update(
    uint64_t now_us,
    const Sensor::AccelerometerData& acceleration)
{
    FlipResult result{};
    result.source = acceleration.metadata.source;
    result.timestamp_us = acceleration.metadata.received_us;
    const bool valid = Sensor::sampleIsFresh(
            acceleration.metadata, now_us, config_.stale_us) &&
        isfinite(acceleration.x_m_s2) &&
        isfinite(acceleration.y_m_s2) &&
        isfinite(acceleration.z_m_s2);
    if (!valid) {
        stable_state_ = FlipState::Unknown;
        flipped_since_us_ = 0;
        upright_since_us_ = 0;
        result.state = stable_state_;
        return result;
    }

    if (source_ != acceleration.metadata.source) {
        source_ = acceleration.metadata.source;
        stable_state_ = FlipState::Unknown;
        flipped_since_us_ = 0;
        upright_since_us_ = 0;
    }

    constexpr float GRAVITY_M_S2 = 9.80665f;
    const float x_g = acceleration.x_m_s2 / GRAVITY_M_S2;
    const float y_g = acceleration.y_m_s2 / GRAVITY_M_S2;
    const float z_g = acceleration.z_m_s2 / GRAVITY_M_S2;
    const float norm_g = sqrtf(x_g * x_g + y_g * y_g + z_g * z_g);
    if (!isfinite(norm_g) || norm_g < config_.minimum_norm_g ||
        norm_g > config_.maximum_norm_g) {
        stable_state_ = FlipState::Unknown;
        flipped_since_us_ = 0;
        upright_since_us_ = 0;
        result.state = stable_state_;
        return result;
    }

    const float z_ratio = z_g / norm_g;
    const uint64_t confirm_us =
        static_cast<uint64_t>(config_.confirm_ms) * 1000ULL;
    if (z_ratio <= config_.flipped_z_ratio) {
        upright_since_us_ = 0;
        if (flipped_since_us_ == 0U) flipped_since_us_ = now_us;
        if (now_us - flipped_since_us_ >= confirm_us) {
            stable_state_ = FlipState::Flipped;
        } else if (stable_state_ != FlipState::Flipped) {
            stable_state_ = FlipState::HighTilt;
        }
    } else if (z_ratio >= config_.upright_z_ratio) {
        flipped_since_us_ = 0;
        if (upright_since_us_ == 0U) upright_since_us_ = now_us;
        if (now_us - upright_since_us_ >= confirm_us) {
            stable_state_ = FlipState::Upright;
        }
    } else {
        flipped_since_us_ = 0;
        upright_since_us_ = 0;
        stable_state_ = FlipState::HighTilt;
    }
    result.state = stable_state_;
    return result;
}

} // namespace Domain::Motion
