#pragma once

#include <stdint.h>

#include "domain/sensor/sensor_types.h"
#include "motion_types.h"

namespace Domain::Motion {

struct FlipConfig {
    uint64_t stale_us = 300000ULL;
    uint32_t confirm_ms = 500;
    float minimum_norm_g = 0.75f;
    float maximum_norm_g = 1.25f;
    float flipped_z_ratio = -0.50f;
    float upright_z_ratio = 0.50f;
};

struct FlipResult {
    FlipState state = FlipState::Unknown;
    Sensor::Source source = Sensor::Source::None;
    uint64_t timestamp_us = 0;
};

class FlipDetector {
public:
    explicit FlipDetector(const FlipConfig& config = FlipConfig{});
    void reset();
    FlipResult update(
        uint64_t now_us,
        const Sensor::AccelerometerData& body_acceleration);

private:
    FlipConfig config_;
    Sensor::Source source_ = Sensor::Source::None;
    FlipState stable_state_ = FlipState::Unknown;
    uint64_t flipped_since_us_ = 0;
    uint64_t upright_since_us_ = 0;
};

} // namespace Domain::Motion
