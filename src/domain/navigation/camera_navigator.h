#pragma once

#include <stdint.h>

#include "domain/sensor/sensor_types.h"
#include "service/Can/srv_can.h"
#include "service/Rasp/srv_rasp.h"

namespace CameraNavigation {

struct Input {
    uint32_t now_ms = 0;
    uint64_t now_us = 0;
    bool has_camera = false;
    bool has_gyroscope = false;
    bool has_encoder = false;
    Rasp::CameraData camera{};
    Sensor::GyroscopeData gyroscope{};
    Can::Data::Encoder encoder{};
};

struct Output {
    float velocity_mm_s = 0.0f;
    float omega_rad_s = 0.0f;
    uint16_t duration_ms = 300;
    bool goal_reached = false;
};

void reset();
Output update(const Input& input);

} // namespace CameraNavigation
