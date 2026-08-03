#pragma once

#include <stdint.h>

namespace FieldConfig {

// The previous GPS origin was the centre goal.  The south-west field corner
// is 30 m south and 30 m west of that surveyed point.
constexpr int32_t ORIGIN_LATITUDE_E7 = 356048841;
constexpr int32_t ORIGIN_LONGITUDE_E7 = 1396820205;
constexpr double ORIGIN_LATITUDE_DEG =
    static_cast<double>(ORIGIN_LATITUDE_E7) * 1.0e-7;
constexpr double ORIGIN_LONGITUDE_DEG =
    static_cast<double>(ORIGIN_LONGITUDE_E7) * 1.0e-7;

constexpr int32_t SIZE_X_MM = 60000;
constexpr int32_t SIZE_Y_MM = 60000;
constexpr int32_t GOAL_X_MM = SIZE_X_MM / 2;
constexpr int32_t GOAL_Y_MM = SIZE_Y_MM / 2;

// WMM2025, 2026-07, Tokyo field. East-positive; 7.92 degrees west.
// Re-check this value when the field location or model epoch changes.
constexpr float MAGNETIC_DECLINATION_RAD = -0.1382300768f;

// A 64 x 64 map with 1 m cells leaves a 2 m margin around the 60 m field.
constexpr float MAP_CELL_SIZE_MM = 1000.0f;
constexpr float MAP_ORIGIN_X_MM = -2000.0f;
constexpr float MAP_ORIGIN_Y_MM = -2000.0f;

} // namespace FieldConfig
