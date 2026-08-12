#pragma once

#include "domain/sensor/sensor_types.h"

namespace SensorAxisTransform {

struct Vector3 {
    float x;
    float y;
    float z;
};

// Domain body frame: X=forward, Y=left, Z=up.

// The modules on ControlBoard are mounted to match this body frame.
inline Vector3 boardImuToBody(float x, float y, float z)
{
    return {x, y, z};
}

// CAN SensorBoard ICM20948 frame: X=right, Y=forward, Z=up.
inline Vector3 canImuToBody(float x, float y, float z)
{
    return {y, -x, z};
}

inline Vector3 boardMagneticToBody(float x, float y, float z)
{
    // BMM350: X=left, Y=forward, Z=down.
    return {y, x, -z};
}

// CAN SensorBoard AK09916 frame: X=right, Y=backward, Z=down.
inline Vector3 canMagneticToBody(float x, float y, float z)
{
    return {-y, -x, -z};
}

inline Vector3 imuToBody(
    Sensor::Source source, float x, float y, float z)
{
    return source == Sensor::Source::Can
        ? canImuToBody(x, y, z)
        : boardImuToBody(x, y, z);
}

inline Vector3 magneticToBody(
    Sensor::Source source, float x, float y, float z)
{
    return source == Sensor::Source::Can
        ? canMagneticToBody(x, y, z)
        : boardMagneticToBody(x, y, z);
}

} // namespace SensorAxisTransform
