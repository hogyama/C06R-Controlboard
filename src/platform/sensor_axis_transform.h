#pragma once

namespace SensorAxisTransform {

struct Vector3 {
    float x;
    float y;
    float z;
};

// IMU座標:
//   X: 右、Y: 前、Z: 上
//
// Domain機体座標:
//   X: 前、Y: 左、Z: 上
inline Vector3 imuToBody(float sensor_x, float sensor_y, float sensor_z)
{
    return {
        sensor_y,
        -sensor_x,
        sensor_z
    };
}

// 地磁気センサー座標:
//   X: 右、Y: 後ろ、Z: 下
//
// Domain機体座標:
//   X: 前、Y: 左、Z: 上
inline Vector3 magneticToBody(
    float sensor_x,
    float sensor_y,
    float sensor_z)
{
    return {
        -sensor_y,
        -sensor_x,
        -sensor_z
    };
}

} // namespace SensorAxisTransform
