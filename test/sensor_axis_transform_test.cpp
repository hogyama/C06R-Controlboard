#include <assert.h>
#include <math.h>

#include "platform/sensor_axis_transform.h"

namespace {

void expectNear(float actual, float expected)
{
    assert(fabsf(actual - expected) < 1.0e-6f);
}

} // namespace

int main()
{
    // IMU: 右、前、上 -> 機体: 前、左、上
    const SensorAxisTransform::Vector3 imu =
        SensorAxisTransform::imuToBody(1.0f, 2.0f, 3.0f);
    expectNear(imu.x, 2.0f);
    expectNear(imu.y, -1.0f);
    expectNear(imu.z, 3.0f);

    // 地磁気: 右、後ろ、下 -> 機体: 前、左、上
    const SensorAxisTransform::Vector3 magnetic =
        SensorAxisTransform::magneticToBody(1.0f, 2.0f, 3.0f);
    expectNear(magnetic.x, -2.0f);
    expectNear(magnetic.y, -1.0f);
    expectNear(magnetic.z, -3.0f);
    return 0;
}
