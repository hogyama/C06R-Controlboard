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
    // BOARD IMU is aligned; CAN IMU uses the legacy SensorBoard frame.
    const SensorAxisTransform::Vector3 board_imu =
        SensorAxisTransform::boardImuToBody(1.0f, 2.0f, 3.0f);
    expectNear(board_imu.x, 1.0f);
    expectNear(board_imu.y, 2.0f);
    expectNear(board_imu.z, 3.0f);

    const SensorAxisTransform::Vector3 can_imu =
        SensorAxisTransform::canImuToBody(1.0f, 2.0f, 3.0f);
    expectNear(can_imu.x, 2.0f);
    expectNear(can_imu.y, -1.0f);
    expectNear(can_imu.z, 3.0f);

    // BOARD BMM350: left, forward, down -> body: forward, left, up.
    const SensorAxisTransform::Vector3 board_magnetic =
        SensorAxisTransform::boardMagneticToBody(1.0f, 2.0f, 3.0f);
    expectNear(board_magnetic.x, 2.0f);
    expectNear(board_magnetic.y, 1.0f);
    expectNear(board_magnetic.z, -3.0f);

    const SensorAxisTransform::Vector3 can_magnetic =
        SensorAxisTransform::canMagneticToBody(1.0f, 2.0f, 3.0f);
    expectNear(can_magnetic.x, -2.0f);
    expectNear(can_magnetic.y, -1.0f);
    expectNear(can_magnetic.z, -3.0f);
    return 0;
}
