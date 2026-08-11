#include <assert.h>
#include <stdint.h>

#include "domain/motion/stuck_detector.h"

namespace {

using Domain::Motion::Assessment;
using Domain::Motion::DetectorSample;
using Domain::Motion::Reason;
using Domain::Motion::StuckDetector;

DetectorSample baseSample(uint32_t timestamp_ms)
{
    DetectorSample sample{};
    sample.timestamp_ms = timestamp_ms;
    sample.dt_ms = 100;
    sample.navigation_active = true;
    sample.command_valid = true;
    sample.command_velocity_mm_s = 500.0f;
    sample.encoder_available = true;
    sample.encoder_left_mm = static_cast<int32_t>(timestamp_ms / 2U);
    sample.encoder_right_mm = static_cast<int32_t>(timestamp_ms / 2U);
    sample.encoder_left_velocity_mm_s = 500.0f;
    sample.encoder_right_velocity_mm_s = 500.0f;
    sample.gyro_available = true;
    sample.gyro_yaw_rate_rad_s = 0.0f;
    sample.acceleration_available = true;
    sample.acceleration_z_g = 1.0f;
    return sample;
}

void assertOnly(const Assessment& result, Reason reason)
{
    assert(result.reason == reason);
    if (reason == Reason::WheelBlocked) {
        assert(result.scores.wheel_blocked == 1000U);
        assert(result.scores.wheel_slip < 1000U);
        assert(result.scores.rotation_blocked < 1000U);
    } else if (reason == Reason::WheelSlip) {
        assert(result.scores.wheel_slip == 1000U);
        assert(result.scores.wheel_blocked < 1000U);
        assert(result.scores.rotation_blocked < 1000U);
    } else if (reason == Reason::RotationBlocked) {
        assert(result.scores.rotation_blocked == 1000U);
        assert(result.scores.wheel_blocked < 1000U);
        assert(result.scores.wheel_slip < 1000U);
    }
}

void testWheelBlocked()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 3000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.encoder_left_velocity_mm_s = 0.0f;
        sample.encoder_right_velocity_mm_s = 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assertOnly(result, Reason::WheelBlocked);
}

void testOneWheelBlocked()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 3000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.encoder_left_velocity_mm_s = 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assertOnly(result, Reason::WheelBlocked);
}

void testWrongDirectionTriggersWheelBlocked()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 1200; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.encoder_left_velocity_mm_s = -500.0f;
        sample.encoder_right_velocity_mm_s = -500.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assertOnly(result, Reason::WheelBlocked);
}

void testHealthyLowSpeedRotationDoesNotTrigger()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 5000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 0.0f;
        sample.command_yaw_rate_rad_s = 0.4f;
        sample.encoder_left_velocity_mm_s = -36.0f;
        sample.encoder_right_velocity_mm_s = 36.0f;
        sample.gyro_yaw_rate_rad_s = 0.4f;
        result = detector.update(sample);
    }
    assert(!result.suspend_requested);
    assert(result.scores.rotation_blocked == 0U);
}

void testRotationBlockedWithGyro()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 3000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 0.0f;
        sample.command_yaw_rate_rad_s = 1.0f;
        sample.encoder_left_velocity_mm_s = -90.0f;
        sample.encoder_right_velocity_mm_s = 90.0f;
        sample.gyro_yaw_rate_rad_s = 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assertOnly(result, Reason::RotationBlocked);
}

void testRotationBlockedWithoutGyroWhenWheelsLocked()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 3000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 0.0f;
        sample.command_yaw_rate_rad_s = 0.5f;
        sample.encoder_left_velocity_mm_s = 0.0f;
        sample.encoder_right_velocity_mm_s = 0.0f;
        sample.gyro_available = false;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assertOnly(result, Reason::RotationBlocked);
}

void testGpsVelocityMismatchTriggersSlip()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 4000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.gps_available = true;
        sample.gps_velocity_available = true;
        sample.gps_updated = (now % 1000U) == 0U;
        sample.gps_horizontal_accuracy_mm = 1000U;
        sample.gps_speed_accuracy_mm_s = 10.0f;
        sample.gps_x_mm = 10000;
        sample.gps_y_mm = 10000;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assertOnly(result, Reason::WheelSlip);
}

void testGpsDistanceMismatchTriggersSlip()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 8000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.gps_available = true;
        sample.gps_updated = (now % 1000U) == 0U;
        sample.gps_horizontal_accuracy_mm = 1000U;
        sample.gps_x_mm = 10000;
        sample.gps_y_mm = 10000;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assertOnly(result, Reason::WheelSlip);
}

void testNoGpsEventuallyRequestsSlipVerification()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 9000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 220.0f;
        sample.encoder_left_velocity_mm_s = 220.0f;
        sample.encoder_right_velocity_mm_s = 220.0f;
        sample.encoder_left_mm = static_cast<int32_t>(now * 22U / 100U);
        sample.encoder_right_mm = sample.encoder_left_mm;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assertOnly(result, Reason::WheelSlip);
}

void testAccelerationImpactAcceleratesFallback()
{
    StuckDetector detector{};
    Assessment result{};
    uint32_t detected_ms = 0;
    for (uint32_t now = 100; now <= 5000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.acceleration_x_g = now >= 200U ? 0.4f : 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) {
            detected_ms = now;
            break;
        }
    }
    assert(result.suspend_requested);
    assertOnly(result, Reason::WheelSlip);
    assert(detected_ms < 6000U);
}

void testEncoderGyroMismatchTriggersSlipWhileSteering()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 3000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 200.0f;
        sample.command_yaw_rate_rad_s = -0.4f;
        sample.encoder_left_velocity_mm_s = 230.0f;
        sample.encoder_right_velocity_mm_s = 150.0f;
        sample.gyro_yaw_rate_rad_s = 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assertOnly(result, Reason::WheelSlip);
}

void testHealthyGpsVelocityDoesNotTriggerSlip()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 10000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.gps_available = true;
        sample.gps_velocity_available = true;
        sample.gps_updated = (now % 1000U) == 0U;
        sample.gps_horizontal_accuracy_mm = 1000U;
        sample.gps_velocity_east_mm_s = 500.0f;
        sample.gps_speed_accuracy_mm_s = 20.0f;
        sample.gps_x_mm = static_cast<int32_t>(now / 2U);
        sample.gps_y_mm = 0;
        result = detector.update(sample);
    }
    assert(!result.suspend_requested);
    assert(result.scores.wheel_slip < 1000U);
}

void testVerificationCompletionRearmsReason()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 3000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.encoder_left_velocity_mm_s = 0.0f;
        sample.encoder_right_velocity_mm_s = 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    detector.completeVerification(Reason::WheelBlocked, false);
    assert(detector.scores().wheel_blocked == 250U);
    detector.completeVerification(Reason::WheelBlocked, true);
    assert(detector.scores().wheel_blocked == 0U);
    DetectorSample healthy = baseSample(4000);
    const Assessment healthy_result = detector.update(healthy);
    assert(!healthy_result.suspend_requested);
    assert(detector.scores().wheel_blocked == 0U);
}

} // namespace

int main()
{
    testWheelBlocked();
    testOneWheelBlocked();
    testWrongDirectionTriggersWheelBlocked();
    testHealthyLowSpeedRotationDoesNotTrigger();
    testRotationBlockedWithGyro();
    testRotationBlockedWithoutGyroWhenWheelsLocked();
    testGpsVelocityMismatchTriggersSlip();
    testGpsDistanceMismatchTriggersSlip();
    testNoGpsEventuallyRequestsSlipVerification();
    testAccelerationImpactAcceleratesFallback();
    testEncoderGyroMismatchTriggersSlipWhileSteering();
    testHealthyGpsVelocityDoesNotTriggerSlip();
    testVerificationCompletionRearmsReason();
    return 0;
}
