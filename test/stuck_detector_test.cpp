#include <assert.h>
#include <math.h>
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
    sample.encoder_updated = true;
    sample.encoder_left_velocity_mm_s = 500.0f;
    sample.encoder_right_velocity_mm_s = 500.0f;
    sample.gyro_available = true;
    sample.gyro_yaw_rate_rad_s = 0.0f;
    sample.acceleration_available = true;
    sample.acceleration_x_g = 0.0f;
    sample.acceleration_y_g = 0.0f;
    sample.acceleration_z_g = 1.0f;
    sample.fusion_available = true;
    sample.fusion_position_usable = true;
    sample.fusion_yaw_usable = true;
    sample.fusion_forward_velocity_mm_s = 500.0f;
    sample.gps_available = false;
    sample.gps_horizontal_accuracy_mm = 1000;
    return sample;
}

void assertOnlyWheelBlocked(const Assessment& result)
{
    assert(result.scores.wheel_blocked >= 700U);
    assert(result.scores.wheel_slip == 0U);
    assert(result.scores.rotation_blocked == 0U);
    assert(result.scores.body_trapped == 0U);
}

void assertOnlyWheelSlip(const Assessment& result)
{
    assert(result.scores.wheel_blocked == 0U);
    assert(result.scores.wheel_slip >= 700U);
    assert(result.scores.rotation_blocked == 0U);
    assert(result.scores.body_trapped == 0U);
}

void assertOnlyRotationBlocked(const Assessment& result)
{
    assert(result.scores.wheel_blocked == 0U);
    assert(result.scores.wheel_slip == 0U);
    assert(result.scores.rotation_blocked >= 700U);
    assert(result.scores.body_trapped == 0U);
}

void assertOnlyBodyTrapped(const Assessment& result)
{
    assert(result.scores.wheel_blocked == 0U);
    assert(result.scores.wheel_slip == 0U);
    assert(result.scores.rotation_blocked == 0U);
    assert(result.scores.body_trapped >= 700U);
}

void testWheelBlockedOwnsOnlyItsScore()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 7000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.encoder_left_velocity_mm_s = 0.0f;
        sample.encoder_right_velocity_mm_s = 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assert(result.reason == Reason::WheelBlocked);
    assertOnlyWheelBlocked(result);
}

void testOneWheelBlockedUsesWheelBlockedScore()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 7000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.encoder_left_velocity_mm_s = 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assert(result.reason == Reason::WheelBlocked);
    assertOnlyWheelBlocked(result);
}

void testWheelSlipUsesRawGpsMismatchOnly()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 18000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 600.0f;
        sample.encoder_left_velocity_mm_s = 600.0f;
        sample.encoder_right_velocity_mm_s = 600.0f;
        sample.gps_available = true;
        sample.gps_updated = (now % 1000U) == 0U;
        sample.gps_x_mm = 10000;
        sample.gps_y_mm = 10000;
        // Deliberately moving fusion position must not hide raw-GPS slip.
        sample.fusion_x_mm = static_cast<int32_t>(now * 6U / 10U);
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assert(result.reason == Reason::WheelSlip);
    assertOnlyWheelSlip(result);
}

void testNoGpsDoesNotInventWheelSlip()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 30000; now += 100) {
        result = detector.update(baseSample(now));
    }
    assert(!result.suspend_requested);
    assert(result.scores.wheel_slip == 0U);
}

void testWrongDirectionEncoderIsNotHealthyOrSlipEvidence()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 18000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.encoder_left_velocity_mm_s = -500.0f;
        sample.encoder_right_velocity_mm_s = -500.0f;
        sample.gps_available = true;
        sample.gps_updated = (now % 1000U) == 0U;
        sample.gps_x_mm = 10000;
        sample.gps_y_mm = 10000;
        result = detector.update(sample);
    }
    assert(!result.suspend_requested);
    assert(result.scores.wheel_slip == 0U);
}

void testMovingGpsDecaysWheelSlip()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 20000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.gps_available = true;
        sample.gps_updated = (now % 1000U) == 0U;
        sample.gps_x_mm = 10000 + static_cast<int32_t>(now / 2U);
        sample.gps_y_mm = 10000;
        result = detector.update(sample);
    }
    assert(!result.suspend_requested);
    assert(result.scores.wheel_slip == 0U);
}

void testRotationBlockedOwnsOnlyItsScore()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 7000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 0.0f;
        sample.command_yaw_rate_rad_s = 1.0f;
        sample.encoder_left_velocity_mm_s = -120.0f;
        sample.encoder_right_velocity_mm_s = 120.0f;
        sample.gyro_yaw_rate_rad_s = 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assert(result.reason == Reason::RotationBlocked);
    assertOnlyRotationBlocked(result);
}

void testActualRotationPreventsRotationScore()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 10000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 0.0f;
        sample.command_yaw_rate_rad_s = 1.0f;
        sample.encoder_left_velocity_mm_s = -120.0f;
        sample.encoder_right_velocity_mm_s = 120.0f;
        sample.gyro_yaw_rate_rad_s = 1.0f;
        result = detector.update(sample);
    }
    assert(!result.suspend_requested);
    assert(result.scores.rotation_blocked == 0U);
}

void testStoppedWheelsDuringRotationUseRotationScore()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 7000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 0.0f;
        sample.command_yaw_rate_rad_s = 0.5f;
        sample.encoder_left_velocity_mm_s = 0.0f;
        sample.encoder_right_velocity_mm_s = 0.0f;
        sample.gyro_yaw_rate_rad_s = 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assert(result.reason == Reason::RotationBlocked);
    assertOnlyRotationBlocked(result);
}

void testBodyTrappedOwnsOnlyItsScore()
{
    StuckDetector detector{};
    Assessment result{};
    constexpr float tilt_rad = 30.0f * 3.14159265f / 180.0f;
    for (uint32_t now = 100; now <= 12000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.acceleration_x_g = sinf(tilt_rad);
        sample.acceleration_z_g = cosf(tilt_rad);
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    assert(result.reason == Reason::BodyTrapped);
    assertOnlyBodyTrapped(result);
}

void testDynamicAccelerationDoesNotCreateBodyScore()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 12000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.acceleration_x_g = 2.0f;
        sample.acceleration_y_g = 1.0f;
        sample.acceleration_z_g = 2.0f;
        result = detector.update(sample);
    }
    assert(!result.suspend_requested);
    assert(result.scores.body_trapped == 0U);
}

void testHealthyEvidenceActivelyDecaysOnlyRelevantScore()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 3000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.encoder_left_velocity_mm_s = 0.0f;
        sample.encoder_right_velocity_mm_s = 0.0f;
        result = detector.update(sample);
    }
    assert(result.scores.wheel_blocked > 0U);
    for (uint32_t now = 3100; now <= 6000; now += 100) {
        result = detector.update(baseSample(now));
    }
    assert(result.scores.wheel_blocked == 0U);
    assert(result.scores.wheel_slip == 0U);
    assert(result.scores.rotation_blocked == 0U);
    assert(result.scores.body_trapped == 0U);
}

void testVerificationCompletionRearmsOneReasonOnly()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 7000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.encoder_left_velocity_mm_s = 0.0f;
        sample.encoder_right_velocity_mm_s = 0.0f;
        result = detector.update(sample);
        if (result.suspend_requested) break;
    }
    assert(result.suspend_requested);
    detector.completeVerification(Reason::WheelBlocked, false);
    assert(detector.scores().wheel_blocked == 200U);
    detector.completeVerification(Reason::WheelBlocked, true);
    assert(detector.scores().wheel_blocked == 0U);
}

} // namespace

int main()
{
    testWheelBlockedOwnsOnlyItsScore();
    testOneWheelBlockedUsesWheelBlockedScore();
    testWheelSlipUsesRawGpsMismatchOnly();
    testNoGpsDoesNotInventWheelSlip();
    testWrongDirectionEncoderIsNotHealthyOrSlipEvidence();
    testMovingGpsDecaysWheelSlip();
    testRotationBlockedOwnsOnlyItsScore();
    testActualRotationPreventsRotationScore();
    testStoppedWheelsDuringRotationUseRotationScore();
    testBodyTrappedOwnsOnlyItsScore();
    testDynamicAccelerationDoesNotCreateBodyScore();
    testHealthyEvidenceActivelyDecaysOnlyRelevantScore();
    testVerificationCompletionRearmsOneReasonOnly();
    return 0;
}
