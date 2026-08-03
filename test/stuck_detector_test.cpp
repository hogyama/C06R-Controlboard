#include <assert.h>
#include <stdint.h>

#include "domain/motion/stuck_detector.h"

namespace {

using Domain::Motion::Assessment;
using Domain::Motion::Condition;
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
    sample.fusion_available = true;
    sample.fusion_position_usable = true;
    sample.fusion_yaw_usable = true;
    sample.fusion_forward_velocity_mm_s = 500.0f;
    sample.gps_available = false;
    sample.gps_horizontal_accuracy_mm = 1000;
    return sample;
}

Assessment runUntil(
    StuckDetector& detector,
    uint32_t end_ms,
    DetectorSample (*make_sample)(uint32_t))
{
    Assessment assessment{};
    for (uint32_t now = 100; now <= end_ms; now += 100) {
        assessment = detector.update(make_sample(now));
        if (assessment.condition == Condition::Stuck ||
            assessment.condition == Condition::SensorFault) {
            break;
        }
    }
    return assessment;
}

DetectorSample translationBlocked(uint32_t now)
{
    DetectorSample sample = baseSample(now);
    sample.encoder_left_velocity_mm_s = 0.0f;
    sample.encoder_right_velocity_mm_s = 0.0f;
    sample.fusion_forward_velocity_mm_s = 0.0f;
    return sample;
}

DetectorSample encoderOnlyMoving(uint32_t now)
{
    DetectorSample sample = baseSample(now);
    sample.fusion_available = false;
    sample.fusion_position_usable = false;
    sample.fusion_yaw_usable = false;
    sample.gyro_available = false;
    return sample;
}

DetectorSample leftWheelBlocked(uint32_t now)
{
    DetectorSample sample = baseSample(now);
    sample.encoder_left_velocity_mm_s = 0.0f;
    sample.encoder_right_velocity_mm_s = 500.0f;
    sample.fusion_forward_velocity_mm_s = 250.0f;
    return sample;
}

DetectorSample rotationBlocked(uint32_t now)
{
    DetectorSample sample = baseSample(now);
    sample.command_velocity_mm_s = 0.0f;
    sample.command_yaw_rate_rad_s = 1.0f;
    sample.encoder_left_velocity_mm_s = 0.0f;
    sample.encoder_right_velocity_mm_s = 0.0f;
    sample.gyro_yaw_rate_rad_s = 0.0f;
    sample.fusion_forward_velocity_mm_s = 0.0f;
    sample.fusion_yaw_rate_rad_s = 0.0f;
    return sample;
}

DetectorSample rotationActuallyMoving(uint32_t now)
{
    DetectorSample sample = rotationBlocked(now);
    sample.gyro_yaw_rate_rad_s = 1.0f;
    sample.fusion_yaw_rate_rad_s = 1.0f;
    return sample;
}

DetectorSample fusionOnlyBlocked(uint32_t now)
{
    DetectorSample sample = translationBlocked(now);
    sample.encoder_available = false;
    sample.encoder_updated = false;
    return sample;
}

DetectorSample unobservable(uint32_t now)
{
    DetectorSample sample = baseSample(now);
    sample.encoder_available = false;
    sample.gyro_available = false;
    sample.fusion_available = false;
    sample.fusion_position_usable = false;
    sample.fusion_yaw_usable = false;
    sample.gps_available = false;
    return sample;
}

void testTranslationBlock()
{
    StuckDetector detector{};
    const Assessment result =
        runUntil(detector, 8000, translationBlocked);
    assert(result.condition == Condition::Stuck);
    assert(result.reason == Reason::TranslationBlocked);
}

void testSingleSensorOperationDoesNotFalseTrigger()
{
    StuckDetector detector{};
    const Assessment result =
        runUntil(detector, 10000, encoderOnlyMoving);
    assert(result.condition != Condition::Stuck);
    assert(result.condition != Condition::SensorFault);
}

void testIndividualWheelAndRotationBlocks()
{
    StuckDetector left_detector{};
    const Assessment left =
        runUntil(left_detector, 5000, leftWheelBlocked);
    assert(left.condition == Condition::Stuck);
    assert(left.reason == Reason::LeftWheelBlocked);

    StuckDetector rotation_detector{};
    const Assessment rotation =
        runUntil(rotation_detector, 5000, rotationBlocked);
    assert(rotation.condition == Condition::Stuck);
    assert(rotation.reason == Reason::RotationBlocked);
}

void testContradictoryMotionPreventsFalseTrigger()
{
    StuckDetector rotation_detector{};
    const Assessment rotation =
        runUntil(rotation_detector, 10000, rotationActuallyMoving);
    assert(rotation.condition != Condition::Stuck);
    assert(rotation.condition != Condition::SensorFault);
}

void testFusionOnlyStillDetectsBlockedMotion()
{
    StuckDetector detector{};
    const Assessment result =
        runUntil(detector, 8000, fusionOnlyBlocked);
    assert(result.condition == Condition::Stuck);
    assert(result.reason == Reason::TranslationBlocked);
}

void testPulsedCameraRotation()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 14000; now += 100) {
        DetectorSample sample = rotationBlocked(now);
        const uint32_t phase = now % 700U;
        if (phase > 200U) {
            sample.command_valid = false;
            sample.command_yaw_rate_rad_s = 0.0f;
        }
        result = detector.update(sample);
        if (result.condition == Condition::Stuck) break;
    }
    assert(result.condition == Condition::Stuck);
    assert(result.reason == Reason::RotationBlocked);
}

void testWheelSlip()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 15000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 600.0f;
        sample.encoder_left_velocity_mm_s = 600.0f;
        sample.encoder_right_velocity_mm_s = 600.0f;
        sample.fusion_forward_velocity_mm_s = 600.0f;
        sample.gps_available = true;
        sample.gps_updated = (now % 1000U) == 0U;
        sample.gps_x_mm = 10000;
        sample.gps_y_mm = 10000;
        result = detector.update(sample);
        if (result.condition == Condition::Stuck) break;
    }
    assert(result.condition == Condition::Stuck);
    assert(result.reason == Reason::WheelSlip);
}

void testMovingGpsPreventsSlipFalseTrigger()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 15000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 600.0f;
        sample.encoder_left_velocity_mm_s = 600.0f;
        sample.encoder_right_velocity_mm_s = 600.0f;
        sample.fusion_forward_velocity_mm_s = 600.0f;
        sample.gps_available = true;
        sample.gps_updated = (now % 1000U) == 0U;
        sample.gps_x_mm =
            10000 + static_cast<int32_t>(now * 6U / 10U);
        sample.gps_y_mm = 10000;
        result = detector.update(sample);
    }
    assert(result.condition != Condition::Stuck);
    assert(result.condition != Condition::SensorFault);
}

void testLowAccuracyGpsIsNotUsedForSlip()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 15000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.command_velocity_mm_s = 600.0f;
        sample.encoder_left_velocity_mm_s = 600.0f;
        sample.encoder_right_velocity_mm_s = 600.0f;
        sample.fusion_forward_velocity_mm_s = 600.0f;
        sample.gps_available = true;
        sample.gps_updated = (now % 1000U) == 0U;
        sample.gps_horizontal_accuracy_mm = 2500;
        sample.gps_x_mm = 10000;
        sample.gps_y_mm = 10000;
        result = detector.update(sample);
    }
    assert(result.condition != Condition::Stuck);
    assert(result.condition != Condition::SensorFault);
}

void testPathNoProgress()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 18000; now += 100) {
        DetectorSample sample = baseSample(now);
        sample.path_available = true;
        sample.path_revision = 1;
        sample.path_nearest_index = 2;
        sample.path_distance_to_goal_mm = 20000.0f;
        sample.fusion_x_mm = 10000;
        sample.fusion_y_mm = 10000;
        result = detector.update(sample);
        if (result.condition == Condition::Stuck) break;
    }
    assert(result.condition == Condition::Stuck);
    assert(result.reason == Reason::PathNoProgress);
}

void testOscillation()
{
    StuckDetector detector{};
    Assessment result{};
    for (uint32_t now = 100; now <= 15000; now += 100) {
        DetectorSample sample = baseSample(now);
        const bool reverse = ((now / 2000U) & 1U) != 0U;
        const float velocity = reverse ? -500.0f : 500.0f;
        sample.command_velocity_mm_s = velocity;
        sample.encoder_left_velocity_mm_s = velocity;
        sample.encoder_right_velocity_mm_s = velocity;
        sample.fusion_forward_velocity_mm_s = velocity;
        sample.fusion_x_mm = reverse ? 10500 : 10000;
        sample.fusion_y_mm = 10000;
        result = detector.update(sample);
        if (result.condition == Condition::Stuck) break;
    }
    assert(result.condition == Condition::Stuck);
    assert(result.reason == Reason::Oscillation);
}

void testCompleteObservabilityLoss()
{
    StuckDetector detector{};
    const Assessment result =
        runUntil(detector, 7000, unobservable);
    assert(result.condition == Condition::SensorFault);
    assert(result.reason == Reason::MotionUnobservable);
}

} // namespace

int main()
{
    testTranslationBlock();
    testSingleSensorOperationDoesNotFalseTrigger();
    testIndividualWheelAndRotationBlocks();
    testContradictoryMotionPreventsFalseTrigger();
    testFusionOnlyStillDetectsBlockedMotion();
    testPulsedCameraRotation();
    testWheelSlip();
    testMovingGpsPreventsSlipFalseTrigger();
    testLowAccuracyGpsIsNotUsedForSlip();
    testPathNoProgress();
    testOscillation();
    testCompleteObservabilityLoss();
    return 0;
}
