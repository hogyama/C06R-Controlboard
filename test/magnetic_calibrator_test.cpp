#include <assert.h>
#include <math.h>
#include <stdio.h>

#include "domain/localization/magnetic_calibrator.h"

namespace {

using Domain::Localization::MagneticCalibration;
using Domain::Localization::MagneticCalibrator;

Sensor::MagneticData ellipsoidSample(
    uint64_t timestamp_us,
    float unit_x,
    float unit_y,
    float unit_z,
    Sensor::Source source = Sensor::Source::BoardI2c)
{
    constexpr float radius = 48.0f;
    const float field[3] = {
        radius * unit_x, radius * unit_y, radius * unit_z};
    Sensor::MagneticData sample{};
    sample.x_uT = 12.0f + 1.30f * field[0] + 0.12f * field[1];
    sample.y_uT = -8.0f + 0.05f * field[0] + 0.78f * field[1];
    sample.z_uT = 21.0f + 0.08f * field[0] + 1.12f * field[2];
    sample.metadata = {
        timestamp_us, timestamp_us, source, true};
    return sample;
}

Sensor::MagneticData sphereSample(uint16_t index, uint16_t count)
{
    constexpr float golden_angle = 2.39996323f;
    const float z = 1.0f - 2.0f * (index + 0.5f) / count;
    const float radial = sqrtf(fmaxf(0.0f, 1.0f - z * z));
    const float angle = golden_angle * index;
    return ellipsoidSample(
        index + 1U,
        radial * cosf(angle),
        radial * sinf(angle),
        z);
}

Sensor::MagneticData largeOffsetSphereSample(uint16_t index, uint16_t count)
{
    Sensor::MagneticData sample = sphereSample(index, count);
    // Approximate the BOARD values measured on the vehicle. The field shape
    // is unchanged; only the fixed hard-iron center is moved from
    // (12,-8,21) uT to about (-28,20,-50) uT.
    sample.x_uT -= 40.0f;
    sample.y_uT += 28.0f;
    sample.z_uT -= 71.0f;
    return sample;
}

void addDeterministicNoise(Sensor::MagneticData& sample, uint32_t index)
{
    sample.x_uT += 0.35f * sinf(0.731f * index + 0.2f);
    sample.y_uT += 0.35f * sinf(1.113f * index + 1.7f);
    sample.z_uT += 0.35f * sinf(1.619f * index + 2.8f);
}

bool addAndTryFit(
    MagneticCalibrator& calibrator,
    const Sensor::MagneticData& sample,
    MagneticCalibration& calibration,
    uint16_t& fit_attempts)
{
    assert(calibrator.add(sample));
    if (calibrator.active()) return false;
    ++fit_attempts;
    const bool success = calibrator.finish(calibration);
    if (!success) assert(calibrator.active());
    return success;
}

void testRepeatedPoseDoesNotComplete()
{
    MagneticCalibrator calibrator;
    calibrator.start();
    for (uint16_t i = 0; i < 2000U; ++i) {
        const auto sample = ellipsoidSample(
            i + 1U, 1.0f, 0.0f, 0.0f);
        assert(calibrator.add(sample));
    }
    assert(calibrator.active());
    assert(!calibrator.coverageReady());
    assert(calibrator.sampleCount() <=
        MagneticCalibrator::MAX_SAMPLES_PER_BIN);
    MagneticCalibration calibration{};
    assert(!calibrator.finish(calibration));
    assert(!calibration.valid);
    printf(
        "mag_cal_sim: repeated_pose result=REJECT samples=%u bins=%u "
        "progress=%u\n",
        calibrator.sampleCount(), calibrator.directionBins(),
        calibrator.progressPercent());
}

void testPlanarRotationDoesNotComplete()
{
    MagneticCalibrator calibrator;
    calibrator.start();
    for (uint16_t i = 0; i < 3000U; ++i) {
        const float angle = 2.0f * 3.14159265358979323846f *
            static_cast<float>(i % 720U) / 720.0f;
        assert(calibrator.add(ellipsoidSample(
            i + 1U, cosf(angle), sinf(angle), 0.0f)));
    }
    assert(calibrator.active());
    assert(!calibrator.coverageReady());
    assert(calibrator.axisRangeUT(2) <
        MagneticCalibrator::MINIMUM_AXIS_RANGE_UT);
    assert(calibrator.eigenvalueRatio() <
        MagneticCalibrator::MINIMUM_EIGENVALUE_RATIO);
    printf(
        "mag_cal_sim: planar_rotation result=REJECT samples=%u bins=%u "
        "eigen=%.3f\n",
        calibrator.sampleCount(), calibrator.directionBins(),
        calibrator.eigenvalueRatio());
}

void testTiltedPlaneWithFullAxisRangesDoesNotComplete()
{
    MagneticCalibrator calibrator;
    calibrator.start();
    constexpr float inverse_sqrt_two = 0.7071067812f;
    for (uint16_t i = 0; i < 3000U; ++i) {
        const float angle = 2.0f * 3.14159265358979323846f *
            static_cast<float>(i % 720U) / 720.0f;
        assert(calibrator.add(ellipsoidSample(
            i + 1U,
            cosf(angle),
            inverse_sqrt_two * sinf(angle),
            inverse_sqrt_two * sinf(angle))));
    }
    assert(calibrator.active());
    assert(!calibrator.coverageReady());
    assert(calibrator.axisRangeUT(0) >=
        MagneticCalibrator::MINIMUM_AXIS_RANGE_UT);
    assert(calibrator.axisRangeUT(1) >=
        MagneticCalibrator::MINIMUM_AXIS_RANGE_UT);
    assert(calibrator.axisRangeUT(2) >=
        MagneticCalibrator::MINIMUM_AXIS_RANGE_UT);
    assert(calibrator.eigenvalueRatio() <
        MagneticCalibrator::MINIMUM_EIGENVALUE_RATIO);
    printf(
        "mag_cal_sim: tilted_plane result=REJECT samples=%u bins=%u "
        "range=%.2f/%.2f/%.2f eigen=%.3f\n",
        calibrator.sampleCount(), calibrator.directionBins(),
        calibrator.axisRangeUT(0), calibrator.axisRangeUT(1),
        calibrator.axisRangeUT(2), calibrator.eigenvalueRatio());
}

void testUpperHemisphereDoesNotComplete()
{
    MagneticCalibrator calibrator;
    calibrator.start();
    MagneticCalibration calibration{};
    uint16_t fit_attempts = 0;
    bool success = false;
    constexpr uint16_t input_count = 5000U;
    constexpr float golden_angle = 2.39996323f;
    for (uint16_t i = 0; i < input_count; ++i) {
        const float z = (i + 0.5f) / input_count;
        const float radial = sqrtf(fmaxf(0.0f, 1.0f - z * z));
        const float angle = golden_angle * i;
        success = addAndTryFit(
            calibrator,
            ellipsoidSample(
                i + 1U,
                radial * cosf(angle),
                radial * sinf(angle),
                z),
            calibration,
            fit_attempts);
        assert(!success);
    }
    assert(calibrator.active());
    assert(!calibrator.coverageReady());
    assert(!calibration.valid);
    printf(
        "mag_cal_sim: upper_hemisphere result=REJECT fit_attempts=%u "
        "samples=%u bins=%u\n",
        fit_attempts, calibrator.sampleCount(), calibrator.directionBins());
}

void testCanSamplesAreRejected()
{
    MagneticCalibrator calibrator;
    calibrator.start();
    assert(!calibrator.add(ellipsoidSample(
        1U, 1.0f, 0.0f, 0.0f, Sensor::Source::Can)));
    assert(calibrator.sampleCount() == 0U);
}

void testUniformSphereCompletesAndFits()
{
    MagneticCalibrator calibrator;
    calibrator.start();
    MagneticCalibration calibration{};
    uint16_t fit_attempts = 0;
    bool success = false;
    constexpr uint16_t input_count = 6000U;
    uint16_t inputs_used = 0;
    for (; inputs_used < input_count && !success; ++inputs_used) {
        success = addAndTryFit(
            calibrator, sphereSample(inputs_used, input_count),
            calibration, fit_attempts);
    }
    assert(success);
    assert(!calibrator.active());
    assert(calibrator.coverageReady());
    assert(calibrator.sampleCount() >=
        MagneticCalibrator::MINIMUM_SAMPLE_COUNT);
    assert(calibrator.directionBins() >=
        MagneticCalibrator::MINIMUM_DIRECTION_BINS);
    assert(calibrator.progressPercent() == 100U);

    assert(calibration.valid);
    assert(fabsf(calibration.hard_iron_uT[0] - 12.0f) < 0.3f);
    assert(fabsf(calibration.hard_iron_uT[1] + 8.0f) < 0.3f);
    assert(fabsf(calibration.hard_iron_uT[2] - 21.0f) < 0.3f);
    assert(calibrator.rmsErrorUT() < 0.5f);
    printf(
        "mag_cal_sim: uniform_sphere result=SUCCESS inputs=%u fit_attempts=%u "
        "samples=%u bins=%u rms=%.3f\n",
        inputs_used, fit_attempts, calibrator.sampleCount(),
        calibrator.directionBins(), calibrator.rmsErrorUT());
}

void testBiasedHistoryThenNoisySphereCompletesAndFits()
{
    MagneticCalibrator calibrator;
    calibrator.start();
    MagneticCalibration calibration{};
    uint16_t fit_attempts = 0;

    // Simulate spending a long time rotating only in one plane first.
    for (uint16_t i = 0; i < 3000U; ++i) {
        const float angle = 2.0f * 3.14159265358979323846f *
            static_cast<float>(i % 720U) / 720.0f;
        Sensor::MagneticData sample = ellipsoidSample(
            i + 1U, cosf(angle), sinf(angle), 0.0f);
        addDeterministicNoise(sample, i);
        assert(calibrator.add(sample));
    }
    assert(calibrator.active());

    constexpr uint16_t sphere_input_count = 6000U;
    uint16_t sphere_inputs_used = 0;
    bool success = false;
    for (; sphere_inputs_used < sphere_input_count && !success;
         ++sphere_inputs_used) {
        Sensor::MagneticData sample = sphereSample(
            sphere_inputs_used, sphere_input_count);
        sample.metadata.timestamp_us += 3000U;
        sample.metadata.received_us += 3000U;
        addDeterministicNoise(sample, sphere_inputs_used + 3000U);
        success = addAndTryFit(
            calibrator, sample, calibration, fit_attempts);
    }
    assert(success);
    assert(!calibrator.active());
    assert(calibrator.coverageReady());

    assert(calibration.valid);
    assert(fabsf(calibration.hard_iron_uT[0] - 12.0f) < 0.6f);
    assert(fabsf(calibration.hard_iron_uT[1] + 8.0f) < 0.6f);
    assert(fabsf(calibration.hard_iron_uT[2] - 21.0f) < 0.6f);
    assert(calibrator.rmsErrorUT() < 1.0f);

    printf(
        "mag_cal_sim: sphere_inputs=%u fit_attempts=%u samples=%u bins=%u "
        "range=%.2f/%.2f/%.2f eigen=%.3f "
        "offset=%.3f/%.3f/%.3f rms=%.3f\n",
        sphere_inputs_used,
        fit_attempts,
        calibrator.sampleCount(),
        calibrator.directionBins(),
        calibrator.axisRangeUT(0),
        calibrator.axisRangeUT(1),
        calibrator.axisRangeUT(2),
        calibrator.eigenvalueRatio(),
        calibration.hard_iron_uT[0],
        calibration.hard_iron_uT[1],
        calibration.hard_iron_uT[2],
        calibrator.rmsErrorUT());
}

void testMeasuredLargeHardIronOffsetCompletesAndFits()
{
    MagneticCalibrator calibrator;
    calibrator.start();
    MagneticCalibration calibration{};
    uint16_t fit_attempts = 0;
    bool success = false;
    constexpr uint16_t input_count = 6000U;
    uint16_t inputs_used = 0;
    for (; inputs_used < input_count && !success; ++inputs_used) {
        Sensor::MagneticData sample = largeOffsetSphereSample(
            inputs_used, input_count);
        addDeterministicNoise(sample, inputs_used);
        success = addAndTryFit(
            calibrator, sample, calibration, fit_attempts);
    }
    assert(success);
    assert(calibration.valid);
    assert(fabsf(calibration.hard_iron_uT[0] + 28.0f) < 0.7f);
    assert(fabsf(calibration.hard_iron_uT[1] - 20.0f) < 0.7f);
    assert(fabsf(calibration.hard_iron_uT[2] + 50.0f) < 0.7f);
    assert(calibrator.rmsErrorUT() < 1.0f);
    printf(
        "mag_cal_sim: measured_offset result=SUCCESS inputs=%u "
        "fit_attempts=%u samples=%u bins=%u/%u "
        "offset=%.3f/%.3f/%.3f rms=%.3f\n",
        inputs_used, fit_attempts, calibrator.sampleCount(),
        calibrator.directionBins(), calibrator.directionTargetBins(),
        calibration.hard_iron_uT[0], calibration.hard_iron_uT[1],
        calibration.hard_iron_uT[2], calibrator.rmsErrorUT());
}

} // namespace

int main()
{
    testRepeatedPoseDoesNotComplete();
    testPlanarRotationDoesNotComplete();
    testTiltedPlaneWithFullAxisRangesDoesNotComplete();
    testUpperHemisphereDoesNotComplete();
    testCanSamplesAreRejected();
    testUniformSphereCompletesAndFits();
    testBiasedHistoryThenNoisySphereCompletesAndFits();
    testMeasuredLargeHardIronOffsetCompletesAndFits();
    return 0;
}
