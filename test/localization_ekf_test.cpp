#include "domain/localization/localization.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

using namespace Domain::Localization;

namespace {

constexpr float PI = 3.14159265358979323846f;

void expect(bool condition, const char* message)
{
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", message);
        std::exit(1);
    }
}

GyroPreintegration gyro(
    uint64_t start_us,
    uint64_t end_us,
    float rate_rad_s)
{
    GyroPreintegration result{};
    result.start_timestamp_us = start_us;
    result.end_timestamp_us = end_us;
    result.integrated_z_rad =
        rate_rad_s * static_cast<float>(end_us - start_us) * 1.0e-6f;
    result.latest_z_rad_s = rate_rad_s;
    result.source = Sensor::Source::BoardI2c;
    result.sample_count = 20;
    result.valid = true;
    return result;
}

GpsObservation gps(
    uint64_t timestamp_us,
    float east_m,
    float north_m,
    float accuracy_m = 0.5f)
{
    GpsObservation result{};
    result.east_m = east_m;
    result.north_m = north_m;
    result.horizontal_accuracy_m = accuracy_m;
    result.fix_type = 3;
    result.satellites = 10;
    result.metadata = {
        timestamp_us,
        timestamp_us + 120000U,
        Sensor::Source::Gps,
        true};
    result.position_valid = true;
    return result;
}

EncoderVelocityObservation encoder(uint64_t timestamp_us, float velocity_m_s)
{
    EncoderVelocityObservation result{};
    result.velocity_m_s = velocity_m_s;
    result.left_velocity_m_s = velocity_m_s;
    result.right_velocity_m_s = velocity_m_s;
    result.variance_m2_s2 = 0.01f;
    result.metadata = {
        timestamp_us, timestamp_us, Sensor::Source::Can, true};
    result.valid = true;
    return result;
}

void testStraightAndReverse()
{
    Config config{};
    Ekf5 filter(config);
    filter.initialize(30.0f, 30.0f, 0.0f, 1.0f, 0.0f, 1000000U, true);
    uint64_t time_us = 1000000U;
    for (int i = 0; i < 100; ++i) {
        const uint64_t next = time_us + 10000U;
        expect(filter.predict(gyro(time_us, next, 0.0f)), "straight predict");
        time_us = next;
    }
    expect(std::fabs(filter.state()[POSITION_X] - 31.0f) < 0.02f,
           "straight position");
    expect(std::fabs(filter.state()[POSITION_Y] - 30.0f) < 0.02f,
           "straight lateral position");

    expect(filter.updateEncoderVelocity(encoder(time_us, -1.0f)),
           "reverse velocity update");
    for (int i = 0; i < 100; ++i) {
        const uint64_t next = time_us + 10000U;
        filter.predict(gyro(time_us, next, 0.0f));
        time_us = next;
    }
    expect(filter.state()[POSITION_X] < 30.4f, "signed reverse motion");
}

void testConstantTurnAndPiBoundary()
{
    Ekf5 filter;
    filter.initialize(0.0f, 0.0f, 3.10f, 0.0f, 0.0f, 1000000U, true);
    expect(filter.predict(gyro(1000000U, 1200000U, 1.0f)),
           "turn predict");
    expect(filter.state()[YAW] >= -PI && filter.state()[YAW] < PI,
           "yaw range");
    expect(filter.state()[YAW] < -2.9f, "pi wrap direction");
}

void testBiasZaru()
{
    Ekf5 filter;
    filter.initialize(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000000U, true);
    for (int i = 0; i < 100; ++i) {
        expect(filter.updateZaru(0.025f, 1.0e-5f), "zaru update");
    }
    expect(std::fabs(filter.state()[GYRO_BIAS_Z] - 0.025f) < 0.002f,
           "gyro bias convergence");
}

void testOutliersAndMagnetic()
{
    Ekf5 filter;
    filter.initialize(30.0f, 30.0f, 0.0f, 0.0f, 0.0f, 1000000U, true);
    const GpsObservation outlier = gps(1000000U, 300.0f, -200.0f, 0.5f);
    expect(!filter.updateGpsPosition(outlier), "gps outlier rejected");
    expect(std::fabs(filter.state()[POSITION_X] - 30.0f) < 0.01f,
           "gps rejection preserves state");

    MagneticHeadingObservation magnetic{};
    magnetic.heading_rad = 0.2f;
    magnetic.variance_rad2 = 0.04f;
    magnetic.metadata = {
        1000000U, 1000000U, Sensor::Source::BoardI2c, true};
    magnetic.valid = true;
    expect(filter.updateMagneticHeading(magnetic), "magnetic heading update");
    expect(filter.state()[YAW] > 0.0f, "magnetic correction direction");
}

void testDelayedGpsReplayAndCovariance()
{
    Localization localization;
    GpsObservation initial = gps(1000000U, 30.0f, 30.0f);
    MagneticHeadingObservation heading{};
    heading.heading_rad = 0.0f;
    heading.variance_rad2 = 0.04f;
    heading.metadata = {
        1000000U, 1000000U, Sensor::Source::BoardI2c, true};
    heading.valid = true;
    expect(localization.initializeFromGps(initial, &heading, 0.0f),
           "localization initialize");

    uint64_t time_us = 1000000U;
    for (int i = 0; i < 100; ++i) {
        CycleInput input{};
        input.gyroscope = gyro(time_us, time_us + 10000U, 0.0f);
        input.encoder = encoder(time_us + 10000U, 1.0f);
        input.has_encoder = true;
        localization.processCycle(input);
        time_us += 10000U;
    }
    GpsObservation delayed = gps(1500000U, 30.5f, 30.0f, 0.4f);
    expect(localization.processGps(delayed), "delayed gps replay");
    const LocalizationEstimate estimate = localization.estimate(time_us);
    expect(estimate.initialized, "delayed estimate initialized");
    for (uint8_t i = 0; i < STATE_COUNT; ++i) {
        expect(std::isfinite(estimate.covariance[i][i]) &&
               estimate.covariance[i][i] > 0.0f,
               "positive finite covariance");
        for (uint8_t j = 0; j < STATE_COUNT; ++j) {
            expect(std::fabs(
                estimate.covariance[i][j] - estimate.covariance[j][i]) <
                1.0e-4f,
                "symmetric covariance");
        }
    }
}

void testEncoderPreprocessorAndMissingFrame()
{
    Config config{};
    Preprocessor preprocessor(config);
    EncoderObservation first{};
    first.left_cumulative_mm = 1000;
    first.right_cumulative_mm = 1000;
    first.sequence = 1;
    first.metadata = {
        1000000U, 1000000U, Sensor::Source::Can, true};
    first.left_valid = true;
    first.right_valid = true;
    expect(!preprocessor.processEncoder(first).valid, "first encoder is baseline");

    EncoderObservation second = first;
    second.left_cumulative_mm += 100;
    second.right_cumulative_mm += 100;
    second.sequence = 2;
    second.metadata.timestamp_us += 100000U;
    second.metadata.received_us += 100000U;
    const EncoderVelocityObservation velocity =
        preprocessor.processEncoder(second);
    expect(velocity.valid, "encoder velocity valid");
    expect(std::fabs(velocity.velocity_m_s - 1.0f) < 0.001f,
           "encoder cumulative mm conversion");

    EncoderObservation discontinuity = second;
    discontinuity.left_cumulative_mm += 10000;
    discontinuity.right_cumulative_mm += 10000;
    discontinuity.sequence = 3;
    discontinuity.metadata.timestamp_us += 100000U;
    discontinuity.metadata.received_us += 100000U;
    expect(!preprocessor.processEncoder(discontinuity).valid,
           "encoder impossible speed rejected");

    Localization localization;
    GpsObservation initial = gps(2000000U, 30.0f, 30.0f);
    MagneticHeadingObservation heading{};
    heading.heading_rad = 0.0f;
    heading.variance_rad2 = 0.04f;
    heading.metadata = {
        2000000U, 2000000U, Sensor::Source::BoardI2c, true};
    heading.valid = true;
    localization.initializeFromGps(initial, &heading, 0.0f);
    CycleInput missing_encoder{};
    missing_encoder.gyroscope = gyro(2000000U, 2010000U, 0.0f);
    expect(localization.processCycle(missing_encoder),
           "prediction continues without encoder frame");
    expect(std::isfinite(localization.ekf().state()[POSITION_X]),
           "missing encoder state finite");
}

void testMagneticPreprocessorAndStationaryGate()
{
    Config config{};
    config.stationary_hold_us = 0;
    Preprocessor preprocessor(config);
    Sensor::AccelerometerData acceleration{};
    acceleration.z_m_s2 = 9.80665f;
    acceleration.metadata = {
        1000000U, 1000000U, Sensor::Source::BoardI2c, true};
    for (uint8_t i = 0; i < 8; ++i) {
        acceleration.metadata.timestamp_us += 16667U;
        preprocessor.updateAcceleration(acceleration);
    }
    expect(preprocessor.stationaryObservation(0.0f, false, 1200000U).stationary,
           "multi-condition stationary accepted");
    expect(!preprocessor.stationaryObservation(0.0f, true, 1210000U).stationary,
           "motor command prevents stationary update");

    Sensor::MagneticData magnetic{};
    // Body magnetic north at east-facing yaw points to +Y.
    magnetic.y_uT = 40.0f;
    magnetic.metadata = {
        1220000U, 1220000U, Sensor::Source::BoardI2c, true};
    const MagneticHeadingObservation heading =
        preprocessor.processMagnetic(magnetic);
    expect(heading.valid && std::fabs(heading.heading_rad) < 0.01f,
           "magnetic east-zero heading");

    MagneticCalibration calibration{};
    calibration.hard_iron_uT[0] = 10.0f;
    calibration.hard_iron_uT[1] = -5.0f;
    calibration.hard_iron_uT[2] = 20.0f;
    calibration.valid = true;
    preprocessor.setMagneticCalibration(calibration);
    magnetic.x_uT = 10.0f;
    magnetic.y_uT = 35.0f;
    magnetic.z_uT = 20.0f;
    const MagneticDiagnostic diagnostic =
        preprocessor.magneticDiagnostic(magnetic);
    expect(diagnostic.vector_valid && diagnostic.calibration_applied,
           "board magnetic diagnostic applies calibration");
    expect(std::fabs(diagnostic.corrected_uT[0]) < 0.01f &&
               std::fabs(diagnostic.corrected_uT[1] - 40.0f) < 0.01f &&
               std::fabs(diagnostic.corrected_uT[2]) < 0.01f,
           "board corrected magnetic vector");
    expect(diagnostic.heading_valid &&
               std::fabs(diagnostic.heading_rad) < 0.01f,
           "diagnostic heading is available before localization");
    magnetic.x_uT = 40.0f;
    magnetic.y_uT = 0.0f;
    magnetic.z_uT = 0.0f;
    magnetic.metadata.source = Sensor::Source::Can;
    const MagneticDiagnostic can_diagnostic =
        preprocessor.magneticDiagnostic(magnetic);
    expect(can_diagnostic.heading_valid &&
               !can_diagnostic.calibration_applied &&
               std::fabs(can_diagnostic.heading_rad -
                   1.57079632679f) < 0.01f,
           "CAN magnetic diagnostic bypasses BOARD calibration");
    magnetic.x_uT = 1000.0f;
    magnetic.y_uT = 1000.0f;
    expect(!preprocessor.processMagnetic(magnetic).valid,
           "magnetic field disturbance rejected");
}

void testLongRunCovarianceAndTimeout()
{
    Localization localization;
    GpsObservation initial = gps(1000000U, 30.0f, 30.0f);
    MagneticHeadingObservation heading{};
    heading.heading_rad = 0.0f;
    heading.variance_rad2 = 0.04f;
    heading.metadata = {
        1000000U, 1000000U, Sensor::Source::BoardI2c, true};
    heading.valid = true;
    localization.initializeFromGps(initial, &heading, 0.0f);
    uint64_t time_us = 1000000U;
    for (int i = 0; i < 10000; ++i) {
        CycleInput input{};
        input.gyroscope = gyro(time_us, time_us + 10000U, 0.01f);
        if ((i % 10) == 0) {
            input.encoder = encoder(time_us + 10000U, 0.5f);
            input.has_encoder = true;
        }
        localization.processCycle(input);
        time_us += 10000U;
    }
    const LocalizationEstimate running = localization.estimate(time_us);
    for (uint8_t i = 0; i < STATE_COUNT; ++i) {
        expect(std::isfinite(running.covariance[i][i]) &&
               running.covariance[i][i] > 0.0f,
               "long-run covariance stable");
    }
    const LocalizationEstimate stale = localization.estimate(time_us + 20000000U);
    expect(stale.gps_health == SensorHealth::Failed,
           "gps timeout becomes failed");
    expect((stale.fault_flags & FAULT_GNSS_TIMEOUT) != 0U,
           "gps timeout fault flag");
}

void testEncoderYawFallback()
{
    Localization localization;
    GpsObservation initial = gps(1000000U, 30.0f, 30.0f);
    MagneticHeadingObservation heading{};
    heading.heading_rad = 0.0f;
    heading.variance_rad2 = 0.04f;
    heading.metadata = {
        1000000U, 1000000U, Sensor::Source::BoardI2c, true};
    heading.valid = true;
    expect(localization.initializeFromGps(initial, &heading, 0.0f),
           "encoder fallback initialize");

    EncoderVelocityObservation wheels{};
    wheels.velocity_m_s = 0.09f;
    wheels.left_velocity_m_s = 0.0f;
    wheels.right_velocity_m_s = 0.18f;
    wheels.delta_distance_m = 0.009f;
    wheels.delta_yaw_rad = 0.1f;
    wheels.yaw_rate_rad_s = 1.0f;
    wheels.yaw_variance_rad2 = 0.001f;
    wheels.interval_us = 100000U;
    wheels.variance_m2_s2 = 0.01f;
    wheels.metadata = {
        1100000U, 1100000U, Sensor::Source::Can, true};
    wheels.valid = true;
    CycleInput input{};
    input.timestamp_us = 1100000U;
    input.encoder = wheels;
    input.has_encoder = true;
    expect(localization.processCycle(input),
           "encoder predicts without gyro");
    expect(localization.ekf().state()[YAW] > 0.05f,
           "encoder differential updates yaw");
}

void testConsistentGpsWarp()
{
    Config config{};
    config.mahalanobis_gate_2d = 0.1f;
    Localization localization(config);
    GpsObservation initial = gps(1000000U, 30.0f, 30.0f);
    MagneticHeadingObservation heading{};
    heading.heading_rad = 0.0f;
    heading.variance_rad2 = 0.04f;
    heading.metadata = {
        1000000U, 1000000U, Sensor::Source::BoardI2c, true};
    heading.valid = true;
    expect(localization.initializeFromGps(initial, &heading, 0.0f),
           "warp initialize");

    uint64_t timestamp_us = 1000000U;
    for (uint8_t i = 0; i < 9U; ++i) {
        CycleInput input{};
        input.timestamp_us = timestamp_us + 100000U;
        input.gyroscope = gyro(timestamp_us, input.timestamp_us, 0.0f);
        localization.processCycle(input);
        timestamp_us = input.timestamp_us;
        localization.processGps(gps(timestamp_us, 40.0f, 30.0f));
    }
    const LocalizationEstimate estimate = localization.estimate(timestamp_us);
    expect(estimate.gps_warp_count == 1U, "gps warp majority trigger");
    expect(std::fabs(estimate.px_m - 40.0f) < 1.0f,
           "gps warp shifts position only");
    expect(std::fabs(estimate.theta_rad) < 0.1f,
           "gps warp preserves yaw");
}

} // namespace

int main()
{
    testStraightAndReverse();
    testConstantTurnAndPiBoundary();
    testBiasZaru();
    testOutliersAndMagnetic();
    testDelayedGpsReplayAndCovariance();
    testEncoderPreprocessorAndMissingFrame();
    testMagneticPreprocessorAndStationaryGate();
    testLongRunCovarianceAndTimeout();
    testEncoderYawFallback();
    testConsistentGpsWarp();
    std::puts("localization_ekf_test: PASS");
    return 0;
}
