#include "domain/fusion/fusion_filter.h"
#include "domain/geodesy/gps_to_xy.h"
#include "platform/field_config.h"

#include <cmath>
#include <cstdint>
#include <iostream>

namespace {

constexpr float PI_F = 3.14159265358979323846f;
int failures = 0;

void expect(bool condition, const char* message)
{
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
}

float angleError(float first, float second)
{
    float error = std::fmod(first - second + PI_F, 2.0f * PI_F);
    if (error < 0.0f) error += 2.0f * PI_F;
    return error - PI_F;
}

Domain::Fusion::ImuObservation stationaryImu(
    uint32_t timestamp_ms,
    float gyro_z_rad_s = 0.0f)
{
    Domain::Fusion::ImuObservation observation{};
    observation.timestamp_ms = timestamp_ms;
    observation.accel_z_g = 1.0f;
    observation.gyro_z_rad_s = gyro_z_rad_s;
    return observation;
}

Domain::Fusion::GpsUpdate gpsObservation(
    uint32_t timestamp_ms,
    int32_t x_mm,
    int32_t y_mm,
    uint32_t horizontal_accuracy_mm = 500)
{
    Domain::Fusion::GpsUpdate observation{};
    observation.timestamp_ms = timestamp_ms;
    observation.x_mm = x_mm;
    observation.y_mm = y_mm;
    observation.horizontal_accuracy_mm = horizontal_accuracy_mm;
    observation.speed_accuracy_mm_s = 500;
    observation.fix_type = 3;
    observation.fix_ok = true;
    return observation;
}

void testSouthWestFieldOrigin()
{
    Domain::Geodesy::GpsToXY converter(
        FieldConfig::ORIGIN_LATITUDE_E7,
        FieldConfig::ORIGIN_LONGITUDE_E7);
    int32_t origin_x_mm = 1;
    int32_t origin_y_mm = 1;
    expect(converter.valid(), "south-west WGS84 origin is valid");
    expect(converter.convert(
               FieldConfig::ORIGIN_LATITUDE_E7,
               FieldConfig::ORIGIN_LONGITUDE_E7,
               origin_x_mm,
               origin_y_mm),
           "Domain geodesy converts the field origin");
    expect(origin_x_mm == 0 && origin_y_mm == 0,
           "south-west coordinate converts to local zero");

    int32_t goal_x_mm = 0;
    int32_t goal_y_mm = 0;
    expect(converter.convert(
               356051630,
               1396831115,
               goal_x_mm,
               goal_y_mm),
           "Domain geodesy converts the surveyed goal");
    expect(goal_x_mm > 0 && goal_y_mm > 0,
           "surveyed point is north-east of the south-west origin");
    expect(FieldConfig::SIZE_X_MM == 60000 &&
           FieldConfig::SIZE_Y_MM == 60000,
           "field dimensions are 60 m by 60 m");
    expect(!converter.convert(
               1000000000,
               FieldConfig::ORIGIN_LONGITUDE_E7,
               goal_x_mm,
               goal_y_mm),
           "invalid latitude is rejected in Domain");
}

void testStationaryPredictionAndYawConvention()
{
    Domain::Fusion::Filter filter;
    filter.initialize(30000, 30000, 0.0f, true, 1000, 500);
    filter.beginCycle();
    expect(filter.predict(stationaryImu(1010)), "stationary IMU is accepted");

    const Domain::Fusion::Output output = filter.output(1010);
    expect(std::abs(output.x_mm - 30000) <= 1, "stationary X remains fixed");
    expect(std::abs(output.y_mm - 30000) <= 1, "stationary Y remains fixed");
    expect(std::abs(angleError(output.yaw_rad, 0.0f)) < 0.01f,
           "east is yaw zero");
    expect((output.status_flags & Domain::Fusion::STATUS_IMU_USED) != 0,
           "IMU-used status is set");
    expect((output.status_flags & Domain::Fusion::STATUS_POSITION_USABLE) != 0,
           "fresh initialized position is usable");
    expect((output.status_flags & Domain::Fusion::STATUS_YAW_USABLE) != 0,
           "fresh initialized yaw is usable");
}

void testQuaternionYawIntegration()
{
    Domain::Fusion::Filter filter;
    filter.initialize(30000, 30000, 0.0f, true, 1000, 500);
    filter.beginCycle();
    expect(filter.predict(stationaryImu(2000, 1.0f)),
           "one-second yaw integration is accepted");

    const Domain::Fusion::Output output = filter.output(2000);
    expect(std::abs(angleError(output.yaw_rad, 1.0f)) < 0.03f,
           "positive Z gyro rotates counter-clockwise from east");
}

void testGpsOutlierIsRejectedWithoutSnap()
{
    Domain::Fusion::Filter filter;
    filter.initialize(30000, 30000, 0.0f, true, 1000, 500);
    filter.beginCycle();
    filter.predict(stationaryImu(1100));

    const bool accepted =
        filter.updateGps(gpsObservation(1100, 59000, 59000, 500));
    const Domain::Fusion::Output output = filter.output(1100);
    expect(!accepted, "large in-field GPS spike is rejected by NIS");
    expect((output.status_flags & Domain::Fusion::STATUS_GPS_REJECTED) != 0,
           "GPS-rejected status is set");
    expect(std::abs(output.x_mm - 30000) < 1000,
           "GPS spike does not snap X");
    expect(std::abs(output.y_mm - 30000) < 1000,
           "GPS spike does not snap Y");
}

void testDelayedGpsIsProjectedToFilterTime()
{
    Domain::Fusion::Filter filter;
    filter.initialize(30000, 30000, 0.0f, true, 1000, 500);
    filter.beginCycle();
    filter.predict(stationaryImu(1200));

    Domain::Fusion::GpsUpdate delayed =
        gpsObservation(1100, 29900, 30000, 500);
    delayed.velocity_east_mm_s = 1000;
    delayed.speed_accuracy_mm_s = 500;
    expect(filter.updateGps(delayed),
           "delayed GPS is projected with receiver velocity and accepted");
    const Domain::Fusion::Output output = filter.output(1200);
    expect((output.status_flags & Domain::Fusion::STATUS_GPS_USED) != 0,
           "projected GPS sets the used flag");
    expect(std::abs(output.x_mm - 30000) < 500,
           "delayed GPS is not applied at its stale position");
}

void testGpsCourseFeedsBackToYaw()
{
    Domain::Fusion::Filter filter;
    filter.initialize(30000, 30000, 0.0f, false, 1000, 500);
    filter.beginCycle();
    filter.predict(stationaryImu(1100));

    Domain::Fusion::GpsUpdate northbound =
        gpsObservation(1100, 30000, 30000, 500);
    northbound.velocity_north_mm_s = 1500;
    northbound.speed_accuracy_mm_s = 150;
    expect(filter.updateGps(northbound),
           "accurate moving GPS course is accepted");

    const Domain::Fusion::Output output = filter.output(1100);
    expect(std::abs(angleError(output.yaw_rad, 0.5f * PI_F)) < 0.15f,
           "northbound GPS course feeds back toward yaw pi/2");
    expect((output.status_flags & Domain::Fusion::STATUS_YAW_USABLE) != 0,
           "accepted GPS course makes yaw usable");
}

void testMagneticOutlierAndHealthHysteresis()
{
    Domain::Fusion::Config config{};
    config.magnetic_declination_rad = -0.1382300768f;
    config.health_failure_rejections = 3;
    Domain::Fusion::Filter filter(config);
    filter.initialize(30000, 30000, 0.0f, true, 1000, 500);

    const float field_azimuth =
        0.5f * PI_F - config.magnetic_declination_rad;
    Domain::Fusion::MagneticObservation nominal{};
    nominal.timestamp_ms = 1010;
    nominal.x_uT = 40.0f * std::cos(field_azimuth);
    nominal.y_uT = 40.0f * std::sin(field_azimuth);
    nominal.z_uT = -35.0f;

    filter.beginCycle();
    filter.predict(stationaryImu(1010));
    expect(filter.updateMagnetic(nominal), "nominal magnetic vector is accepted");

    for (uint32_t i = 1; i <= 3; ++i) {
        filter.beginCycle();
        filter.predict(stationaryImu(1010 + i * 10));
        Domain::Fusion::MagneticObservation outlier = nominal;
        outlier.timestamp_ms = 1010 + i * 10;
        outlier.x_uT = -nominal.x_uT;
        outlier.y_uT = -nominal.y_uT;
        expect(!filter.updateMagnetic(outlier),
               "opposite magnetic heading is rejected");
    }

    const Domain::Fusion::Output output = filter.output(1040);
    expect((output.status_flags &
            Domain::Fusion::STATUS_MAGNETIC_UNHEALTHY) != 0,
           "repeated magnetic spikes mark the sensor unhealthy");
    expect(std::abs(angleError(output.yaw_rad, 0.0f)) < 0.1f,
           "magnetic spikes do not rotate yaw");
}

void testEncoderPhysicalGateAndHealth()
{
    Domain::Fusion::Config config{};
    config.health_failure_rejections = 3;
    Domain::Fusion::Filter filter(config);
    filter.initialize(30000, 30000, 0.0f, true, 1000, 500);

    Domain::Fusion::EncoderObservation encoder{};
    encoder.timestamp_ms = 1000;
    filter.updateEncoder(encoder);

    for (uint32_t i = 1; i <= 3; ++i) {
        filter.beginCycle();
        filter.predict(stationaryImu(1000 + i * 100));
        encoder.timestamp_ms = 1000 + i * 100;
        encoder.left_mm += 10000;
        encoder.right_mm += 10000;
        expect(!filter.updateEncoder(encoder),
               "physically impossible encoder speed is rejected");
    }

    const Domain::Fusion::Output output = filter.output(1300);
    expect((output.status_flags &
            Domain::Fusion::STATUS_ENCODER_UNHEALTHY) != 0,
           "repeated encoder spikes mark the sensor unhealthy");
}

void testLongGapAndUsabilityExpiry()
{
    Domain::Fusion::Config config{};
    config.maximum_yaw_aiding_age_ms = 1000;
    Domain::Fusion::Filter filter(config);
    filter.initialize(30000, 30000, 0.0f, true, 1000, 500);

    filter.beginCycle();
    expect(filter.predict(stationaryImu(1500)),
           "sub-two-second IMU gap is subdivided and propagated");

    filter.beginCycle();
    expect(!filter.predict(stationaryImu(4000)),
           "unsafe multi-second IMU gap is rejected");
    const Domain::Fusion::Output degraded = filter.output(4000);
    expect((degraded.status_flags & Domain::Fusion::STATUS_DEGRADED) != 0,
           "long gap marks output degraded");

    const Domain::Fusion::Output expired = filter.output(6000);
    expect((expired.status_flags &
            Domain::Fusion::STATUS_POSITION_USABLE) != 0,
           "encoder dead reckoning remains usable without GPS");
    expect((expired.status_flags & Domain::Fusion::STATUS_YAW_USABLE) == 0,
           "yaw usability expires without aiding and IMU");
}

void testAccelerationIsNotIntegratedIntoPosition()
{
    Domain::Fusion::Filter filter;
    filter.initialize(30000, 30000, 0.0f, true, 1000, 500);
    for (uint32_t timestamp_ms = 1010;
         timestamp_ms <= 11000;
         timestamp_ms += 10) {
        Domain::Fusion::ImuObservation imu =
            stationaryImu(timestamp_ms);
        imu.accel_x_g = 0.15f;
        filter.beginCycle();
        expect(filter.predict(imu), "tilted IMU sample is accepted");
    }
    const Domain::Fusion::Output output = filter.output(11000);
    expect(std::abs(output.x_mm - 30000) <= 1,
           "accelerometer tilt does not drift X");
    expect(std::abs(output.y_mm - 30000) <= 1,
           "accelerometer tilt does not drift Y");
}

void testEncoderUpdatesPosition()
{
    Domain::Fusion::Filter filter;
    filter.initialize(30000, 30000, 0.0f, true, 1000, 500);
    Domain::Fusion::EncoderObservation encoder{};
    encoder.timestamp_ms = 1000;
    filter.updateEncoder(encoder);
    encoder.timestamp_ms = 1100;
    encoder.left_mm = 100;
    encoder.right_mm = 100;
    filter.beginCycle();
    expect(filter.updateEncoder(encoder), "encoder movement is accepted");
    const Domain::Fusion::Output output = filter.output(1100);
    expect(std::abs(output.x_mm - 30100) < 10,
           "east-facing encoder movement advances X");
    expect(std::abs(output.y_mm - 30000) < 10,
           "straight encoder movement keeps Y");
}

void testEncoderKeepsYawWhenGyroIsMissing()
{
    Domain::Fusion::Filter filter;
    filter.initialize(30000, 30000, 0.0f, true, 1000, 500);
    Domain::Fusion::EncoderObservation encoder{};
    encoder.timestamp_ms = 1000;
    filter.updateEncoder(encoder);
    encoder.timestamp_ms = 1100;
    encoder.left_mm = -90;
    encoder.right_mm = 90;
    filter.beginCycle();
    expect(filter.updateEncoder(encoder),
           "differential encoder turn is accepted without gyro");
    const Domain::Fusion::Output output = filter.output(1100);
    expect(std::abs(angleError(output.yaw_rad, 1.0f)) < 0.05f,
           "encoder differential supports yaw during gyro loss");
}

void testStableGpsRecoveryResnapsAfterThreeSamples()
{
    Domain::Fusion::Filter filter;
    filter.initialize(10000, 10000, 0.0f, true, 1000, 500);
    for (uint32_t index = 1; index <= 3; ++index) {
        filter.beginCycle();
        filter.predict(stationaryImu(1000 + index * 100));
        const bool accepted = filter.updateGps(
            gpsObservation(
                1000 + index * 100,
                20000 + static_cast<int32_t>(index),
                20000,
                500));
        expect(accepted == (index == 3),
               "large GPS recovery requires three stable samples");
    }
    const Domain::Fusion::Output output = filter.output(1300);
    expect(std::abs(output.x_mm - 20002) < 10,
           "stable GPS recovery reinitializes position");
}

void testOutsideFieldGpsCanRecoverFusion()
{
    Domain::Fusion::Filter filter;
    filter.initialize(10000, 10000, 0.0f, true, 1000, 500);
    for (uint32_t index = 1; index <= 3; ++index) {
        filter.beginCycle();
        filter.predict(stationaryImu(1000 + index * 100));
        filter.updateGps(gpsObservation(
            1000 + index * 100,
            -15000 + static_cast<int32_t>(index),
            7000,
            1000));
    }
    const Domain::Fusion::Output output = filter.output(1300);
    expect(std::abs(output.x_mm + 14998) < 10,
           "stable GPS outside the field is accepted");
    expect((output.status_flags &
            Domain::Fusion::STATUS_OUTSIDE_FIELD) != 0,
           "outside position remains visible to Navigation");
}

void testLowAccuracyGpsCannotResnapFusion()
{
    Domain::Fusion::Filter filter;
    filter.initialize(10000, 10000, 0.0f, true, 1000, 500);
    for (uint32_t index = 1; index <= 3; ++index) {
        filter.beginCycle();
        filter.predict(stationaryImu(1000 + index * 100));
        expect(!filter.updateGps(gpsObservation(
                   1000 + index * 100,
                   50000,
                   50000,
                   50000)),
               "low-accuracy GPS is rejected");
    }
    const Domain::Fusion::Output output = filter.output(1300);
    expect(std::abs(output.x_mm - 10000) < 10 &&
           std::abs(output.y_mm - 10000) < 10,
           "low-accuracy GPS cannot resnap position");
}

} // namespace

int main()
{
    testSouthWestFieldOrigin();
    testStationaryPredictionAndYawConvention();
    testQuaternionYawIntegration();
    testGpsOutlierIsRejectedWithoutSnap();
    testDelayedGpsIsProjectedToFilterTime();
    testGpsCourseFeedsBackToYaw();
    testMagneticOutlierAndHealthHysteresis();
    testEncoderPhysicalGateAndHealth();
    testLongGapAndUsabilityExpiry();
    testAccelerationIsNotIntegratedIntoPosition();
    testEncoderUpdatesPosition();
    testEncoderKeepsYawWhenGyroIsMissing();
    testStableGpsRecoveryResnapsAfterThreeSamples();
    testOutsideFieldGpsCanRecoverFusion();
    testLowAccuracyGpsCannotResnapFusion();

    if (failures != 0) {
        std::cerr << failures << " fusion test(s) failed\n";
        return 1;
    }
    std::cout << "All fusion tests passed\n";
    return 0;
}
