#include <assert.h>
#include <math.h>

#include "domain/localization/magnetic_calibrator.h"

int main()
{
    using namespace Domain::Localization;
    MagneticCalibrator calibrator;
    calibrator.start();
    constexpr float radius = 48.0f;
    constexpr float golden_angle = 2.39996323f;
    for (uint16_t i = 0; i < MagneticCalibrator::SAMPLE_CAPACITY; ++i) {
        const float z = 1.0f - 2.0f * (i + 0.5f) /
            MagneticCalibrator::SAMPLE_CAPACITY;
        const float radial = sqrtf(1.0f - z * z);
        const float angle = golden_angle * i;
        const float field[3] = {
            radius * radial * cosf(angle),
            radius * radial * sinf(angle),
            radius * z};
        Sensor::MagneticData sample{};
        sample.x_uT = 12.0f + 1.30f * field[0] + 0.12f * field[1];
        sample.y_uT = -8.0f + 0.05f * field[0] + 0.78f * field[1];
        sample.z_uT = 21.0f + 0.08f * field[0] + 1.12f * field[2];
        sample.metadata = {i + 1U, i + 1U, Sensor::Source::BoardI2c, true};
        assert(calibrator.add(sample));
    }
    MagneticCalibration calibration{};
    assert(calibrator.finish(calibration));
    assert(calibration.valid);
    assert(fabsf(calibration.hard_iron_uT[0] - 12.0f) < 0.2f);
    assert(fabsf(calibration.hard_iron_uT[1] + 8.0f) < 0.2f);
    assert(fabsf(calibration.hard_iron_uT[2] - 21.0f) < 0.2f);
    assert(calibrator.rmsErrorUT() < 0.2f);
    return 0;
}
