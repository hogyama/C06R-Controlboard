#pragma once

#include "localization_preprocessor.h"

#include <stdint.h>

namespace Domain::Localization {

enum class MagneticCalibrationResult : uint8_t {
    None = 0,
    Collecting,
    Success,
    InsufficientCoverage,
    Singular,
    InvalidFit
};

class MagneticCalibrator {
public:
    static constexpr uint16_t SAMPLE_CAPACITY = 1200;

    void start();
    void cancel();
    bool add(const Sensor::MagneticData& board_magnetic_body);
    bool finish(MagneticCalibration& calibration);

    bool active() const { return active_; }
    uint16_t sampleCount() const { return sample_count_; }
    MagneticCalibrationResult result() const { return result_; }
    float rmsErrorUT() const { return rms_error_uT_; }

private:
    struct Sample { float value[3]; };

    Sample samples_[SAMPLE_CAPACITY]{};
    uint16_t sample_count_ = 0;
    MagneticCalibrationResult result_ = MagneticCalibrationResult::None;
    float rms_error_uT_ = 0.0f;
    bool active_ = false;
};

} // namespace Domain::Localization
