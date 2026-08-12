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

enum class MagneticCalibrationNeed : uint8_t {
    None = 0,
    Samples,
    Directions,
    RangeX,
    RangeY,
    RangeZ,
    Distribution,
    FitRetry,
    FitQuality
};

class MagneticCalibrator {
public:
    static constexpr uint8_t DIRECTION_BIN_COUNT = 26;
    static constexpr uint8_t MAX_SAMPLES_PER_BIN = 48;
    static constexpr uint16_t SAMPLE_CAPACITY =
        DIRECTION_BIN_COUNT * MAX_SAMPLES_PER_BIN;
    static constexpr uint16_t MINIMUM_SAMPLE_COUNT = 600;
    // Six independent samples in a direction are enough to count the bin;
    // the separate 600-sample and covariance requirements prevent a sparse
    // directional pass from starting the fit.
    static constexpr uint8_t MINIMUM_SAMPLES_PER_BIN = 6;
    static constexpr uint8_t MINIMUM_DIRECTION_BINS = 20;
    // Fitted coordinates have a more accurate center but redistribute edge
    // samples. Requiring 18 bins still rejects a plane or a hemisphere while
    // avoiding an endless collection caused by one marginal corner bin.
    static constexpr uint8_t MINIMUM_FITTED_DIRECTION_BINS = 18;
    static constexpr float MINIMUM_AXIS_RANGE_UT = 30.0f;
    static constexpr float MINIMUM_EIGENVALUE_RATIO = 0.12f;
    static constexpr float MAXIMUM_NORMALIZED_RMS_ERROR = 0.10f;
    static constexpr uint8_t FIT_RETRY_SAMPLE_COUNT = 128;

    void start();
    void cancel();
    bool add(const Sensor::MagneticData& board_magnetic_body);
    bool finish(MagneticCalibration& calibration);

    bool active() const { return active_; }
    uint16_t sampleCount() const { return sample_count_; }
    MagneticCalibrationResult result() const { return result_; }
    float rmsErrorUT() const { return rms_error_uT_; }
    uint8_t directionBins() const { return direction_bins_; }
    uint8_t directionTargetBins() const { return direction_target_bins_; }
    float axisRangeUT(uint8_t axis) const {
        return axis < 3U ? axis_range_uT_[axis] : 0.0f;
    }
    float eigenvalueRatio() const { return eigenvalue_ratio_; }
    uint8_t progressPercent() const { return progress_percent_; }
    bool coverageReady() const { return coverage_ready_; }
    MagneticCalibrationNeed need() const { return need_; }
    uint8_t currentDirectionBin() const { return current_direction_bin_; }
    uint8_t directionBinCount(uint8_t bin) const {
        return bin < DIRECTION_BIN_COUNT ? direction_bin_counts_[bin] : 0U;
    }

private:
    struct Sample { float value[3]; };

    Sample samples_[SAMPLE_CAPACITY]{};
    uint16_t replacement_cursor_[DIRECTION_BIN_COUNT]{};
    uint16_t sample_count_ = 0;
    uint16_t samples_since_fit_attempt_ = 0;
    MagneticCalibrationResult result_ = MagneticCalibrationResult::None;
    float rms_error_uT_ = 0.0f;
    float axis_range_uT_[3]{};
    float eigenvalue_ratio_ = 0.0f;
    uint8_t direction_bin_counts_[DIRECTION_BIN_COUNT]{};
    uint8_t direction_bins_ = 0;
    uint8_t direction_target_bins_ = MINIMUM_DIRECTION_BINS;
    uint8_t current_direction_bin_ = UINT8_MAX;
    uint8_t progress_percent_ = 0;
    MagneticCalibrationNeed need_ = MagneticCalibrationNeed::None;
    bool coverage_ready_ = false;
    bool active_ = false;

    static uint8_t directionBin(
        const Sample& sample, const float center[3]);
    void compactDirectionBins(const float center[3]);
    void updateCoverage();
};

} // namespace Domain::Localization
