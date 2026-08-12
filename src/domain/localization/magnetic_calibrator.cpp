#include "magnetic_calibrator.h"

#include <math.h>
#include <string.h>

namespace Domain::Localization {
namespace {

constexpr uint8_t INVALID_DIRECTION_BIN = UINT8_MAX;

float clamp01(float value)
{
    if (value < 0.0f) return 0.0f;
    if (value > 1.0f) return 1.0f;
    return value;
}

bool solve(double* matrix, double* right, double* answer, uint8_t size)
{
    for (uint8_t column = 0; column < size; ++column) {
        uint8_t pivot = column;
        for (uint8_t row = column + 1; row < size; ++row) {
            if (fabs(matrix[row * size + column]) >
                fabs(matrix[pivot * size + column])) pivot = row;
        }
        if (fabs(matrix[pivot * size + column]) < 1.0e-12) return false;
        if (pivot != column) {
            for (uint8_t j = column; j < size; ++j) {
                const double temporary = matrix[column * size + j];
                matrix[column * size + j] = matrix[pivot * size + j];
                matrix[pivot * size + j] = temporary;
            }
            const double temporary = right[column];
            right[column] = right[pivot];
            right[pivot] = temporary;
        }
        const double diagonal = matrix[column * size + column];
        for (uint8_t row = column + 1; row < size; ++row) {
            const double factor = matrix[row * size + column] / diagonal;
            for (uint8_t j = column; j < size; ++j) {
                matrix[row * size + j] -= factor * matrix[column * size + j];
            }
            right[row] -= factor * right[column];
        }
    }
    for (int row = size - 1; row >= 0; --row) {
        double value = right[row];
        for (uint8_t column = row + 1; column < size; ++column) {
            value -= matrix[row * size + column] * answer[column];
        }
        answer[row] = value / matrix[row * size + row];
    }
    return true;
}

bool invert3(const double input[3][3], double inverse[3][3])
{
    for (uint8_t column = 0; column < 3; ++column) {
        double matrix[9]{};
        double right[3]{};
        double answer[3]{};
        memcpy(matrix, input, sizeof(matrix));
        right[column] = 1.0;
        if (!solve(matrix, right, answer, 3)) return false;
        for (uint8_t row = 0; row < 3; ++row) inverse[row][column] = answer[row];
    }
    return true;
}

bool symmetricEigen(double matrix[3][3], double vectors[3][3])
{
    memset(vectors, 0, sizeof(double) * 9);
    for (uint8_t i = 0; i < 3; ++i) vectors[i][i] = 1.0;
    for (uint8_t iteration = 0; iteration < 24; ++iteration) {
        uint8_t p = 0, q = 1;
        if (fabs(matrix[0][2]) > fabs(matrix[p][q])) { p = 0; q = 2; }
        if (fabs(matrix[1][2]) > fabs(matrix[p][q])) { p = 1; q = 2; }
        if (fabs(matrix[p][q]) < 1.0e-12) return true;
        const double angle = 0.5 * atan2(
            2.0 * matrix[p][q], matrix[q][q] - matrix[p][p]);
        const double cosine = cos(angle);
        const double sine = sin(angle);
        const double app = matrix[p][p];
        const double aqq = matrix[q][q];
        const double apq = matrix[p][q];
        matrix[p][p] = cosine * cosine * app -
            2.0 * sine * cosine * apq + sine * sine * aqq;
        matrix[q][q] = sine * sine * app +
            2.0 * sine * cosine * apq + cosine * cosine * aqq;
        matrix[p][q] = matrix[q][p] = 0.0;
        for (uint8_t r = 0; r < 3; ++r) {
            if (r == p || r == q) continue;
            const double arp = matrix[r][p];
            const double arq = matrix[r][q];
            matrix[r][p] = matrix[p][r] = cosine * arp - sine * arq;
            matrix[r][q] = matrix[q][r] = sine * arp + cosine * arq;
        }
        for (uint8_t r = 0; r < 3; ++r) {
            const double vrp = vectors[r][p];
            const double vrq = vectors[r][q];
            vectors[r][p] = cosine * vrp - sine * vrq;
            vectors[r][q] = sine * vrp + cosine * vrq;
        }
    }
    return false;
}

} // namespace

void MagneticCalibrator::start()
{
    memset(samples_, 0, sizeof(samples_));
    memset(replacement_cursor_, 0, sizeof(replacement_cursor_));
    memset(direction_bin_counts_, 0, sizeof(direction_bin_counts_));
    sample_count_ = 0;
    samples_since_fit_attempt_ = 0;
    rms_error_uT_ = 0.0f;
    memset(axis_range_uT_, 0, sizeof(axis_range_uT_));
    eigenvalue_ratio_ = 0.0f;
    direction_bins_ = 0;
    direction_target_bins_ = MINIMUM_DIRECTION_BINS;
    current_direction_bin_ = INVALID_DIRECTION_BIN;
    progress_percent_ = 0;
    need_ = MagneticCalibrationNeed::Samples;
    coverage_ready_ = false;
    result_ = MagneticCalibrationResult::Collecting;
    active_ = true;
}

void MagneticCalibrator::cancel()
{
    sample_count_ = 0;
    samples_since_fit_attempt_ = 0;
    rms_error_uT_ = 0.0f;
    memset(direction_bin_counts_, 0, sizeof(direction_bin_counts_));
    memset(axis_range_uT_, 0, sizeof(axis_range_uT_));
    eigenvalue_ratio_ = 0.0f;
    direction_bins_ = 0;
    direction_target_bins_ = MINIMUM_DIRECTION_BINS;
    current_direction_bin_ = INVALID_DIRECTION_BIN;
    progress_percent_ = 0;
    need_ = MagneticCalibrationNeed::None;
    coverage_ready_ = false;
    result_ = MagneticCalibrationResult::None;
    active_ = false;
}

uint8_t MagneticCalibrator::directionBin(
    const Sample& sample,
    const float center[3])
{
    float value[3] = {
        sample.value[0] - center[0],
        sample.value[1] - center[1],
        sample.value[2] - center[2]};
    float norm_square = value[0] * value[0] + value[1] * value[1] +
        value[2] * value[2];
    if (norm_square < 1.0e-6f) {
        // The first samples have no reliable center yet. Raw direction is
        // sufficient for limiting repeated samples until the ranges grow.
        value[0] = sample.value[0];
        value[1] = sample.value[1];
        value[2] = sample.value[2];
        norm_square = value[0] * value[0] + value[1] * value[1] +
            value[2] * value[2];
    }
    if (norm_square < 1.0e-6f || !isfinite(norm_square)) {
        return INVALID_DIRECTION_BIN;
    }

    const float maximum_absolute = fmaxf(
        fabsf(value[0]), fmaxf(fabsf(value[1]), fabsf(value[2])));
    if (!(maximum_absolute > 0.0f)) return INVALID_DIRECTION_BIN;
    constexpr float ACTIVE_COMPONENT_RATIO = 0.40f;
    const float threshold = maximum_absolute * ACTIVE_COMPONENT_RATIO;
    int8_t direction[3]{};
    for (uint8_t axis = 0; axis < 3; ++axis) {
        if (fabsf(value[axis]) >= threshold) {
            direction[axis] = value[axis] >= 0.0f ? 1 : -1;
        }
    }
    // Base-3 encoding maps {-1,0,+1}^3 to [0,26]. Remove the all-zero
    // center code (13) to obtain exactly 26 directional bins.
    const uint8_t code = static_cast<uint8_t>(
        (direction[0] + 1) * 9 +
        (direction[1] + 1) * 3 +
        (direction[2] + 1));
    if (code == 13U) return INVALID_DIRECTION_BIN;
    return code < 13U ? code : static_cast<uint8_t>(code - 1U);
}

void MagneticCalibrator::compactDirectionBins(const float center[3])
{
    uint8_t counts[DIRECTION_BIN_COUNT]{};
    uint16_t write = 0;
    for (uint16_t read = 0; read < sample_count_; ++read) {
        const uint8_t bin = directionBin(samples_[read], center);
        if (bin == INVALID_DIRECTION_BIN ||
            counts[bin] >= MAX_SAMPLES_PER_BIN) {
            continue;
        }
        if (write != read) samples_[write] = samples_[read];
        ++write;
        ++counts[bin];
    }
    sample_count_ = write;
}

void MagneticCalibrator::updateCoverage()
{
    direction_bins_ = 0;
    direction_target_bins_ = MINIMUM_DIRECTION_BINS;
    eigenvalue_ratio_ = 0.0f;
    progress_percent_ = 0;
    coverage_ready_ = false;
    need_ = MagneticCalibrationNeed::Samples;
    memset(direction_bin_counts_, 0, sizeof(direction_bin_counts_));
    memset(axis_range_uT_, 0, sizeof(axis_range_uT_));
    if (sample_count_ == 0U) return;

    double mean[3]{};
    float minimum[3] = {
        samples_[0].value[0], samples_[0].value[1], samples_[0].value[2]};
    float maximum[3] = {minimum[0], minimum[1], minimum[2]};
    for (uint16_t i = 0; i < sample_count_; ++i) {
        for (uint8_t axis = 0; axis < 3; ++axis) {
            const float value = samples_[i].value[axis];
            mean[axis] += value;
            if (value < minimum[axis]) minimum[axis] = value;
            if (value > maximum[axis]) maximum[axis] = value;
        }
    }
    float center[3]{};
    for (uint8_t axis = 0; axis < 3; ++axis) {
        mean[axis] /= sample_count_;
        center[axis] = 0.5f * (minimum[axis] + maximum[axis]);
        axis_range_uT_[axis] = maximum[axis] - minimum[axis];
    }

    double covariance[3][3]{};
    for (uint16_t i = 0; i < sample_count_; ++i) {
        const uint8_t bin = directionBin(samples_[i], center);
        if (bin != INVALID_DIRECTION_BIN &&
            direction_bin_counts_[bin] < UINT8_MAX) {
            ++direction_bin_counts_[bin];
        }
        double difference[3]{};
        for (uint8_t axis = 0; axis < 3; ++axis) {
            difference[axis] = samples_[i].value[axis] - mean[axis];
        }
        for (uint8_t row = 0; row < 3; ++row) {
            for (uint8_t column = 0; column < 3; ++column) {
                covariance[row][column] +=
                    difference[row] * difference[column];
            }
        }
    }
    for (uint8_t bin = 0; bin < DIRECTION_BIN_COUNT; ++bin) {
        if (direction_bin_counts_[bin] >= MINIMUM_SAMPLES_PER_BIN) {
            ++direction_bins_;
        }
    }
    for (uint8_t row = 0; row < 3; ++row) {
        for (uint8_t column = 0; column < 3; ++column) {
            covariance[row][column] /= sample_count_;
        }
    }
    double vectors[3][3]{};
    if (symmetricEigen(covariance, vectors)) {
        double smallest = covariance[0][0];
        double largest = covariance[0][0];
        for (uint8_t axis = 1; axis < 3; ++axis) {
            if (covariance[axis][axis] < smallest) {
                smallest = covariance[axis][axis];
            }
            if (covariance[axis][axis] > largest) {
                largest = covariance[axis][axis];
            }
        }
        if (isfinite(smallest) && isfinite(largest) && largest > 1.0e-9) {
            eigenvalue_ratio_ = static_cast<float>(
                fmax(0.0, smallest) / largest);
        }
    }

    float progress = clamp01(
        static_cast<float>(sample_count_) / MINIMUM_SAMPLE_COUNT);
    progress = fminf(progress, clamp01(
        static_cast<float>(direction_bins_) / MINIMUM_DIRECTION_BINS));
    for (uint8_t axis = 0; axis < 3; ++axis) {
        progress = fminf(progress, clamp01(
            axis_range_uT_[axis] / MINIMUM_AXIS_RANGE_UT));
    }
    progress = fminf(progress, clamp01(
        eigenvalue_ratio_ / MINIMUM_EIGENVALUE_RATIO));
    const bool basic_coverage_ready =
        sample_count_ >= MINIMUM_SAMPLE_COUNT &&
        direction_bins_ >= MINIMUM_DIRECTION_BINS &&
        axis_range_uT_[0] >= MINIMUM_AXIS_RANGE_UT &&
        axis_range_uT_[1] >= MINIMUM_AXIS_RANGE_UT &&
        axis_range_uT_[2] >= MINIMUM_AXIS_RANGE_UT &&
        eigenvalue_ratio_ >= MINIMUM_EIGENVALUE_RATIO;
    if (sample_count_ < MINIMUM_SAMPLE_COUNT) {
        need_ = MagneticCalibrationNeed::Samples;
    } else if (axis_range_uT_[0] < MINIMUM_AXIS_RANGE_UT) {
        need_ = MagneticCalibrationNeed::RangeX;
    } else if (axis_range_uT_[1] < MINIMUM_AXIS_RANGE_UT) {
        need_ = MagneticCalibrationNeed::RangeY;
    } else if (axis_range_uT_[2] < MINIMUM_AXIS_RANGE_UT) {
        need_ = MagneticCalibrationNeed::RangeZ;
    } else if (eigenvalue_ratio_ < MINIMUM_EIGENVALUE_RATIO) {
        need_ = MagneticCalibrationNeed::Distribution;
    } else if (direction_bins_ < MINIMUM_DIRECTION_BINS) {
        need_ = MagneticCalibrationNeed::Directions;
    } else if (samples_since_fit_attempt_ < FIT_RETRY_SAMPLE_COUNT) {
        need_ = MagneticCalibrationNeed::FitRetry;
    } else {
        need_ = MagneticCalibrationNeed::None;
    }
    coverage_ready_ = basic_coverage_ready &&
        samples_since_fit_attempt_ >= FIT_RETRY_SAMPLE_COUNT;
    const uint8_t rounded_progress = static_cast<uint8_t>(
        lroundf(100.0f * progress));
    progress_percent_ = coverage_ready_
        ? 100U
        : (rounded_progress >= 100U ? 99U : rounded_progress);
}

bool MagneticCalibrator::add(const Sensor::MagneticData& sample)
{
    if (!active_ || sample.metadata.source != Sensor::Source::BoardI2c ||
        !sample.metadata.valid || !isfinite(sample.x_uT) ||
        !isfinite(sample.y_uT) || !isfinite(sample.z_uT)) return false;
    const Sample incoming{{sample.x_uT, sample.y_uT, sample.z_uT}};

    float minimum[3] = {sample.x_uT, sample.y_uT, sample.z_uT};
    float maximum[3] = {sample.x_uT, sample.y_uT, sample.z_uT};
    for (uint16_t i = 0; i < sample_count_; ++i) {
        for (uint8_t axis = 0; axis < 3; ++axis) {
            if (samples_[i].value[axis] < minimum[axis]) {
                minimum[axis] = samples_[i].value[axis];
            }
            if (samples_[i].value[axis] > maximum[axis]) {
                maximum[axis] = samples_[i].value[axis];
            }
        }
    }
    const float center[3] = {
        0.5f * (minimum[0] + maximum[0]),
        0.5f * (minimum[1] + maximum[1]),
        0.5f * (minimum[2] + maximum[2])};
    compactDirectionBins(center);

    const uint8_t incoming_bin = directionBin(incoming, center);
    if (incoming_bin == INVALID_DIRECTION_BIN) return false;
    current_direction_bin_ = incoming_bin;
    uint8_t bin_count = 0;
    for (uint16_t i = 0; i < sample_count_; ++i) {
        if (directionBin(samples_[i], center) == incoming_bin) {
            ++bin_count;
        }
    }
    if (bin_count < MAX_SAMPLES_PER_BIN &&
        sample_count_ < SAMPLE_CAPACITY) {
        samples_[sample_count_++] = incoming;
    } else {
        const uint16_t replacement_divisor =
            bin_count > 0U ? bin_count : 1U;
        const uint8_t replacement = static_cast<uint8_t>(
            replacement_cursor_[incoming_bin]++ %
            replacement_divisor);
        uint8_t occurrence = 0;
        for (uint16_t i = 0; i < sample_count_; ++i) {
            if (directionBin(samples_[i], center) != incoming_bin) continue;
            if (occurrence++ == replacement) {
                samples_[i] = incoming;
                break;
            }
        }
    }
    if (samples_since_fit_attempt_ < UINT16_MAX) {
        ++samples_since_fit_attempt_;
    }
    updateCoverage();
    if (coverage_ready_) active_ = false;
    return true;
}

bool MagneticCalibrator::finish(MagneticCalibration& calibration)
{
    if (active_ || !coverage_ready_) return false;
    double mean[3]{};
    double minimum[3] = {samples_[0].value[0], samples_[0].value[1], samples_[0].value[2]};
    double maximum[3] = {minimum[0], minimum[1], minimum[2]};
    for (uint16_t i = 0; i < sample_count_; ++i) {
        for (uint8_t axis = 0; axis < 3; ++axis) {
            const double value = samples_[i].value[axis];
            mean[axis] += value;
            if (value < minimum[axis]) minimum[axis] = value;
            if (value > maximum[axis]) maximum[axis] = value;
        }
    }
    for (uint8_t axis = 0; axis < 3; ++axis) {
        mean[axis] /= sample_count_;
        if (maximum[axis] - minimum[axis] < MINIMUM_AXIS_RANGE_UT) {
            result_ = MagneticCalibrationResult::InsufficientCoverage;
            need_ = static_cast<MagneticCalibrationNeed>(
                static_cast<uint8_t>(MagneticCalibrationNeed::RangeX) + axis);
            return false;
        }
    }
    double scale_square = 0.0;
    for (uint16_t i = 0; i < sample_count_; ++i) {
        for (uint8_t axis = 0; axis < 3; ++axis) {
            const double difference = samples_[i].value[axis] - mean[axis];
            scale_square += difference * difference;
        }
    }
    const double scale = sqrt(scale_square / (3.0 * sample_count_));
    if (!isfinite(scale) || scale < 1.0) {
        result_ = MagneticCalibrationResult::InsufficientCoverage;
        need_ = MagneticCalibrationNeed::Distribution;
        return false;
    }

    double normal[81]{};
    double right[9]{};
    for (uint16_t i = 0; i < sample_count_; ++i) {
        double u[3]{};
        for (uint8_t axis = 0; axis < 3; ++axis) {
            u[axis] = (samples_[i].value[axis] - mean[axis]) / scale;
        }
        const double row[9] = {
            u[0] * u[0], u[1] * u[1], u[2] * u[2],
            2.0 * u[0] * u[1], 2.0 * u[0] * u[2], 2.0 * u[1] * u[2],
            2.0 * u[0], 2.0 * u[1], 2.0 * u[2]};
        for (uint8_t r = 0; r < 9; ++r) {
            right[r] += row[r];
            for (uint8_t c = 0; c < 9; ++c) normal[r * 9 + c] += row[r] * row[c];
        }
    }
    double answer[9]{};
    if (!solve(normal, right, answer, 9)) {
        result_ = MagneticCalibrationResult::Singular;
        need_ = MagneticCalibrationNeed::FitQuality;
        return false;
    }
    const double a[3][3] = {
        {answer[0], answer[3], answer[4]},
        {answer[3], answer[1], answer[5]},
        {answer[4], answer[5], answer[2]}};
    const double b[3] = {answer[6], answer[7], answer[8]};
    double inverse_a[3][3]{};
    if (!invert3(a, inverse_a)) {
        result_ = MagneticCalibrationResult::Singular;
        need_ = MagneticCalibrationNeed::FitQuality;
        return false;
    }
    double center[3]{};
    for (uint8_t r = 0; r < 3; ++r) {
        for (uint8_t c = 0; c < 3; ++c) center[r] -= inverse_a[r][c] * b[c];
    }
    double denominator = 1.0;
    for (uint8_t r = 0; r < 3; ++r) {
        for (uint8_t c = 0; c < 3; ++c) denominator += center[r] * a[r][c] * center[c];
    }
    if (!isfinite(denominator) || fabs(denominator) < 1.0e-9) {
        result_ = MagneticCalibrationResult::InvalidFit;
        need_ = MagneticCalibrationNeed::FitQuality;
        return false;
    }
    double shape[3][3]{};
    for (uint8_t r = 0; r < 3; ++r) {
        for (uint8_t c = 0; c < 3; ++c) shape[r][c] = a[r][c] / denominator;
    }
    double vectors[3][3]{};
    if (!symmetricEigen(shape, vectors)) {
        result_ = MagneticCalibrationResult::InvalidFit;
        need_ = MagneticCalibrationNeed::FitQuality;
        return false;
    }
    double smallest = shape[0][0], largest = shape[0][0];
    for (uint8_t i = 0; i < 3; ++i) {
        if (!isfinite(shape[i][i]) || shape[i][i] <= 0.0) {
            result_ = MagneticCalibrationResult::InvalidFit;
            need_ = MagneticCalibrationNeed::FitQuality;
            return false;
        }
        if (shape[i][i] < smallest) smallest = shape[i][i];
        if (shape[i][i] > largest) largest = shape[i][i];
    }
    if (largest / smallest > 100.0) {
        result_ = MagneticCalibrationResult::InvalidFit;
        need_ = MagneticCalibrationNeed::FitQuality;
        return false;
    }

    double target = 0.0;
    for (uint16_t i = 0; i < sample_count_; ++i) {
        double norm_square = 0.0;
        for (uint8_t axis = 0; axis < 3; ++axis) {
            const double raw_center = mean[axis] + scale * center[axis];
            const double value = samples_[i].value[axis] - raw_center;
            norm_square += value * value;
        }
        target += sqrt(norm_square);
    }
    target /= sample_count_;
    if (!isfinite(target) || target < 10.0 || target > 100.0) {
        result_ = MagneticCalibrationResult::InvalidFit;
        need_ = MagneticCalibrationNeed::FitQuality;
        return false;
    }

    MagneticCalibration candidate{};
    for (uint8_t axis = 0; axis < 3; ++axis) {
        candidate.hard_iron_uT[axis] = static_cast<float>(mean[axis] + scale * center[axis]);
    }
    for (uint8_t r = 0; r < 3; ++r) {
        for (uint8_t c = 0; c < 3; ++c) {
            double value = 0.0;
            for (uint8_t k = 0; k < 3; ++k) {
                value += vectors[r][k] * sqrt(shape[k][k]) * vectors[c][k];
            }
            candidate.soft_iron[r][c] = static_cast<float>(target * value / scale);
        }
    }
    uint16_t corrected_bin_counts[DIRECTION_BIN_COUNT]{};
    const float zero_center[3]{};
    double error_square = 0.0;
    for (uint16_t i = 0; i < sample_count_; ++i) {
        double corrected[3]{};
        for (uint8_t r = 0; r < 3; ++r) {
            for (uint8_t c = 0; c < 3; ++c) {
                corrected[r] += candidate.soft_iron[r][c] *
                    (samples_[i].value[c] - candidate.hard_iron_uT[c]);
            }
        }
        const double norm = sqrt(corrected[0] * corrected[0] +
            corrected[1] * corrected[1] + corrected[2] * corrected[2]);
        const double error = norm - target;
        error_square += error * error;
        const Sample corrected_sample{{
            static_cast<float>(corrected[0]),
            static_cast<float>(corrected[1]),
            static_cast<float>(corrected[2])}};
        const uint8_t bin = directionBin(corrected_sample, zero_center);
        if (bin != INVALID_DIRECTION_BIN) ++corrected_bin_counts[bin];
    }
    uint8_t corrected_direction_bins = 0;
    direction_target_bins_ = MINIMUM_FITTED_DIRECTION_BINS;
    memset(direction_bin_counts_, 0, sizeof(direction_bin_counts_));
    for (uint8_t bin = 0; bin < DIRECTION_BIN_COUNT; ++bin) {
        direction_bin_counts_[bin] = static_cast<uint8_t>(fmin(
            static_cast<double>(UINT8_MAX),
            static_cast<double>(corrected_bin_counts[bin])));
        if (corrected_bin_counts[bin] >= MINIMUM_SAMPLES_PER_BIN) {
            ++corrected_direction_bins;
        }
    }
    if (corrected_direction_bins < MINIMUM_FITTED_DIRECTION_BINS) {
        // A min/max center can make a hemisphere look balanced. Validate
        // again around the fitted hard-iron center, then keep collecting
        // instead of forcing the operator to restart the calibration.
        direction_bins_ = corrected_direction_bins;
        progress_percent_ = static_cast<uint8_t>(fminf(
            99.0f,
            100.0f * corrected_direction_bins /
                MINIMUM_FITTED_DIRECTION_BINS));
        coverage_ready_ = false;
        samples_since_fit_attempt_ = 0;
        need_ = MagneticCalibrationNeed::Directions;
        result_ = MagneticCalibrationResult::Collecting;
        active_ = true;
        return false;
    }
    rms_error_uT_ = static_cast<float>(sqrt(error_square / sample_count_));
    if (!isfinite(rms_error_uT_) ||
        rms_error_uT_ > target * MAXIMUM_NORMALIZED_RMS_ERROR) {
        result_ = MagneticCalibrationResult::InvalidFit;
        need_ = MagneticCalibrationNeed::FitQuality;
        return false;
    }
    candidate.valid = true;
    calibration = candidate;
    direction_bins_ = corrected_direction_bins;
    need_ = MagneticCalibrationNeed::None;
    result_ = MagneticCalibrationResult::Success;
    return true;
}

} // namespace Domain::Localization
