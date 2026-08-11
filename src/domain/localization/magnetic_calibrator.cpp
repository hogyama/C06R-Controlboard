#include "magnetic_calibrator.h"

#include <math.h>
#include <string.h>

namespace Domain::Localization {
namespace {

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
    sample_count_ = 0;
    rms_error_uT_ = 0.0f;
    result_ = MagneticCalibrationResult::Collecting;
    active_ = true;
}

void MagneticCalibrator::cancel()
{
    sample_count_ = 0;
    rms_error_uT_ = 0.0f;
    result_ = MagneticCalibrationResult::None;
    active_ = false;
}

bool MagneticCalibrator::add(const Sensor::MagneticData& sample)
{
    if (!active_ || sample.metadata.source != Sensor::Source::BoardI2c ||
        !sample.metadata.valid || !isfinite(sample.x_uT) ||
        !isfinite(sample.y_uT) || !isfinite(sample.z_uT)) return false;
    samples_[sample_count_].value[0] = sample.x_uT;
    samples_[sample_count_].value[1] = sample.y_uT;
    samples_[sample_count_].value[2] = sample.z_uT;
    ++sample_count_;
    if (sample_count_ >= SAMPLE_CAPACITY) active_ = false;
    return true;
}

bool MagneticCalibrator::finish(MagneticCalibration& calibration)
{
    if (active_ || sample_count_ < SAMPLE_CAPACITY) return false;
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
        if (maximum[axis] - minimum[axis] < 20.0) {
            result_ = MagneticCalibrationResult::InsufficientCoverage;
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
        return false;
    }
    double shape[3][3]{};
    for (uint8_t r = 0; r < 3; ++r) {
        for (uint8_t c = 0; c < 3; ++c) shape[r][c] = a[r][c] / denominator;
    }
    double vectors[3][3]{};
    if (!symmetricEigen(shape, vectors)) {
        result_ = MagneticCalibrationResult::InvalidFit;
        return false;
    }
    double smallest = shape[0][0], largest = shape[0][0];
    for (uint8_t i = 0; i < 3; ++i) {
        if (!isfinite(shape[i][i]) || shape[i][i] <= 0.0) {
            result_ = MagneticCalibrationResult::InvalidFit;
            return false;
        }
        if (shape[i][i] < smallest) smallest = shape[i][i];
        if (shape[i][i] > largest) largest = shape[i][i];
    }
    if (largest / smallest > 100.0) {
        result_ = MagneticCalibrationResult::InvalidFit;
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
    }
    rms_error_uT_ = static_cast<float>(sqrt(error_square / sample_count_));
    if (!isfinite(rms_error_uT_) || rms_error_uT_ > target * 0.2) {
        result_ = MagneticCalibrationResult::InvalidFit;
        return false;
    }
    candidate.valid = true;
    calibration = candidate;
    result_ = MagneticCalibrationResult::Success;
    return true;
}

} // namespace Domain::Localization
