#include "localization.h"

#include <math.h>
#include <string.h>

namespace Domain::Localization {
namespace {

Sensor::SampleMetadata metadataForGyro(
    const GyroPreintegration& gyroscope)
{
    return {
        gyroscope.end_timestamp_us,
        gyroscope.end_timestamp_us,
        gyroscope.source,
        gyroscope.valid};
}

float median(float* values, uint8_t count)
{
    for (uint8_t i = 1; i < count; ++i) {
        const float value = values[i];
        uint8_t position = i;
        while (position > 0U && values[position - 1U] > value) {
            values[position] = values[position - 1U];
            --position;
        }
        values[position] = value;
    }
    return values[count / 2U];
}

} // namespace

Localization::Localization(const Config& config)
    : config_(config), ekf_(config)
{
    reset();
}

void Localization::reset()
{
    ekf_.reset();
    memset(history_, 0, sizeof(history_));
    history_oldest_ = 0;
    history_count_ = 0;
    gps_health_.reset();
    encoder_health_.reset();
    gyro_health_.reset();
    accel_health_.reset();
    magnetic_health_.reset();
    active_sensor_flags_ = 0;
    fault_flags_ = 0;
    cycle_status_flags_ = 0;
    latest_gyro_z_rad_s_ = 0.0f;
    magnetic_field_uT_ = 0.0f;
    magnetic_reject_reason_ = MagneticRejectReason::None;
    gps_course_reject_reason_ = GpsCourseRejectReason::None;
    last_good_snapshot_ = {};
    have_last_good_snapshot_ = false;
    memset(gps_residuals_, 0, sizeof(gps_residuals_));
    gps_residual_count_ = 0;
    gps_residual_write_ = 0;
    gps_warp_count_ = 0;
    gps_warp_timestamp_us_ = 0;
    gps_warp_east_m_ = 0.0f;
    gps_warp_north_m_ = 0.0f;
}

bool Localization::recoverFromLastGood()
{
    if (!have_last_good_snapshot_) return false;
    const Ekf5::Snapshot checkpoint = last_good_snapshot_;
    memset(history_, 0, sizeof(history_));
    history_oldest_ = 0;
    history_count_ = 0;
    gps_health_.reset();
    encoder_health_.reset();
    gyro_health_.reset();
    accel_health_.reset();
    magnetic_health_.reset();
    cycle_status_flags_ = STATUS_NONE;
    fault_flags_ = FAULT_NONE;
    return ekf_.restore(checkpoint);
}

bool Localization::initializeFromGps(
    const GpsObservation& gps,
    const MagneticHeadingObservation* magnetic,
    float gyro_bias_rad_s)
{
    if (!gps.position_valid || gps.metadata.timestamp_us == 0U ||
        !isfinite(gps.east_m) || !isfinite(gps.north_m)) return false;

    float yaw = 0.0f;
    float velocity = 0.0f;
    bool yaw_initialized = false;
    if (magnetic != nullptr && magnetic->valid) {
        yaw = magnetic->heading_rad;
        yaw_initialized = true;
    } else if (gps.velocity_valid) {
        const float speed = hypotf(
            gps.velocity_east_m_s, gps.velocity_north_m_s);
        if (speed >= config_.gps_minimum_course_speed_m_s &&
            gps.speed_accuracy_m_s <=
                config_.gps_maximum_speed_accuracy_m_s) {
            yaw = atan2f(gps.velocity_north_m_s, gps.velocity_east_m_s);
            velocity = speed;
            yaw_initialized = true;
        }
    }
    if (yaw_initialized && gps.velocity_valid) {
        velocity = gps.velocity_east_m_s * cosf(yaw) +
            gps.velocity_north_m_s * sinf(yaw);
    }

    ekf_.initialize(
        gps.east_m,
        gps.north_m,
        yaw,
        velocity,
        isfinite(gyro_bias_rad_s) ? gyro_bias_rad_s : 0.0f,
        gps.metadata.timestamp_us,
        yaw_initialized);
    // Use receiver accuracy as the initial position covariance instead of
    // keeping only the broad startup covariance.
    ekf_.updateGpsPosition(gps);
    if (magnetic != nullptr && magnetic->valid) {
        ekf_.updateMagneticHeading(*magnetic);
    }
    if (gps.velocity_valid && yaw_initialized) {
        ekf_.updateGpsVelocity(gps);
    }
    gps_health_.noteSample(gps.metadata);
    gps_health_.noteAccepted();
    active_sensor_flags_ |= ACTIVE_GNSS;
    return ekf_.initialized();
}

Localization::HistoryEntry& Localization::historyAt(uint16_t logical_index)
{
    return history_[
        static_cast<uint16_t>((history_oldest_ + logical_index) %
                              HISTORY_CAPACITY)];
}

const Localization::HistoryEntry& Localization::historyAt(
    uint16_t logical_index) const
{
    return history_[
        static_cast<uint16_t>((history_oldest_ + logical_index) %
                              HISTORY_CAPACITY)];
}

void Localization::appendHistory(const HistoryEntry& entry)
{
    if (history_count_ < HISTORY_CAPACITY) {
        historyAt(history_count_++) = entry;
        return;
    }
    history_[history_oldest_] = entry;
    history_oldest_ = static_cast<uint16_t>(
        (history_oldest_ + 1U) % HISTORY_CAPACITY);
}

bool Localization::processCycle(const CycleInput& input)
{
    if (!ekf_.initialized()) return false;
    CycleInput normalized = input;
    if (normalized.timestamp_us == 0U) {
        normalized.timestamp_us = normalized.gyroscope.valid
            ? normalized.gyroscope.end_timestamp_us
            : (normalized.has_encoder
                ? normalized.encoder.metadata.timestamp_us
                : ekf_.timestampUs());
    }
    HistoryEntry entry{};
    entry.before = ekf_.snapshot();
    entry.input = normalized;
    const bool result = applyCycle(normalized, true);
    appendHistory(entry);
    return result;
}

bool Localization::applyCycle(const CycleInput& input, bool record_health)
{
    cycle_status_flags_ = STATUS_NONE;
    bool changed = false;
    bool motion_predicted = false;
    if (input.gyroscope.valid) {
        latest_gyro_z_rad_s_ = input.gyroscope.latest_z_rad_s;
        if (record_health) gyro_health_.noteSample(metadataForGyro(input.gyroscope));
        const bool accepted = ekf_.predict(input.gyroscope);
        if (record_health) {
            if (accepted) gyro_health_.noteAccepted();
            else gyro_health_.noteRejected();
        }
        if (accepted) {
            motion_predicted = true;
            cycle_status_flags_ |= STATUS_IMU_USED;
            active_sensor_flags_ &= ~(ACTIVE_GYRO_BOARD | ACTIVE_GYRO_CAN);
            active_sensor_flags_ |=
                input.gyroscope.source == Sensor::Source::BoardI2c
                    ? ACTIVE_GYRO_BOARD : ACTIVE_GYRO_CAN;
            changed = true;
            fault_flags_ &= ~FAULT_GYRO_GAP;
        } else {
            cycle_status_flags_ |= STATUS_IMU_REJECTED;
            fault_flags_ |= FAULT_GYRO_GAP;
        }
    }

    if (input.has_encoder) {
        if (record_health) encoder_health_.noteSample(input.encoder.metadata);
        if (!motion_predicted && input.encoder.valid) {
            motion_predicted = ekf_.predictEncoderMotion(input.encoder);
        }
        const bool accepted = input.encoder.valid &&
            ekf_.updateEncoderVelocity(input.encoder);
        if (record_health) {
            if (accepted) encoder_health_.noteAccepted(
                input.encoder.velocity_m_s -
                    ekf_.state()[FORWARD_VELOCITY],
                ekf_.lastMahalanobis());
            else encoder_health_.noteRejected(0.0f, ekf_.lastMahalanobis());
        }
        if (accepted) {
            cycle_status_flags_ |= STATUS_ENCODER_USED;
            active_sensor_flags_ |= ACTIVE_ENCODER_CAN;
            changed = true;
            fault_flags_ &=
                ~(FAULT_ENCODER_RANGE | FAULT_ENCODER_DISCONTINUITY);
        } else {
            cycle_status_flags_ |= STATUS_ENCODER_REJECTED;
            fault_flags_ |= FAULT_ENCODER_RANGE;
        }
    }

    if (input.timestamp_us > ekf_.timestampUs()) {
        changed = ekf_.predictCoast(input.timestamp_us) || changed;
    }

    if (input.has_magnetic) {
        magnetic_field_uT_ = input.magnetic.field_strength_uT;
        if (record_health) magnetic_health_.noteSample(input.magnetic.metadata);
        const bool accepted =
            input.magnetic.valid && ekf_.updateMagneticHeading(input.magnetic);
        if (record_health) {
            if (accepted) magnetic_health_.noteAccepted(
                Ekf5::wrapPi(
                    input.magnetic.heading_rad - ekf_.state()[YAW]),
                ekf_.lastMahalanobis());
            else magnetic_health_.noteRejected(0.0f, ekf_.lastMahalanobis());
        }
        if (accepted) {
            magnetic_reject_reason_ = MagneticRejectReason::None;
            cycle_status_flags_ |= STATUS_MAGNETIC_USED;
            active_sensor_flags_ &= ~(ACTIVE_MAG_BOARD | ACTIVE_MAG_CAN);
            active_sensor_flags_ |=
                input.magnetic.metadata.source == Sensor::Source::BoardI2c
                    ? ACTIVE_MAG_BOARD : ACTIVE_MAG_CAN;
            changed = true;
            fault_flags_ &= ~FAULT_MAG_DISTURBANCE;
        } else {
            magnetic_reject_reason_ = input.magnetic.valid
                ? MagneticRejectReason::MahalanobisGate
                : MagneticRejectReason::InvalidInput;
            cycle_status_flags_ |= STATUS_MAGNETIC_REJECTED;
            fault_flags_ |= FAULT_MAG_DISTURBANCE;
        }
    }

    if (input.stationary.stationary) {
        const bool velocity_updated = ekf_.updateZeroVelocity(0.0025f);
        const bool bias_updated = ekf_.updateZaru(
            input.stationary.mean_gyro_z_rad_s, 0.0004f);
        if (velocity_updated) active_sensor_flags_ |= ACTIVE_ZUPT;
        if (bias_updated) active_sensor_flags_ |= ACTIVE_ZARU;
        changed = changed || velocity_updated || bias_updated;
    }
    if (!ekf_.covarianceValid()) fault_flags_ |= FAULT_COVARIANCE;
    else fault_flags_ &= ~FAULT_COVARIANCE;
    return changed;
}

bool Localization::processGps(const GpsObservation& observation)
{
    if (!ekf_.initialized()) return false;
    gps_health_.noteSample(observation.metadata);
    bool accepted = false;
    if (observation.metadata.timestamp_us < ekf_.timestampUs()) {
        accepted = replayDelayedGps(observation);
        return applyGpsWarp(observation) || accepted;
    }
    if (observation.metadata.timestamp_us > ekf_.timestampUs()) {
        gps_health_.noteRejected();
        return false;
    }
    accepted = applyGpsNow(observation, true);
    return applyGpsWarp(observation) || accepted;
}

bool Localization::applyGpsNow(
    const GpsObservation& observation,
    bool record_health)
{
    bool accepted = false;
    if (observation.position_valid) {
        recordGpsResidual(observation);
        accepted = ekf_.updateGpsPosition(observation) || accepted;
    }
    if (observation.velocity_valid) {
        const float speed = hypotf(
            observation.velocity_east_m_s,
            observation.velocity_north_m_s);
        if (speed >= config_.gps_minimum_course_speed_m_s &&
            observation.speed_accuracy_m_s <=
                config_.gps_maximum_speed_accuracy_m_s) {
            accepted = ekf_.updateGpsVelocity(observation) || accepted;
            gps_course_reject_reason_ = accepted
                ? GpsCourseRejectReason::None
                : GpsCourseRejectReason::MahalanobisGate;
        } else {
            gps_course_reject_reason_ =
                speed < config_.gps_minimum_course_speed_m_s
                    ? GpsCourseRejectReason::SpeedTooLow
                    : GpsCourseRejectReason::SpeedAccuracyTooLarge;
        }
    } else {
        gps_course_reject_reason_ =
            GpsCourseRejectReason::VelocityUnavailable;
    }
    if (record_health) {
        if (accepted) gps_health_.noteAccepted(0.0f, ekf_.lastMahalanobis());
        else gps_health_.noteRejected(0.0f, ekf_.lastMahalanobis());
    }
    if (accepted) {
        cycle_status_flags_ |= STATUS_GPS_USED;
        active_sensor_flags_ |= ACTIVE_GNSS;
        fault_flags_ &= ~FAULT_GNSS_OUTLIER;
    } else {
        cycle_status_flags_ |= STATUS_GPS_REJECTED;
        fault_flags_ |= FAULT_GNSS_OUTLIER;
    }
    return accepted;
}

void Localization::recordGpsResidual(const GpsObservation& observation)
{
    if (!observation.position_valid ||
        observation.horizontal_accuracy_m >
            config_.gps_warp_maximum_accuracy_m ||
        observation.east_m < -config_.gps_warp_field_margin_m ||
        observation.north_m < -config_.gps_warp_field_margin_m ||
        observation.east_m >
            config_.field_size_x_m + config_.gps_warp_field_margin_m ||
        observation.north_m >
            config_.field_size_y_m + config_.gps_warp_field_margin_m) {
        return;
    }
    const float east = observation.east_m - ekf_.state()[POSITION_X];
    const float north = observation.north_m - ekf_.state()[POSITION_Y];
    const float distance = hypotf(east, north);
    if (!isfinite(distance)) return;
    if (distance < config_.gps_warp_residual_spread_m) {
        gps_residual_count_ = 0;
        gps_residual_write_ = 0;
        return;
    }
    if (distance < config_.gps_warp_minimum_residual_m) return;
    gps_residuals_[gps_residual_write_] = {east, north};
    gps_residual_write_ = static_cast<uint8_t>(
        (gps_residual_write_ + 1U) % 9U);
    if (gps_residual_count_ < 9U) ++gps_residual_count_;
}

bool Localization::applyGpsWarp(const GpsObservation& observation)
{
    if (gps_residual_count_ < 9U ||
        (gps_warp_timestamp_us_ != 0U &&
         observation.metadata.received_us >= gps_warp_timestamp_us_ &&
         observation.metadata.received_us - gps_warp_timestamp_us_ <
            config_.gps_warp_cooldown_us)) {
        return false;
    }
    float east_values[9]{};
    float north_values[9]{};
    for (uint8_t i = 0; i < 9U; ++i) {
        east_values[i] = gps_residuals_[i].east_m;
        north_values[i] = gps_residuals_[i].north_m;
    }
    const float east = median(east_values, 9U);
    const float north = median(north_values, 9U);
    if (hypotf(east, north) < config_.gps_warp_minimum_residual_m) {
        return false;
    }
    uint8_t consistent = 0;
    for (const GpsResidual& residual : gps_residuals_) {
        if (hypotf(residual.east_m - east, residual.north_m - north) <=
            config_.gps_warp_residual_spread_m) {
            ++consistent;
        }
    }
    if (consistent < 7U || !ekf_.shiftPosition(
            east, north, observation.horizontal_accuracy_m)) {
        return false;
    }
    for (uint16_t i = 0; i < history_count_; ++i) {
        HistoryEntry& entry = historyAt(i);
        entry.before.state[POSITION_X] += east;
        entry.before.state[POSITION_Y] += north;
    }
    if (have_last_good_snapshot_) {
        last_good_snapshot_.state[POSITION_X] += east;
        last_good_snapshot_.state[POSITION_Y] += north;
    }
    gps_warp_timestamp_us_ = observation.metadata.received_us;
    gps_warp_east_m_ = east;
    gps_warp_north_m_ = north;
    ++gps_warp_count_;
    cycle_status_flags_ |= STATUS_GPS_WARPED;
    fault_flags_ &= ~FAULT_GNSS_OUTLIER;
    gps_health_.noteAccepted();
    gps_residual_count_ = 0;
    gps_residual_write_ = 0;
    return true;
}

bool Localization::replayDelayedGps(const GpsObservation& observation)
{
    if (history_count_ == 0U ||
        observation.metadata.timestamp_us <
            historyAt(0).before.timestamp_us) {
        gps_health_.noteRejected();
        return false;
    }

    uint16_t target = history_count_;
    for (uint16_t i = 0; i < history_count_; ++i) {
        const HistoryEntry& entry = historyAt(i);
        const uint64_t end_us = entry.input.timestamp_us;
        if (observation.metadata.timestamp_us >= entry.before.timestamp_us &&
            observation.metadata.timestamp_us <= end_us) {
            target = i;
            break;
        }
    }
    if (target == history_count_) {
        gps_health_.noteRejected();
        return false;
    }

    if (!ekf_.restore(historyAt(target).before)) {
        gps_health_.noteRejected();
        return false;
    }
    bool gps_applied = false;
    bool gps_attempted = false;
    for (uint16_t i = target; i < history_count_; ++i) {
        HistoryEntry& entry = historyAt(i);
        entry.before = ekf_.snapshot();
        CycleInput replay = entry.input;
        if (!gps_attempted && replay.gyroscope.valid &&
            observation.metadata.timestamp_us >= entry.before.timestamp_us &&
            observation.metadata.timestamp_us <
                replay.gyroscope.start_timestamp_us) {
            // There is no motion sample in this gap. Apply the observation to
            // the held state before replaying the next measured interval.
            gps_applied = applyGpsNow(observation, false);
            gps_attempted = true;
        }
        if (!gps_attempted && replay.gyroscope.valid &&
            observation.metadata.timestamp_us >= replay.gyroscope.start_timestamp_us &&
            observation.metadata.timestamp_us <= replay.gyroscope.end_timestamp_us) {
            const uint64_t duration = replay.gyroscope.end_timestamp_us -
                replay.gyroscope.start_timestamp_us;
            const uint64_t first_duration = observation.metadata.timestamp_us -
                replay.gyroscope.start_timestamp_us;
            const float fraction = duration > 0U
                ? static_cast<float>(first_duration) /
                    static_cast<float>(duration)
                : 0.0f;
            GyroPreintegration first = replay.gyroscope;
            first.end_timestamp_us = observation.metadata.timestamp_us;
            first.integrated_z_rad *= fraction;
            first.valid = first.end_timestamp_us > first.start_timestamp_us;
            if (first.valid) ekf_.predict(first);
            gps_applied = applyGpsNow(observation, false);
            gps_attempted = true;
            GyroPreintegration second = replay.gyroscope;
            second.start_timestamp_us = observation.metadata.timestamp_us;
            second.integrated_z_rad *= 1.0f - fraction;
            second.valid = second.end_timestamp_us > second.start_timestamp_us;
            replay.gyroscope = second;
        } else if (!gps_attempted &&
                   observation.metadata.timestamp_us == entry.before.timestamp_us) {
            gps_applied = applyGpsNow(observation, false);
            gps_attempted = true;
        }
        applyCycle(replay, false);
    }
    if (gps_applied) {
        cycle_status_flags_ |= STATUS_GPS_USED;
        active_sensor_flags_ |= ACTIVE_GNSS;
        fault_flags_ &= ~FAULT_GNSS_OUTLIER;
        gps_health_.noteAccepted(0.0f, ekf_.lastMahalanobis());
    }
    else gps_health_.noteRejected(0.0f, ekf_.lastMahalanobis());
    return gps_applied;
}

void Localization::noteAcceleration(
    const Sensor::AccelerometerData& observation)
{
    accel_health_.noteSample(observation.metadata);
    if (observation.metadata.valid && isfinite(observation.x_m_s2) &&
        isfinite(observation.y_m_s2) && isfinite(observation.z_m_s2)) {
        accel_health_.noteAccepted();
        active_sensor_flags_ &= ~(ACTIVE_ACCEL_BOARD | ACTIVE_ACCEL_CAN);
        active_sensor_flags_ |=
            observation.metadata.source == Sensor::Source::BoardI2c
                ? ACTIVE_ACCEL_BOARD : ACTIVE_ACCEL_CAN;
    } else {
        accel_health_.noteRejected();
    }
}

LocalizationEstimate Localization::estimate(uint64_t now_us)
{
    LocalizationEstimate result{};
    result.timestamp_us = ekf_.timestampUs();
    result.active_sensor_flags = active_sensor_flags_;
    result.fault_flags = fault_flags_;
    result.status_flags = cycle_status_flags_;
    result.initialized = ekf_.initialized();
    if (!result.initialized) {
        result.quality = Quality::Uninitialized;
        return result;
    }

    const float* state = ekf_.state();
    result.px_m = state[POSITION_X];
    result.py_m = state[POSITION_Y];
    result.theta_rad = state[YAW];
    result.velocity_m_s = state[FORWARD_VELOCITY];
    result.gyro_bias_rad_s = state[GYRO_BIAS_Z];
    memcpy(result.covariance, ekf_.covariance(), sizeof(result.covariance));
    result.latest_gyro_z_rad_s = latest_gyro_z_rad_s_;
    result.magnetic_field_uT = magnetic_field_uT_;
    result.magnetic_mahalanobis = magnetic_health_.report().last_mahalanobis;
    result.magnetic_reject_reason = magnetic_reject_reason_;
    result.gps_course_reject_reason = gps_course_reject_reason_;

    result.imu_health = gyro_health_.update(
        now_us, config_.gyro_stale_us, config_.gyro_failed_us, config_);
    result.encoder_health = encoder_health_.update(
        now_us, config_.encoder_stale_us, config_.encoder_failed_us, config_);
    result.magnetic_health = magnetic_health_.update(
        now_us, config_.magnetic_stale_us, config_.magnetic_failed_us, config_);
    result.gps_health = gps_health_.update(
        now_us, config_.gps_stale_us, config_.gps_failed_us, config_);
    const SensorHealth acceleration_health = accel_health_.update(
        now_us, config_.accel_stale_us, config_.accel_failed_us, config_);

    if (result.imu_health == SensorHealth::Failed) {
        result.status_flags |= STATUS_IMU_REJECTED;
        result.fault_flags |= FAULT_GYRO_TIMEOUT;
    }
    if (acceleration_health == SensorHealth::Failed) {
        result.fault_flags |= FAULT_ACCEL_TIMEOUT;
    }
    if (result.encoder_health == SensorHealth::Failed) {
        result.status_flags |= STATUS_ENCODER_UNHEALTHY;
        result.fault_flags |= FAULT_ENCODER_TIMEOUT;
    }
    if (result.magnetic_health == SensorHealth::Failed) {
        result.status_flags |= STATUS_MAGNETIC_UNHEALTHY;
        result.fault_flags |= FAULT_MAG_TIMEOUT;
    }
    if (result.gps_health == SensorHealth::Failed) {
        result.status_flags |= STATUS_GPS_UNHEALTHY;
        result.fault_flags |= FAULT_GNSS_TIMEOUT;
    }

    result.status_flags |= STATUS_INITIALIZED;
    const float position_std = sqrtf(fmaxf(
        result.covariance[POSITION_X][POSITION_X],
        result.covariance[POSITION_Y][POSITION_Y]));
    const float yaw_std = sqrtf(result.covariance[YAW][YAW]);
    const bool position_source_available =
        result.gps_health != SensorHealth::Failed ||
        result.encoder_health != SensorHealth::Failed;
    const bool gps_course_available =
        result.gps_health != SensorHealth::Failed &&
        result.gps_course_reject_reason == GpsCourseRejectReason::None;
    const bool yaw_source_available =
        result.imu_health != SensorHealth::Failed ||
        result.encoder_health != SensorHealth::Failed ||
        result.magnetic_health != SensorHealth::Failed ||
        gps_course_available;
    const bool position_usable = position_source_available &&
        position_std <= config_.maximum_position_std_m;
    const bool yaw_usable = yaw_source_available &&
        ekf_.yawInitialized() && yaw_std <= config_.maximum_yaw_std_rad;
    if (position_usable) result.status_flags |= STATUS_POSITION_USABLE;
    if (yaw_usable) result.status_flags |= STATUS_YAW_USABLE;
    result.gps_warp_count = gps_warp_count_;
    result.gps_warp_timestamp_us = gps_warp_timestamp_us_;
    result.gps_warp_east_m = gps_warp_east_m_;
    result.gps_warp_north_m = gps_warp_north_m_;
    if (gps_warp_timestamp_us_ != 0U &&
        now_us >= gps_warp_timestamp_us_ &&
        now_us - gps_warp_timestamp_us_ <= config_.gps_warp_status_hold_us) {
        result.status_flags |= STATUS_GPS_WARPED;
    }

    const uint8_t failed_count =
        (result.imu_health == SensorHealth::Failed ? 1U : 0U) +
        (result.encoder_health == SensorHealth::Failed ? 1U : 0U) +
        (result.magnetic_health == SensorHealth::Failed ? 1U : 0U) +
        (result.gps_health == SensorHealth::Failed ? 1U : 0U);
    if (!ekf_.covarianceValid()) {
        result.quality = Quality::Failed;
    } else if (!position_usable || !yaw_usable) {
        result.quality = Quality::Failed;
    } else if (failed_count > 0U) {
        result.quality = Quality::Degraded;
        result.status_flags |= STATUS_DEGRADED;
    } else {
        result.quality = Quality::Normal;
    }
    result.valid = position_usable && yaw_usable &&
        result.quality != Quality::Failed;
    if (result.valid && ekf_.covarianceValid()) {
        last_good_snapshot_ = ekf_.snapshot();
        have_last_good_snapshot_ = true;
    }
    return result;
}

} // namespace Domain::Localization
