#pragma once

#include <math.h>

#include "localization_config.h"
#include "localization_types.h"

namespace Domain::Localization {

class HealthTracker {
public:
    void reset()
    {
        report_ = {};
        report_.state = SensorHealth::Failed;
    }

    void noteSample(const Sensor::SampleMetadata& metadata)
    {
        report_.last_timestamp_us = metadata.timestamp_us;
        report_.last_received_us = metadata.received_us;
    }

    void noteAccepted(float innovation = 0.0f, float mahalanobis = 0.0f)
    {
        report_.last_innovation = innovation;
        report_.last_mahalanobis = mahalanobis;
        report_.consecutive_bad = 0;
        if (report_.consecutive_good != UINT16_MAX) {
            ++report_.consecutive_good;
        }
    }

    void noteRejected(float innovation = 0.0f, float mahalanobis = 0.0f)
    {
        report_.last_innovation = innovation;
        report_.last_mahalanobis = mahalanobis;
        report_.consecutive_good = 0;
        if (report_.consecutive_bad != UINT16_MAX) {
            ++report_.consecutive_bad;
        }
    }

    SensorHealth update(
        uint64_t now_us,
        uint64_t stale_us,
        uint64_t failed_us,
        const Config& config)
    {
        if (report_.last_received_us == 0U || now_us < report_.last_received_us) {
            report_.state = SensorHealth::Failed;
        } else {
            const uint64_t age_us = now_us - report_.last_received_us;
            if (age_us > failed_us ||
                report_.consecutive_bad >= config.health_failure_rejections) {
                report_.state = SensorHealth::Failed;
            } else if (age_us > stale_us) {
                report_.state = SensorHealth::Stale;
            } else if (report_.state == SensorHealth::Failed &&
                       report_.consecutive_good <
                           config.health_recovery_accepts) {
                report_.state = SensorHealth::Stale;
            } else {
                report_.state = SensorHealth::Fresh;
            }
        }
        return report_.state;
    }

    void setActive(bool active) { report_.active = active; }
    const SensorHealthReport& report() const { return report_; }

private:
    SensorHealthReport report_{};
};

} // namespace Domain::Localization
