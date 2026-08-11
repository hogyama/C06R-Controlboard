#pragma once

#include "sensor_types.h"

namespace Sensor {

// timestamp_us is the sensor sampling time and may use a hardware clock.
// received_us is in the ESP timer domain, so only it is suitable for freshness.
inline bool sampleIsFresh(
    const SampleMetadata& metadata,
    uint64_t now_us,
    uint64_t maximum_age_us)
{
    return metadata.valid && metadata.received_us != 0U &&
        now_us >= metadata.received_us &&
        now_us - metadata.received_us <= maximum_age_us;
}

} // namespace Sensor
