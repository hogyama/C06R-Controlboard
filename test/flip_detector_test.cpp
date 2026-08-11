#include <assert.h>

#include "domain/motion/flip_detector.h"

namespace {

using Domain::Motion::FlipDetector;
using Domain::Motion::FlipState;

Sensor::AccelerometerData acceleration(
    uint64_t timestamp_us,
    float z_g,
    Sensor::Source source = Sensor::Source::BoardI2c)
{
    Sensor::AccelerometerData value{};
    value.z_m_s2 = z_g * 9.80665f;
    value.metadata.timestamp_us = timestamp_us;
    value.metadata.received_us = timestamp_us;
    value.metadata.source = source;
    value.metadata.valid = true;
    return value;
}

void testFlipRequiresFiveHundredMilliseconds()
{
    FlipDetector detector{};
    auto result = detector.update(1000000ULL, acceleration(1000000ULL, -1.0f));
    assert(result.state != FlipState::Flipped);
    result = detector.update(1490000ULL, acceleration(1490000ULL, -1.0f));
    assert(result.state != FlipState::Flipped);
    result = detector.update(1500000ULL, acceleration(1500000ULL, -1.0f));
    assert(result.state == FlipState::Flipped);
}

void testDynamicAccelerationIsUnknown()
{
    FlipDetector detector{};
    Sensor::AccelerometerData value = acceleration(1000000ULL, -1.0f);
    value.x_m_s2 = 2.0f * 9.80665f;
    const auto result = detector.update(1000000ULL, value);
    assert(result.state == FlipState::Unknown);
}

void testStaleAccelerationIsUnknown()
{
    FlipDetector detector{};
    const auto result = detector.update(
        1400000ULL, acceleration(1000000ULL, -1.0f));
    assert(result.state == FlipState::Unknown);
}

void testFreshnessUsesEspReceiveTime()
{
    FlipDetector detector{};
    Sensor::AccelerometerData value = acceleration(1000000ULL, -1.0f);
    value.metadata.received_us = 2000000ULL;
    const auto result = detector.update(2000000ULL, value);
    assert(result.state == FlipState::HighTilt);
    assert(result.timestamp_us == 2000000ULL);
}

void testSourceChangeRestartsConfirmation()
{
    FlipDetector detector{};
    detector.update(1000000ULL, acceleration(1000000ULL, -1.0f));
    auto result = detector.update(
        1490000ULL,
        acceleration(1490000ULL, -1.0f, Sensor::Source::Can));
    assert(result.state != FlipState::Flipped);
    result = detector.update(
        1990000ULL,
        acceleration(1990000ULL, -1.0f, Sensor::Source::Can));
    assert(result.state == FlipState::Flipped);
}

} // namespace

int main()
{
    testFlipRequiresFiveHundredMilliseconds();
    testDynamicAccelerationIsUnknown();
    testStaleAccelerationIsUnknown();
    testFreshnessUsesEspReceiveTime();
    testSourceChangeRestartsConfirmation();
    return 0;
}
