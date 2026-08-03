#include "tasks.h"

#include "app_queue.h"
#include "app_types.h"

#include <math.h>

namespace {

constexpr uint32_t ARBITER_PERIOD_MS = 10;
constexpr int16_t MAX_VELOCITY_MM_S = 1000;
constexpr int16_t MAX_OMEGA_X100 = 500;

constexpr uint8_t sourceIndex(MotionCommandSource source)
{
    return static_cast<uint8_t>(source);
}

int16_t clampInt16(int16_t value, int16_t minimum, int16_t maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

} // namespace

void taskMotionArbiter(void* pvParameters)
{
    (void)pvParameters;

    constexpr uint8_t SOURCE_COUNT =
        static_cast<uint8_t>(MotionCommandSource::Safety) + 1U;
    MotionCommandRequest latest[SOURCE_COUNT] = {};
    bool valid[SOURCE_COUNT] = {};

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(ARBITER_PERIOD_MS);

    while (true) {
        xTaskDelayUntil(&last_wake, period);
        const uint32_t now_ms = millis();

        MotionCommandRequest incoming{};
        while (xQueueReceive(
                   fifo_motion_command_request,
                   &incoming,
                   0) == pdTRUE) {
            const uint8_t index = sourceIndex(incoming.source);
            if (index < SOURCE_COUNT) {
                latest[index] = incoming;
                valid[index] = true;
            }
        }

        int selected = -1;
        for (uint8_t index = 0; index < SOURCE_COUNT; ++index) {
            if (valid[index] &&
                static_cast<uint32_t>(
                    now_ms - latest[index].timestamp_ms) >=
                    latest[index].duration_ms) {
                valid[index] = false;
            }
            // enum値がそのまま優先度。後ろほど優先する。
            if (valid[index]) selected = index;
        }

        JogData jog{};
        jog.timestamp_ms = now_ms;
        jog.duration_ms = ARBITER_PERIOD_MS * 3U;
        jog.source = JogSource::Navigation;

        if (selected >= 0) {
            const MotionCommandRequest& request = latest[selected];
            float scale = 1.0f;

            Coordinate coordinate{};
            const bool has_coordinate =
                xQueuePeek(mbx_coordinate, &coordinate, 0) == pdTRUE;
            if (request.source == MotionCommandSource::GpsNavigation) {
                if (!has_coordinate ||
                    coordinate.fusion_quality ==
                        Domain::Fusion::Quality::Failed ||
                    coordinate.fusion_quality ==
                        Domain::Fusion::Quality::Unreliable) {
                    scale = 0.0f;
                } else if (coordinate.fusion_quality ==
                           Domain::Fusion::Quality::Degraded) {
                    scale = 0.5f;
                }
            } else if (
                request.source ==
                    MotionCommandSource::CameraNavigation &&
                (!has_coordinate ||
                 coordinate.imu_health ==
                    Domain::Fusion::SensorHealth::Failed)) {
                scale = 0.0f;
            }

            jog.velocity_mm_s = static_cast<float>(clampInt16(
                static_cast<int16_t>(lroundf(
                    request.velocity_mm_s * scale)),
                -MAX_VELOCITY_MM_S,
                MAX_VELOCITY_MM_S));
            jog.omega_rad_s =
                static_cast<float>(clampInt16(
                    static_cast<int16_t>(lroundf(
                        request.omega_rad_s_x100 * scale)),
                    -MAX_OMEGA_X100,
                    MAX_OMEGA_X100)) /
                100.0f;
            jog.source =
                request.source == MotionCommandSource::Manual
                    ? JogSource::Manual
                    : JogSource::Navigation;
        }

        // 候補がない場合もゼロを明示し、CAN側の停止を保証する。
        xQueueOverwrite(mbx_can_jog_cmd, &jog);
    }
}
