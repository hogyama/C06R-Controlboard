#include "tasks.h"

#include "app_queue.h"
#include "app_types.h"

#include <math.h>

namespace {

constexpr uint32_t ARBITER_PERIOD_MS = 10;
constexpr int16_t MAX_VELOCITY_MM_S = 1000;
constexpr int16_t MAX_OMEGA_X100 = 500;
constexpr int16_t GPS_YAW_AIDING_MIN_COMMAND_MM_S = 400;
constexpr int16_t RECOVERY_MAX_VELOCITY_MM_S = 500;

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
        jog.nav_hold_reason = NavHoldReason::NoCommand;

        if (selected >= 0) {
            const MotionCommandRequest& request = latest[selected];
            float scale = 1.0f;
            NavHoldReason hold_reason = request.nav_hold_reason;

            Coordinate coordinate{};
            const bool has_coordinate =
                xQueuePeek(mbx_coordinate, &coordinate, 0) == pdTRUE;
            if (request.source == MotionCommandSource::GpsNavigation) {
                const bool position_usable = has_coordinate &&
                    (coordinate.localization_status_flags &
                     Domain::Localization::STATUS_POSITION_USABLE) != 0U;
                const bool yaw_usable = has_coordinate &&
                    (coordinate.localization_status_flags &
                     Domain::Localization::STATUS_YAW_USABLE) != 0U;
                if (!has_coordinate) {
                    scale = 0.0f;
                    hold_reason = NavHoldReason::CoordinateUnavailable;
                } else if (!position_usable) {
                    scale = 0.0f;
                    hold_reason = NavHoldReason::PositionUnusable;
                } else if (!yaw_usable ||
                           coordinate.localization_quality ==
                               Domain::Localization::Quality::Failed) {
                    scale = 0.0f;
                    hold_reason = !yaw_usable
                        ? NavHoldReason::YawUnusable
                        : NavHoldReason::LocalizationFailed;
                }
            } else if (
                request.source ==
                    MotionCommandSource::NavigationRecovery) {
                SystemData system{};
                const bool recovery_allowed =
                    xQueuePeek(mbx_system_data, &system, 0) == pdTRUE &&
                    system.state == SystemState::STATE_GPS_NAV;
                if (!recovery_allowed) {
                    scale = 0.0f;
                    hold_reason = NavHoldReason::ArbiterSafety;
                }
            }

            int16_t scaled_velocity = static_cast<int16_t>(lroundf(
                request.velocity_mm_s * scale));
            if (request.source == MotionCommandSource::GpsNavigation &&
                scale > 0.0f && request.velocity_mm_s != 0 &&
                abs(scaled_velocity) < GPS_YAW_AIDING_MIN_COMMAND_MM_S) {
                scaled_velocity = request.velocity_mm_s > 0
                    ? GPS_YAW_AIDING_MIN_COMMAND_MM_S
                    : -GPS_YAW_AIDING_MIN_COMMAND_MM_S;
            }
            const int16_t velocity_limit =
                request.source == MotionCommandSource::NavigationRecovery
                    ? RECOVERY_MAX_VELOCITY_MM_S
                    : MAX_VELOCITY_MM_S;
            scaled_velocity = clampInt16(
                scaled_velocity,
                static_cast<int16_t>(-velocity_limit),
                velocity_limit);
            jog.velocity_mm_s = static_cast<float>(scaled_velocity);
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
            jog.nav_hold_reason = hold_reason;
            jog.recovery_phase = request.recovery_phase;
            jog.navigation_reset_count = request.navigation_reset_count;
            jog.jog_before_scale_mm_s = request.velocity_mm_s;
            jog.jog_after_scale_mm_s = scaled_velocity;
        }

        // 候補がない場合もゼロを明示し、CAN側の停止を保証する。
        xQueueOverwrite(mbx_can_jog_cmd, &jog);
    }
}
