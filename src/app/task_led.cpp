#include "tasks.h"
#include "platform/pin_config.h"
#include "app_types.h"
#include "app_queue.h"

#include <Arduino.h>

namespace {

constexpr uint32_t LED_TASK_PERIOD_MS = 50;

/*
 * LED表示一覧
 *
 * LED_STATE（機体状態）
 * - PRELAUNCH          : 2秒ごとに100ms点灯
 * - AWAIT_ASCENT       : 2秒ごとに100msを2回点灯
 * - ASCENT_TO_LANDING  : 250ms点灯、250ms消灯
 * - SEPARATION         : 2秒ごとに100msを3回点灯
 * - GPS_NAV            : 1秒点灯、1秒消灯
 * - CAMERA_NAV         : 1秒ごとに100ms点灯
 * - ESCAPE             : 100ms点灯、100ms消灯
 * - UPRIGHT_RECOVERY   : 50ms点灯、50ms消灯
 * - GOAL               : 常時点灯
 *
 * LED_MAP（Flash保存状態）
 * - LOG保存成功        : 80msを1回点灯
 * - MAP保存成功        : 100msを2回点灯
 * - 書き込み失敗       : 100msを3回点灯（一時的な故障表示）
 * - 現在のLOGファイル満杯: 500ms点灯、500ms消灯（故障ではなく保存停止状態）
 * - Flash初期化失敗    : 100ms点灯、100ms消灯を継続（復旧不能な故障表示）
 *
 * LED_MAPの表示優先順位
 * 1. Flash初期化失敗
 * 2. LOGファイル満杯
 * 3. MAP/LOG書き込み失敗
 * 4. MAP/LOG保存成功
 *
 * DEBUGモードではSerialを使用するため、両LEDを常に消灯する。
 */

bool isPulseOn(uint32_t phase_ms, uint32_t start_ms, uint32_t duration_ms)
{
    return phase_ms >= start_ms && phase_ms < start_ms + duration_ms;
}

bool stateLedOn(SystemState state, uint32_t now_ms)
{
    switch (state) {
        case SystemState::STATE_PRELAUNCH:
            return isPulseOn(now_ms % 2000U, 0, 100);

        case SystemState::STATE_AWAIT_ASCENT: {
            const uint32_t phase = now_ms % 2000U;
            return isPulseOn(phase, 0, 100) || isPulseOn(phase, 200, 100);
        }

        case SystemState::STATE_ASCENT_TO_LANDING:
            return (now_ms % 500U) < 250U;

        case SystemState::STATE_SEPARATION: {
            const uint32_t phase = now_ms % 2000U;
            return isPulseOn(phase, 0, 100) ||
                isPulseOn(phase, 200, 100) ||
                isPulseOn(phase, 400, 100);
        }

        case SystemState::STATE_GPS_NAV:
            return (now_ms % 2000U) < 1000U;

        case SystemState::STATE_CAMERA_NAV:
            return (now_ms % 1000U) < 100U;

        case SystemState::STATE_ESCAPE:
            return (now_ms % 200U) < 100U;

        case SystemState::STATE_UPRIGHT_RECOVERY:
            return (now_ms % 100U) < 50U;

        case SystemState::STATE_GOAL:
            return true;
    }
    return false;
}

bool mapLedOn(const FlashStatus& flash_status, uint32_t now_ms)
{
    if (!flash_status.initialized ||
        flash_status.last_event == FlashLedEvent::InitError) {
        // Flash初期化失敗は100ms間隔の連続高速点滅。
        return (now_ms % 200U) < 100U;
    }

    if (flash_status.storage_full ||
        flash_status.last_event == FlashLedEvent::LogFileFull) {
        // 現在のログファイル満杯は500ms点灯、500ms消灯。
        return (now_ms % 1000U) < 500U;
    }

    const uint32_t age_ms = static_cast<uint32_t>(
        now_ms - flash_status.event_timestamp_ms);
    switch (flash_status.last_event) {
        case FlashLedEvent::LogSaved:
            return age_ms < 80U;

        case FlashLedEvent::MapSaved:
            return isPulseOn(age_ms, 0, 100) ||
                isPulseOn(age_ms, 200, 100);

        case FlashLedEvent::WriteError:
            return isPulseOn(age_ms, 0, 100) ||
                isPulseOn(age_ms, 200, 100) ||
                isPulseOn(age_ms, 400, 100);

        case FlashLedEvent::None:
        case FlashLedEvent::LogFileFull:
        case FlashLedEvent::InitError:
            return false;
    }
    return false;
}

} // namespace

void taskLed(void *pvParameters)
{
    (void)pvParameters;

    pinMode(LED_STATE, OUTPUT);
    pinMode(LED_MAP, OUTPUT);
    digitalWrite(LED_STATE, LOW);
    digitalWrite(LED_MAP, LOW);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(LED_TASK_PERIOD_MS);

    while (true) {
        xTaskDelayUntil(&last_wake, period);

        SystemData system_data{};
        FlashStatus flash_status{};
        const bool has_system =
            xQueuePeek(mbx_system_data, &system_data, 0) == pdTRUE;
        const bool has_flash =
            xQueuePeek(mbx_flash_status, &flash_status, 0) == pdTRUE;

        // DEBUGではSerialだけを使うため、両方のLEDを必ず消灯する。
        const bool debug_mode = has_system &&
            system_data.boot_mode == BootMode::DEBUG;
        if (debug_mode) {
            digitalWrite(LED_STATE, LOW);
            digitalWrite(LED_MAP, LOW);
            continue;
        }

        const uint32_t now_ms = millis();
        digitalWrite(
            LED_STATE,
            has_system && stateLedOn(system_data.state, now_ms) ? HIGH : LOW);
        digitalWrite(
            LED_MAP,
            has_flash && mapLedOn(flash_status, now_ms) ? HIGH : LOW);
    }
}
