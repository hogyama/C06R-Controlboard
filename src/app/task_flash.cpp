#include "tasks.h"
#include "platform/pin_config.h"
#include "app_types.h"
#include "app_queue.h"
#include "app_context.h"
#include "algorithm/astar.h"

#include <string.h>

namespace {

// ログは全フェーズ10 Hz。状態遷移とスタックはtask周期内で優先保存する。
constexpr uint32_t FLASH_TASK_PERIOD_MS = 20;
constexpr uint32_t LOG_SAVE_PERIOD_MS = 100;
constexpr uint32_t MAP_SAVE_QUIET_MS = 10000;
constexpr uint32_t MAP_SAVE_MIN_INTERVAL_MS = 30000;
constexpr uint32_t MAP_SAVE_MAX_DELAY_MS = 120000;

FlashDebugResult convertMapResult(SrvFlash::MapLoadResult result)
{
    switch (result) {
        case SrvFlash::MapLoadResult::Valid: return FlashDebugResult::Ok;
        case SrvFlash::MapLoadResult::NotFound: return FlashDebugResult::NotFound;
        case SrvFlash::MapLoadResult::OriginMismatch: return FlashDebugResult::OriginMismatch;
        case SrvFlash::MapLoadResult::Corrupt: return FlashDebugResult::Corrupt;
        case SrvFlash::MapLoadResult::ReadError: return FlashDebugResult::ReadError;
        case SrvFlash::MapLoadResult::InvalidArgument: return FlashDebugResult::InvalidArgument;
    }
    return FlashDebugResult::ReadError;
}

FlashDebugResult convertLogResult(SrvFlash::LogReadResult result)
{
    switch (result) {
        case SrvFlash::LogReadResult::Valid: return FlashDebugResult::Ok;
        case SrvFlash::LogReadResult::EndOfFile: return FlashDebugResult::EndOfFile;
        case SrvFlash::LogReadResult::Corrupt: return FlashDebugResult::Corrupt;
        case SrvFlash::LogReadResult::ReadError: return FlashDebugResult::ReadError;
        case SrvFlash::LogReadResult::InvalidArgument: return FlashDebugResult::InvalidArgument;
    }
    return FlashDebugResult::ReadError;
}

void publishFlashStatus(
    bool initialized,
    FlashLedEvent last_event,
    uint32_t event_timestamp_ms)
{
    FlashStatus status{};
    status.initialized = initialized;
    status.active_file_index = initialized ? flash.getActiveFileIndex() : -1;
    status.used_file_flags = initialized ? flash.getUsedFileBitFlag() : 0;
    status.storage_full = initialized && flash.isStorageFull();
    status.last_event = last_event;
    status.event_timestamp_ms = event_timestamp_ms;
    status.timestamp_ms = millis();
    xQueueOverwrite(mbx_flash_status, &status);
}

void processFlashRequest(
    const FlashDebugRequest& request,
    bool initialized,
    bool allow_destructive_operations)
{
    // 2KBの応答バッファをtask stackへ毎回積まない。
    static FlashDebugResponse response{};
    memset(&response, 0, sizeof(response));
    response.type = request.type;
    response.result = initialized
        ? FlashDebugResult::InvalidArgument
        : FlashDebugResult::NotReady;
    response.request_id = request.request_id;
    response.file_index = request.file_index;
    response.log_index = request.log_index;

    if (initialized) {
        switch (request.type) {
            case FlashDebugRequestType::ListFiles:
                response.result = FlashDebugResult::Ok;
                response.file_index =
                    static_cast<uint8_t>(flash.getActiveFileIndex());
                response.used_file_flags = flash.getUsedFileBitFlag();
                break;

            case FlashDebugRequestType::ReadLog: {
                Flash::LogFrame log{};
                response.result = convertLogResult(
                    flash.loadLog(request.file_index, request.log_index, log));
                if (response.result == FlashDebugResult::Ok) {
                    static_assert(sizeof(log) <= AStar::MAP_BYTES,
                        "LogFrame is too large for FlashDebugResponse");
                    memcpy(response.data, &log, sizeof(log));
                    response.data_size = sizeof(log);
                }
                break;
            }

            case FlashDebugRequestType::ReadLatestMap:
                response.result = convertMapResult(flash.loadLatestMap(response.data));
                if (response.result == FlashDebugResult::Ok) {
                    response.data_size = AStar::MAP_BYTES;
                }
                break;

            case FlashDebugRequestType::EraseFile:
                if (!allow_destructive_operations) {
                    response.result = FlashDebugResult::InvalidArgument;
                } else if (request.file_index >= MAX_FILES) {
                    response.result = FlashDebugResult::InvalidArgument;
                } else {
                    response.result = flash.eraseFILE(request.file_index)
                        ? FlashDebugResult::Ok
                        : FlashDebugResult::EraseError;
                }
                break;

            case FlashDebugRequestType::EraseAllFiles:
                if (!allow_destructive_operations) {
                    response.result = FlashDebugResult::InvalidArgument;
                } else {
                    response.result = flash.resetAllFiles()
                        ? FlashDebugResult::Ok
                        : FlashDebugResult::EraseError;
                }
                break;

            case FlashDebugRequestType::EraseAllMaps:
                if (!allow_destructive_operations) {
                    response.result = FlashDebugResult::InvalidArgument;
                } else {
                    response.result = flash.resetAllMaps()
                        ? FlashDebugResult::Ok
                        : FlashDebugResult::EraseError;
                }
                break;

            case FlashDebugRequestType::None:
            default:
                response.result = FlashDebugResult::InvalidArgument;
                break;
        }
        response.used_file_flags = flash.getUsedFileBitFlag();
    }

    // 要求元専用mailboxへ返し、別タスクが応答を先取りしないようにする。
    QueueHandle_t response_queue =
        request.response_target == FlashResponseTarget::Navigation
            ? mbx_flash_nav_response
            : mbx_flash_debug_response;
    xQueueOverwrite(response_queue, &response);
    publishFlashStatus(
        initialized,
        initialized ? FlashLedEvent::None : FlashLedEvent::InitError,
        millis());
}

} // namespace

void taskFlash(void *pvParameters)
{
    (void)pvParameters;

    const bool flash_initialized = flash.init(
        FLASH_SCLK,
        FLASH_MISO,
        FLASH_MOSI,
        FLASH_CS
    );
    FlashLedEvent led_event = flash_initialized
        ? FlashLedEvent::None
        : FlashLedEvent::InitError;
    uint32_t led_event_ms = millis();
    publishFlashStatus(flash_initialized, led_event, led_event_ms);

    uint8_t map_snapshot[AStar::MAP_BYTES]{};
    uint32_t saved_map_update_count = 0;
    uint32_t observed_map_update_count = 0;
    uint32_t last_map_change_ms = millis();
    uint32_t last_map_save_ms = millis();
    SystemState previous_state = SystemState::STATE_PRELAUNCH;
    bool sequence_started = false;
    uint32_t last_log_save_ms = 0;
    Flash::LogFrame last_saved_log{};
    bool have_saved_log = false;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(FLASH_TASK_PERIOD_MS);

    while (true) {
        xTaskDelayUntil(&last_wake, period);
        const uint32_t now_ms = millis();

        SystemData status{};
        if (xQueuePeek(mbx_system_data, &status, 0) != pdTRUE) {
            continue;
        }

        // 起動時のMAP読込要求は通常モードでも受け付ける。削除はDEBUGだけに限定する。
        if (status.boot_mode != BootMode::DEBUG) {
            FlashDebugRequest request{};
            while (xQueueReceive(fifo_flash_debug_request, &request, 0) == pdTRUE) {
                processFlashRequest(request, flash_initialized, false);
            }
        }

        // DEBUGでは読み出し・削除要求だけを扱い、ログと地図は保存しない。
        if (status.boot_mode == BootMode::DEBUG) {
            FlashDebugRequest request{};
            while (xQueueReceive(fifo_flash_debug_request, &request, 0) == pdTRUE) {
                processFlashRequest(request, flash_initialized, true);
            }
            publishFlashStatus(flash_initialized, led_event, led_event_ms);
            previous_state = status.state;
            continue;
        }

        // シーケンス開始時に保存領域を切り替える。
        const bool entered_sequence =
            previous_state == SystemState::STATE_PRELAUNCH &&
            status.state == SystemState::STATE_AWAIT_ASCENT;
        const bool entered_navigation =
            status.state == SystemState::STATE_GPS_NAV ||
            status.state == SystemState::STATE_CAMERA_NAV;

        if (flash_initialized && !sequence_started &&
            (entered_sequence || entered_navigation)) {
            sequence_started = flash.startNewSequence(map_snapshot);
            if (!sequence_started) {
                led_event = flash.isStorageFull()
                    ? FlashLedEvent::LogFileFull
                    : FlashLedEvent::WriteError;
                led_event_ms = now_ms;
            } else {
                uint32_t initial_map_update_count = 0;

                // シーケンス開始時点のRAM地図を必ず1回保存する。
                // 障害物更新がまだ無いUNKNOWN地図でも、DEBUGのmで確認できるようにする。
                if (xSemaphoreTake(mutex_grid_map, portMAX_DELAY) == pdTRUE) {
                    memcpy(map_snapshot, grid_map, AStar::MAP_BYTES);
                    initial_map_update_count = grid_map_update_count;
                    xSemaphoreGive(mutex_grid_map);
                }

                if (flash.saveMap(map_snapshot)) {
                    saved_map_update_count = initial_map_update_count;
                    observed_map_update_count = initial_map_update_count;
                    last_map_change_ms = now_ms;
                    last_map_save_ms = now_ms;
                    led_event = FlashLedEvent::MapSaved;
                    led_event_ms = now_ms;
                } else {
                    // 次の通常保存条件で再試行できるよう、未保存として扱う。
                    saved_map_update_count = initial_map_update_count - 1U;
                    observed_map_update_count = initial_map_update_count;
                    last_map_change_ms = now_ms;
                    last_map_save_ms = now_ms;
                    led_event = FlashLedEvent::WriteError;
                    led_event_ms = now_ms;
                }
            }
            last_log_save_ms = now_ms - LOG_SAVE_PERIOD_MS;
        }

        // 全状態100 ms。状態遷移とスタックイベントは周期を待たず保存する。
        Flash::LogFrame latest_log{};
        if (flash_initialized && sequence_started && !flash.isStorageFull() &&
            xQueuePeek(mbx_flash_log, &latest_log, 0) == pdTRUE) {
            const bool new_log =
                !have_saved_log ||
                latest_log.message_number != last_saved_log.message_number;
            const bool state_event =
                have_saved_log &&
                latest_log.mission_state != last_saved_log.mission_state;
            const bool stuck_event =
                latest_log.stuck_reason != 0 &&
                (!have_saved_log ||
                 latest_log.stuck_reason != last_saved_log.stuck_reason ||
                 latest_log.stuck_cell_x != last_saved_log.stuck_cell_x ||
                 latest_log.stuck_cell_y != last_saved_log.stuck_cell_y);
            const bool periodic_due =
                static_cast<uint32_t>(now_ms - last_log_save_ms) >=
                    LOG_SAVE_PERIOD_MS;

            if (new_log && (state_event || stuck_event || periodic_due)) {
                if (flash.saveLog(latest_log)) {
                    last_saved_log = latest_log;
                    have_saved_log = true;
                    last_log_save_ms = now_ms;

                    if (flash.isStorageFull()) {
                        // 20480件目を保存した後は、このシーケンス中のLOG保存を止める。
                        led_event = FlashLedEvent::LogFileFull;
                        led_event_ms = now_ms;
                    } else {
                        const uint32_t event_age_ms =
                            static_cast<uint32_t>(now_ms - led_event_ms);
                        const bool higher_priority_event_active =
                            (led_event == FlashLedEvent::MapSaved && event_age_ms < 400U) ||
                            (led_event == FlashLedEvent::WriteError && event_age_ms < 600U);
                        if (!higher_priority_event_active) {
                            led_event = FlashLedEvent::LogSaved;
                            led_event_ms = now_ms;
                        }
                    }
                } else {
                    // 失敗時も現在フェーズの保存周期まで待ち、連続再書き込みを避ける。
                    last_log_save_ms = now_ms;
                    led_event = flash.isStorageFull()
                        ? FlashLedEvent::LogFileFull
                        : FlashLedEvent::WriteError;
                    led_event_ms = now_ms;
                }
            }
        }

        uint32_t current_map_update_count = observed_map_update_count;
        if (xSemaphoreTake(mutex_grid_map, portMAX_DELAY) == pdTRUE) {
            current_map_update_count = grid_map_update_count;
            xSemaphoreGive(mutex_grid_map);
        }

        // 更新番号が変化した時刻を記録し、連続更新をまとめて保存する。
        if (current_map_update_count != observed_map_update_count) {
            observed_map_update_count = current_map_update_count;
            last_map_change_ms = now_ms;
            have_saved_log = false;
            last_log_save_ms = now_ms;
        }

        const bool map_dirty =
            current_map_update_count != saved_map_update_count;
        const bool quiet_elapsed =
            static_cast<uint32_t>(now_ms - last_map_change_ms) >= MAP_SAVE_QUIET_MS;
        const bool minimum_interval_elapsed =
            static_cast<uint32_t>(now_ms - last_map_save_ms) >= MAP_SAVE_MIN_INTERVAL_MS;
        const bool maximum_delay_elapsed =
            static_cast<uint32_t>(now_ms - last_map_save_ms) >= MAP_SAVE_MAX_DELAY_MS;
        const bool entered_goal =
            previous_state != SystemState::STATE_GOAL &&
            status.state == SystemState::STATE_GOAL;

        const bool should_save =
            flash_initialized &&
            sequence_started &&
            map_dirty &&
            (entered_goal ||
             (status.state == SystemState::STATE_GOAL && minimum_interval_elapsed) ||
             (quiet_elapsed && minimum_interval_elapsed) ||
             maximum_delay_elapsed);

        if (should_save) {
            uint32_t snapshot_update_count = saved_map_update_count;

            // mutex保持中はRAMコピーだけを行い、Flash書き込み中は地図を塞がない。
            if (xSemaphoreTake(mutex_grid_map, portMAX_DELAY) == pdTRUE) {
                memcpy(map_snapshot, grid_map, AStar::MAP_BYTES);
                snapshot_update_count = grid_map_update_count;
                xSemaphoreGive(mutex_grid_map);
            }

            if (snapshot_update_count != saved_map_update_count) {
                if (flash.saveMap(map_snapshot)) {
                    saved_map_update_count = snapshot_update_count;
                    last_map_save_ms = now_ms;
                    led_event = FlashLedEvent::MapSaved;
                    led_event_ms = now_ms;
                } else {
                    // 失敗時は最小保存間隔まで待ち、Flashを連続アクセスしない。
                    last_map_save_ms = now_ms;
                    led_event = FlashLedEvent::WriteError;
                    led_event_ms = now_ms;
                }
            }
        }

        // GOALログと最新MAPの両方を実際に保存できてから、以降の保存を停止する。
        // taskLogのsnapshot生成がGOAL遷移より遅い場合も、次周期以降まで待つ。
        const bool goal_log_complete =
            (have_saved_log &&
             last_saved_log.mission_state ==
                static_cast<uint8_t>(SystemState::STATE_GOAL)) ||
            (flash_initialized && flash.isStorageFull());
        bool latest_map_saved = false;
        if (xSemaphoreTake(mutex_grid_map, portMAX_DELAY) == pdTRUE) {
            latest_map_saved =
                saved_map_update_count == grid_map_update_count;
            xSemaphoreGive(mutex_grid_map);
        }
        if (status.state == SystemState::STATE_GOAL &&
            goal_log_complete && latest_map_saved) {
            sequence_started = false;
        }

        // Reset後は次のシーケンス開始時に新しい保存領域を準備する。
        if (status.state == SystemState::STATE_PRELAUNCH &&
            previous_state != SystemState::STATE_PRELAUNCH) {
            sequence_started = false;
            saved_map_update_count = 0;
            observed_map_update_count = 0;
            last_map_change_ms = now_ms;
            led_event = FlashLedEvent::None;
            led_event_ms = now_ms;
        }

        previous_state = status.state;
        publishFlashStatus(flash_initialized, led_event, led_event_ms);
    }
}
