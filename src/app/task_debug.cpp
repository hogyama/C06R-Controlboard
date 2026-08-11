#include "tasks.h"
#include "app_types.h"
#include "app_queue.h"
#include "service/Can/srv_can.h"

#include <Arduino.h>
#include <esp_rom_crc.h>
#include <stdarg.h>
#include <string.h>

namespace {

constexpr uint32_t DEBUG_TASK_PERIOD_MS = 20;
constexpr uint32_t ERASE_CONFIRM_TIMEOUT_MS = 5000;

enum class ConfirmTarget : uint8_t {
    None,
    File,
    AllFiles,
    AllMaps
};

struct DebugState {
    int8_t selected_file = -1;
    uint32_t next_log_index = 0;
    uint32_t last_log_index = 0;
    bool has_last_log = false;
    bool waiting_file_digit = false;

    bool request_pending = false;
    bool pending_log_advances = false;
    uint32_t request_id = 0;

    ConfirmTarget confirm_target = ConfirmTarget::None;
    uint32_t confirm_started_ms = 0;
    uint32_t gyro_bias_generation = 0;
    uint32_t magnetic_calibration_generation = 0;
    uint32_t magnetic_reset_generation = 0;
};

const char* magneticCalibrationResultName(uint8_t result)
{
    switch (result) {
        case 0: return "NONE";
        case 1: return "COLLECTING";
        case 2: return "SUCCESS";
        case 3: return "INSUFFICIENT_COVERAGE";
        case 4: return "SINGULAR";
        case 5: return "INVALID_FIT";
        default: return "UNKNOWN";
    }
}

const char* healthName(Domain::Localization::SensorHealth health)
{
    using Domain::Localization::SensorHealth;
    switch (health) {
        case SensorHealth::Disabled: return "DISABLED";
        case SensorHealth::Fresh: return "HEALTHY";
        case SensorHealth::Stale: return "DEGRADED";
        case SensorHealth::Failed: return "FAILED";
    }
    return "UNKNOWN";
}

const char* sourceName(Sensor::Source source)
{
    switch (source) {
        case Sensor::Source::BoardI2c: return "BOARD";
        case Sensor::Source::Can: return "CAN";
        case Sensor::Source::Gps: return "GPS";
        case Sensor::Source::None: return "NONE";
    }
    return "NONE";
}

int8_t decodeFileIndex(char command)
{
    if (command >= '0' && command <= '2') return command - '0';
    return -1;
}

const char* resultName(FlashDebugResult result)
{
    switch (result) {
        case FlashDebugResult::Ok: return "OK";
        case FlashDebugResult::NotReady: return "NOT_READY";
        case FlashDebugResult::NotFound: return "NOT_FOUND";
        case FlashDebugResult::EndOfFile: return "END_OF_FILE";
        case FlashDebugResult::InvalidArgument: return "INVALID_ARGUMENT";
        case FlashDebugResult::OriginMismatch: return "ORIGIN_MISMATCH";
        case FlashDebugResult::Corrupt: return "CORRUPT";
        case FlashDebugResult::ReadError: return "READ_ERROR";
        case FlashDebugResult::EraseError: return "ERASE_ERROR";
    }
    return "UNKNOWN";
}

bool serialConnected()
{
    return static_cast<bool>(Serial);
}

void serialWriteBytes(const char* data, size_t length)
{
    if (!serialConnected() || data == nullptr) return;

    size_t offset = 0;
    while (offset < length && serialConnected()) {
        // ESP32-S3 USB CDCの1 packetに合わせて最大64 byteずつ送る。
        const size_t chunk = min<size_t>(length - offset, 64U);
        const size_t written = Serial.write(
            reinterpret_cast<const uint8_t*>(data + offset),
            chunk);
        offset += written;

        // USB hostが直前のpacketを回収する時間を確保する。
        delay(1);
    }
}

void serialWriteLine(const char* line)
{
    if (line == nullptr) return;

    // 本文と改行を同じbufferへまとめ、別packetとの行連結を防ぐ。
    char framed_line[514]{};
    const size_t text_length = strnlen(line, sizeof(framed_line) - 3);
    memcpy(framed_line, line, text_length);
    framed_line[text_length] = '\r';
    framed_line[text_length + 1] = '\n';
    serialWriteBytes(framed_line, text_length + 2);
}

void serialPrintfLine(const char* format, ...)
{
    char line[512]{};
    va_list args;
    va_start(args, format);
    vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    serialWriteLine(line);
}

void printHelp()
{
    serialWriteLine("$HELP,?=help,v=status,h=sensor_health,b=gyro_bias_cal,k=mag_cal_start,c=mag_cal_status,C=mag_cal_reset,f=list,i+0-2=select,n=next,r=repeat,m=map,x+x=erase_file,X+X=erase_all_files,M+M=erase_maps,R=can_reset,s=can_start");
}

void printSensorStatus()
{
    LocalizationDebugStatus status{};
    if (xQueuePeek(mbx_localization_debug_status, &status, 0) != pdTRUE) {
        serialWriteLine("$ERROR,LOCALIZATION_STATUS_UNAVAILABLE");
        return;
    }
    serialPrintfLine(
        "$SENSOR,GYRO,active=%s,board=%s,board_hz=%.1f,can=%s,can_hz=%.1f",
        sourceName(status.gyroscope.active_source),
        healthName(status.gyroscope.board_health),
        status.gyroscope.board_rate_hz,
        healthName(status.gyroscope.can_health),
        status.gyroscope.can_rate_hz);
    serialPrintfLine(
        "$SENSOR,ACCEL,active=%s,board=%s,board_hz=%.1f,can=%s,can_hz=%.1f",
        sourceName(status.accelerometer.active_source),
        healthName(status.accelerometer.board_health),
        status.accelerometer.board_rate_hz,
        healthName(status.accelerometer.can_health),
        status.accelerometer.can_rate_hz);
    serialPrintfLine(
        "$SENSOR,MAG,active=%s,board=%s,board_hz=%.1f,can=%s,can_hz=%.1f",
        sourceName(status.magnetic.active_source),
        healthName(status.magnetic.board_health),
        status.magnetic.board_rate_hz,
        healthName(status.magnetic.can_health),
        status.magnetic.can_rate_hz);
    serialPrintfLine(
        "$SENSOR,ENCODER,active=CAN,health=%s,rate_hz=%.1f",
        healthName(status.encoder_health), status.encoder_rate_hz);
    serialPrintfLine(
        "$SENSOR,GNSS,active=GPS,health=%s,rate_hz=%.1f",
        healthName(status.gps_health), status.gps_rate_hz);
    serialPrintfLine(
        "$SENSOR,PRESSURE,active=CAN,health=%s,rate_hz=%.1f",
        healthName(status.pressure_health), status.pressure_rate_hz);
}

void printMagneticCalibration()
{
    LocalizationDebugStatus status{};
    if (xQueuePeek(mbx_localization_debug_status, &status, 0) != pdTRUE) {
        serialWriteLine("$ERROR,LOCALIZATION_STATUS_UNAVAILABLE");
        return;
    }
    serialPrintfLine(
        "$MAG_CAL,source=BOARD,valid=%u,active=%u,result=%s,samples=%lu/%u,rms_uT=%.3f,offset_x=%.3f,offset_y=%.3f,offset_z=%.3f",
        status.magnetic_calibration_valid ? 1U : 0U,
        status.magnetic_calibrating ? 1U : 0U,
        magneticCalibrationResultName(status.magnetic_calibration_result),
        static_cast<unsigned long>(status.magnetic_calibration_samples),
        static_cast<unsigned>(status.magnetic_calibration_target_samples),
        status.magnetic_calibration_rms_uT,
        status.magnetic_hard_iron_uT[0],
        status.magnetic_hard_iron_uT[1],
        status.magnetic_hard_iron_uT[2]);
    for (uint8_t row = 0; row < 3; ++row) {
        serialPrintfLine("$MAG_CAL,M%u=%.6f,%.6f,%.6f",
            static_cast<unsigned>(row),
            status.magnetic_soft_iron[row][0],
            status.magnetic_soft_iron[row][1],
            status.magnetic_soft_iron[row][2]);
    }
}

bool sendLocalizationCommand(LocalizationDebugCommand command)
{
    if (xQueueSend(fifo_localization_debug_command, &command, 0) == pdTRUE) {
        return true;
    }
    serialWriteLine("$ERROR,LOCALIZATION_COMMAND_QUEUE_FULL");
    return false;
}

void handleLocalizationStatus(DebugState& state)
{
    LocalizationDebugStatus status{};
    if (xQueuePeek(mbx_localization_debug_status, &status, 0) != pdTRUE) return;
    if (status.gyro_bias_generation != state.gyro_bias_generation) {
        state.gyro_bias_generation = status.gyro_bias_generation;
        if (!status.gyro_bias_last_success) {
            serialPrintfLine(
                "$GYRO_BIAS,ERROR,INSUFFICIENT_SAMPLES,samples=%lu",
                static_cast<unsigned long>(status.gyro_bias_samples));
        } else {
            serialPrintfLine(
                "$GYRO_BIAS,OK,z_rad_s=%.7f,std_rad_s=%.7f,max_dev_rad_s=%.7f,samples=%lu",
                status.gyro_bias_z_rad_s,
                status.gyro_bias_std_rad_s,
                status.gyro_bias_max_deviation_rad_s,
                static_cast<unsigned long>(status.gyro_bias_samples));
            if (status.gyro_bias_std_rad_s > 0.02f) {
                serialWriteLine("$GYRO_BIAS,WARNING,MOTION_DETECTED");
            }
        }
    }
    if (status.magnetic_calibration_generation !=
        state.magnetic_calibration_generation) {
        state.magnetic_calibration_generation =
            status.magnetic_calibration_generation;
        if (status.magnetic_calibration_last_success) {
            serialPrintfLine("$MAG_CAL,OK,samples=%lu,rms_uT=%.3f",
                static_cast<unsigned long>(
                    status.magnetic_calibration_samples),
                status.magnetic_calibration_rms_uT);
        } else {
            serialPrintfLine("$MAG_CAL,ERROR,%s,samples=%lu",
                magneticCalibrationResultName(
                    status.magnetic_calibration_result),
                static_cast<unsigned long>(
                    status.magnetic_calibration_samples));
        }
    }
    if (status.magnetic_reset_generation !=
        state.magnetic_reset_generation) {
        state.magnetic_reset_generation = status.magnetic_reset_generation;
        serialWriteLine("$MAG_CAL,RESET,OK");
    }
}

bool sendFlashRequest(
    DebugState& state,
    FlashDebugRequestType type,
    uint8_t file_index = 0,
    uint32_t log_index = 0,
    bool advance_log = false)
{
    if (state.request_pending) {
        serialWriteLine("$BUSY");
        return false;
    }

    FlashDebugRequest request{};
    request.type = type;
    request.response_target = FlashResponseTarget::Debug;
    request.file_index = file_index;
    request.log_index = log_index;
    request.request_id = ++state.request_id;
    if (request.request_id == 0) request.request_id = ++state.request_id;

    if (xQueueSend(fifo_flash_debug_request, &request, 0) != pdTRUE) {
        serialWriteLine("$ERROR,FLASH_REQUEST_QUEUE_FULL");
        return false;
    }

    state.request_pending = true;
    state.pending_log_advances = advance_log;
    return true;
}

void printHexData(const FlashDebugResponse& response)
{
    const char* kind = response.type == FlashDebugRequestType::ReadLog
        ? "LOG"
        : "MAP";
    const uint32_t crc = esp_rom_crc32_le(0, response.data, response.data_size);

    serialPrintfLine(
        "$BEGIN,type=%s,file=%X,index=%lu,size=%u",
        kind,
        response.file_index,
        static_cast<unsigned long>(response.log_index),
        response.data_size);

    // 1行16 byte固定にし、PC側で欠落行とoffsetを検出しやすくする。
    for (uint16_t offset = 0; offset < response.data_size; offset += 16) {
        const uint16_t length = min<uint16_t>(16, response.data_size - offset);
        char line[64]{};
        int position = snprintf(line, sizeof(line), "$DATA,offset=%04X,", offset);
        for (uint16_t i = 0; i < length; ++i) {
            position += snprintf(
                line + position,
                sizeof(line) - static_cast<size_t>(position),
                "%02X",
                response.data[offset + i]);
        }
        serialWriteLine(line);
    }
    serialPrintfLine("$END,crc32=%08lX", static_cast<unsigned long>(crc));
}

void handleFlashResponse(DebugState& state)
{
    static FlashDebugResponse response{};
    if (xQueueReceive(mbx_flash_debug_response, &response, 0) != pdTRUE) return;
    if (!state.request_pending || response.request_id != state.request_id) return;

    state.request_pending = false;
    if (response.result != FlashDebugResult::Ok) {
        serialPrintfLine("$ERROR,type=%u,result=%s,file=%X,index=%lu",
            static_cast<unsigned>(response.type),
            resultName(response.result),
            response.file_index,
            static_cast<unsigned long>(response.log_index));
        return;
    }

    switch (response.type) {
        case FlashDebugRequestType::ListFiles:
            // 既存PCツールとの互換性を保ち、active fileは別行にする。
            serialPrintfLine("$FILES,used=%X", response.used_file_flags);
            serialPrintfLine(
                "$ACTIVE,index=%d",
                response.file_index < MAX_FILES
                    ? static_cast<int>(response.file_index)
                    : -1);
            for (uint8_t i = 0; i < MAX_FILES; ++i) {
                serialPrintfLine("$FILE,index=%X,used=%u", i,
                    (response.used_file_flags & (1U << i)) != 0 ? 1 : 0);
            }
            break;

        case FlashDebugRequestType::ReadLog:
            printHexData(response);
            state.last_log_index = response.log_index;
            state.has_last_log = true;
            if (state.pending_log_advances) {
                state.next_log_index = response.log_index + 1;
            }
            break;

        case FlashDebugRequestType::ReadLatestMap:
            printHexData(response);
            break;

        case FlashDebugRequestType::EraseFile:
            serialPrintfLine("$ERASE_FILE,index=%X,result=OK", response.file_index);
            if (state.selected_file == static_cast<int8_t>(response.file_index)) {
                state.next_log_index = 0;
                state.has_last_log = false;
            }
            break;

        case FlashDebugRequestType::EraseAllFiles:
            serialWriteLine("$ERASE_FILES,result=OK");
            state.selected_file = -1;
            state.next_log_index = 0;
            state.has_last_log = false;
            break;

        case FlashDebugRequestType::EraseAllMaps:
            serialWriteLine("$ERASE_MAPS,result=OK");
            break;

        default:
            serialWriteLine("$OK");
            break;
    }
}

void sendCanAction(Can::Command::ActionType type, const char* label)
{
    const char command = static_cast<char>(type);
    if (xQueueSend(fifo_can_char_cmd, &command, 0) == pdTRUE) {
        serialPrintfLine("$CAN,%s,QUEUED", label);
    } else {
        serialWriteLine("$ERROR,CAN_QUEUE_FULL");
    }
}

void printState()
{
    SystemData system{};
    FlashStatus flash_status{};
    const bool has_system =
        xQueuePeek(mbx_system_data, &system, 0) == pdTRUE;
    const bool has_flash =
        xQueuePeek(mbx_flash_status, &flash_status, 0) == pdTRUE;

    serialPrintfLine(
        "$STATE,state=%u,boot=%u,flash_ok=%u,active=%d,used=%X,full=%u",
        has_system ? static_cast<unsigned>(system.state) : 0U,
        has_system ? static_cast<unsigned>(system.boot_mode) : 0U,
        has_flash && flash_status.initialized ? 1U : 0U,
        has_flash ? static_cast<int>(flash_status.active_file_index) : -1,
        has_flash ? flash_status.used_file_flags : 0U,
        has_flash && flash_status.storage_full ? 1U : 0U);
}

void handleCommand(char command, DebugState& state, uint32_t now_ms)
{
    if (command == '\r' || command == '\n' || command == ' ') return;

    // ファイル番号待ちの途中でもvで状態を1回確認できる。
    if (command == 'v') {
        state.waiting_file_digit = false;
        state.confirm_target = ConfirmTarget::None;
        printState();
        return;
    }

    if (state.waiting_file_digit) {
        state.waiting_file_digit = false;
        const int8_t file = decodeFileIndex(command);
        if (file < 0 || file >= MAX_FILES) {
            serialWriteLine("$ERROR,FILE_INDEX_EXPECTED_0_TO_2");
            return;
        }
        state.selected_file = file;
        state.next_log_index = 0;
        state.has_last_log = false;
        serialPrintfLine("$SELECT_FILE,index=%X", file);
        return;
    }

    if (state.confirm_target != ConfirmTarget::None &&
        static_cast<uint32_t>(now_ms - state.confirm_started_ms) >
            ERASE_CONFIRM_TIMEOUT_MS) {
        state.confirm_target = ConfirmTarget::None;
        serialWriteLine("$CONFIRM,TIMEOUT");
    }

    switch (command) {
        case '?':
            state.confirm_target = ConfirmTarget::None;
            printHelp();
            break;

        case 'h':
            state.confirm_target = ConfirmTarget::None;
            printSensorStatus();
            break;

        case 'b': {
            state.confirm_target = ConfirmTarget::None;
            const LocalizationDebugCommand request =
                LocalizationDebugCommand::CalibrateGyroBias;
            if (sendLocalizationCommand(request)) {
                serialWriteLine("$GYRO_BIAS,CALIBRATING,forced=1");
            }
            break;
        }

        case 'k': {
            state.confirm_target = ConfirmTarget::None;
            const LocalizationDebugCommand request =
                LocalizationDebugCommand::CalibrateMagnetic;
            if (sendLocalizationCommand(request)) {
                serialWriteLine(
                    "$MAG_CAL,COLLECTING,source=BOARD,samples=1200,rotate_all_axes=1");
            }
            break;
        }

        case 'c':
            state.confirm_target = ConfirmTarget::None;
            printMagneticCalibration();
            break;

        case 'C': {
            state.confirm_target = ConfirmTarget::None;
            const LocalizationDebugCommand request =
                LocalizationDebugCommand::ResetMagneticCalibration;
            if (sendLocalizationCommand(request)) {
                serialWriteLine("$MAG_CAL,RESET,QUEUED");
            }
            break;
        }

        case 'f':
            state.confirm_target = ConfirmTarget::None;
            sendFlashRequest(state, FlashDebugRequestType::ListFiles);
            break;

        case 'i':
            state.confirm_target = ConfirmTarget::None;
            state.waiting_file_digit = true;
            serialWriteLine("$INPUT,FILE_INDEX_0_TO_2");
            break;

        case 'n':
            state.confirm_target = ConfirmTarget::None;
            if (state.selected_file < 0) {
                serialWriteLine("$ERROR,SELECT_FILE_FIRST");
            } else {
                sendFlashRequest(state, FlashDebugRequestType::ReadLog,
                    state.selected_file, state.next_log_index, true);
            }
            break;

        case 'r':
            state.confirm_target = ConfirmTarget::None;
            if (state.selected_file < 0 || !state.has_last_log) {
                serialWriteLine("$ERROR,NO_LOG_TO_REPEAT");
            } else {
                sendFlashRequest(state, FlashDebugRequestType::ReadLog,
                    state.selected_file, state.last_log_index, false);
            }
            break;

        case 'm':
            state.confirm_target = ConfirmTarget::None;
            sendFlashRequest(state, FlashDebugRequestType::ReadLatestMap);
            break;

        case 'x':
            if (state.selected_file < 0) {
                serialWriteLine("$ERROR,SELECT_FILE_FIRST");
            } else if (state.confirm_target == ConfirmTarget::File) {
                state.confirm_target = ConfirmTarget::None;
                sendFlashRequest(state, FlashDebugRequestType::EraseFile,
                    state.selected_file);
            } else {
                state.confirm_target = ConfirmTarget::File;
                state.confirm_started_ms = now_ms;
                serialPrintfLine("$CONFIRM,press=x,erase_file=%X,timeout_ms=%lu",
                    state.selected_file,
                    static_cast<unsigned long>(ERASE_CONFIRM_TIMEOUT_MS));
            }
            break;

        case 'X':
            if (state.confirm_target == ConfirmTarget::AllFiles) {
                state.confirm_target = ConfirmTarget::None;
                sendFlashRequest(state, FlashDebugRequestType::EraseAllFiles);
            } else {
                state.confirm_target = ConfirmTarget::AllFiles;
                state.confirm_started_ms = now_ms;
                serialPrintfLine("$CONFIRM,press=X,erase_all_files=1,timeout_ms=%lu",
                    static_cast<unsigned long>(ERASE_CONFIRM_TIMEOUT_MS));
            }
            break;

        case 'M':
            if (state.confirm_target == ConfirmTarget::AllMaps) {
                state.confirm_target = ConfirmTarget::None;
                sendFlashRequest(state, FlashDebugRequestType::EraseAllMaps);
            } else {
                state.confirm_target = ConfirmTarget::AllMaps;
                state.confirm_started_ms = now_ms;
                serialPrintfLine("$CONFIRM,press=M,erase_all_maps=1,timeout_ms=%lu",
                    static_cast<unsigned long>(ERASE_CONFIRM_TIMEOUT_MS));
            }
            break;

        case 'R':
            state.confirm_target = ConfirmTarget::None;
            sendCanAction(Can::Command::Reset, "RESET");
            break;

        case 's':
            state.confirm_target = ConfirmTarget::None;
            sendCanAction(Can::Command::SequenceStart, "SEQUENCE_START");
            break;

        default:
            state.confirm_target = ConfirmTarget::None;
            serialPrintfLine("$ERROR,UNKNOWN_COMMAND=%02X",
                static_cast<unsigned>(static_cast<uint8_t>(command)));
            break;
    }
}

} // namespace

void taskDebug(void *pvParameters)
{
    (void)pvParameters;
    Serial.begin(115200);

    DebugState state{};
    bool debug_active = false;
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(DEBUG_TASK_PERIOD_MS);

    while (true) {
        xTaskDelayUntil(&last_wake, period);

        SystemData system_data{};
        const bool is_debug =
            xQueuePeek(mbx_system_data, &system_data, 0) == pdTRUE &&
            system_data.boot_mode == BootMode::DEBUG;
        if (!is_debug) {
            debug_active = false;
            continue;
        }

        if (!debug_active) {
            debug_active = true;
            state = DebugState{};
            serialWriteLine("$DEBUG,READY");
            printHelp();
        }

        handleFlashResponse(state);
        handleLocalizationStatus(state);

        while (Serial.available() > 0) {
            handleCommand(static_cast<char>(Serial.read()), state, millis());
        }

    }
}
