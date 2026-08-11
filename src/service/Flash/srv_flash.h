#pragma once
#include <Arduino.h>
#include <esp_rom_crc.h>
#include "hal_S25FL128L.h"


// S25FL128Lは16MB = 16 * 1024 KB = 16 * 1024 * 1024 

// 1回のMAPは2048B = 2KB(64x64マップ、1セル4bit)
// MAP保存領域のセクタは header 256B x3 + MAP 1024B x3 + padding 256B = 4096B = 4KBとする
// 1回のログは128B

// MAP保存領域は128KB = 4KB * 32セクタより、96回のMAPを保存可能
// FILEは5120KB = 1280 * 4KB = 5MBより、40960個のログを保存可能

// MAP保存領域 x 2 + FILE保存領域 x 3として運用する

// MAP0領域   0x000000 - 0x01FFFF
// MAP1領域   0x020000 - 0x03FFFF

// FILE0領域  0x040000 - 0x53FFFF
// FILE1領域  0x540000 - 0xA3FFFF
// FILE2領域  0xA40000 - 0xF3FFFF

/**
 * @brief S25FL128L ストレージ運用仕様
 *
 * [MAP領域]
 * - ロード/初期化
 *   - シーケンス開始時: 最新MAPを自動ロード後、別領域をクリアし新保存先とする
 *   - 手動リセット    : シーケンス開始前にSerialコマンドで実行
 * - 最新データの判定
 *   1. 領域: 2面のうち `session_id` が大きい方
 *   2. MAP : 選択領域内で `update_count` が最大のもの
 *
 * [FILE領域]
 * - 書き込み条件
 *   - シーケンス開始時にだけ新しいFILEを選び、DEBUGでは保存しない
 *   - 常に最新のFILE領域に追記 
 * - 満杯時の保護 (STORAGE_FULL)
 *   - 既存データの破壊を防ぐため新規保存を停止
 *   - LEDを「2回点滅して休む」パターンで通知
 * - 削除ルール
 *   - DEBUGモードのSerialコマンドから個別FILEまたは全FILEを削除できる
 *   - 通常モードでは削除要求を拒否する
 * 
 * [header変数]
 *  session_idはMAPとFILEで共通のカウンタを使用する
 *  update_countはMAP領域内での更新回数。毎回0からカウントアップする
 *  record_countはFILE領域内の位置を示すカウンタ。0からカウントアップする
 */

#define MAP0_START_ADDRESS 0x000000
#define MAP0_END_ADDRESS 0x01FFFF

#define MAP1_START_ADDRESS 0x020000
#define MAP1_END_ADDRESS 0x03FFFF

#define FILES_START_ADDRESS 0x040000
#define FILES_END_ADDRESS 0xF3FFFF

#define FILE_SIZE 0x500000      // FILEのサイズ（5MB）
#define LOG_RECORD_SIZE 128     // header 16 byte + log 112 byte
#define MAX_LOGS_PER_FILE (FILE_SIZE / LOG_RECORD_SIZE) // 1FILEあたり40960ログ
#define MAX_FILES 3             // FLASHに保存可能なFILEの最大数

static_assert(
    FILES_START_ADDRESS + MAX_FILES * FILE_SIZE == FILES_END_ADDRESS + 1UL,
    "Flash FILE regions must exactly fill the configured log area"
);

#define MAP_SIZE 2048                                   // 64x64, 4bit/cell
#define MAP_HEADER_SIZE 256                             // 1回のMAPのヘッダサイズ
#define MAP_RECORD_SIZE (MAP_HEADER_SIZE + MAP_SIZE)    // 1回のMAPのサイズ
#define MAPS_PER_SECTOR 1                               // 1セクタに1MAPを保存
#define MAP_SECTOR_PADDING (SECTOR_SIZE - MAP_RECORD_SIZE)
#define MAX_MAPS 32                                     // 128KB / 4KB

#define MAP_VALID_MAGIC 0x5F50414D // asciiでMAP_ 

#define LOG_VALID_MAGIC 0x5F474F4C // asciiでLOG_

#define NO_MAP_UPDATE_COUNT 0xFFFFFFFFUL
#define SECTOR_SIZE 4096UL
#define MAP_REGION_SIZE (MAP1_START_ADDRESS - MAP0_START_ADDRESS)
#define LOG_RECORDS_PER_SECTOR (SECTOR_SIZE / LOG_RECORD_SIZE)

static_assert(MAP_RECORD_SIZE <= SECTOR_SIZE,
              "Packed 4-bit map must fit in one Flash sector");
static_assert(MAX_MAPS * SECTOR_SIZE == MAP_REGION_SIZE,
              "Map records must exactly fill one map region");

namespace Flash {
    constexpr uint8_t MAP_FORMAT_VERSION = 2;
    constexpr uint8_t MAP_BITS_PER_CELL = 4;
    constexpr uint8_t LOG_FORMAT_VERSION = 12;

    // Compact 100 ms snapshot. Header + frame fits one 128-byte record.
    struct LogFrame {
        uint8_t format_version;
        int8_t flash_file_index;
        uint8_t mission_state;
        uint8_t boot_mode;
        uint32_t message_number;
        uint32_t timestamp_ms;
        uint16_t valid_flags;
        uint16_t localization_status_flags;

        int32_t x_mm;
        int32_t y_mm;
        uint16_t yaw_deg_1e2;
        int16_t forward_velocity_mm_s;
        int32_t lat_1e7;
        int32_t lng_1e7;
        uint8_t gps_fix_type;
        uint8_t gps_satellites;
        uint16_t gps_horizontal_accuracy_mm;
        uint8_t sensor_sources;
        uint8_t camera_flags;
        int16_t camera_angle_error_deg10;
        uint16_t camera_occupancy_permille;
        uint8_t camera_confidence;
        uint8_t stuck_reason;
        uint8_t stuck_verification_result;
        uint8_t rasp_state;
        uint8_t gps_state;
        uint8_t flash_state;
        int16_t board_acc_x_mg;
        int16_t board_acc_y_mg;
        int16_t board_acc_z_mg;
        int16_t board_gyro_x_rad_s_x1000;
        int16_t board_gyro_y_rad_s_x1000;
        int16_t board_gyro_z_rad_s_x1000;
        int16_t can_acc_x_mg;
        int16_t can_acc_y_mg;
        int16_t can_acc_z_mg;
        int16_t can_gyro_x_rad_s_x1000;
        int16_t can_gyro_y_rad_s_x1000;
        int16_t can_gyro_z_rad_s_x1000;
        int16_t board_magnetic_x_uT_x10;
        int16_t board_magnetic_y_uT_x10;
        int16_t board_magnetic_z_uT_x10;
        int16_t can_magnetic_x_uT_x10;
        int16_t can_magnetic_y_uT_x10;
        int16_t can_magnetic_z_uT_x10;
        uint16_t pressure_pa_div10;
        int32_t encoder_left_mm;
        int32_t encoder_right_mm;
        uint64_t camera_scene_hash;
        uint16_t gyro_samples_100ms;
        uint8_t accel_samples_100ms;
        uint8_t magnetic_samples_100ms;
        uint16_t imu_fifo_overflow_count;
    } __attribute__((packed));

    static_assert(sizeof(LogFrame) == 112, "Flash LogFrame v12 size mismatch");
}

class SrvFlash
{
public:
    enum class LogReadResult : uint8_t {
        Valid,
        EndOfFile,
        Corrupt,
        ReadError,
        InvalidArgument
    };

    enum class MapLoadResult : uint8_t {
        Valid,
        NotFound,
        OriginMismatch,
        Corrupt,
        ReadError,
        InvalidArgument
    };

    SrvFlash(int32_t origin_lat_1e7, int32_t origin_lon_1e7);
    ~SrvFlash();
    
    bool init(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, uint32_t clock_hz = 8000000, uint8_t mode = 3); 

    // FLASHの初期化を行い、最新MAPをロードする
    bool startNewSequence(uint8_t* map_scratch_buffer);
    /**
     * MAP操作
     */
    // 最新のMAPをmemoryにloadする
    MapLoadResult loadLatestMap(uint8_t* map_buffer_out);
    
    // MAPを保存し、update_countをインクリメントする
    bool saveMap(const uint8_t* map_data);
    
    // MAP[region]を削除する
    bool eraseMap(uint8_t map_region);

    // MAP領域を全て削除する
    bool resetAllMaps(); 
    
    /**
     * FILE操作
     */

    //LogをFILEに追記する
    bool saveLog(const Flash::LogFrame& packet);
    
    // FILE[file_idx]のLog[index]を読み込む
    LogReadResult loadLog(uint8_t file_idx, uint32_t log_index, Flash::LogFrame& packet_out);

    // FILE[file_idx]を削除する
    bool eraseFILE(uint8_t file_idx);

    // 全FILE領域をFILE 0から順番に個別消去する
    bool resetAllFiles();

    /**
     * 状態管理
     */

    // 現在のsession_idを取得する
    uint32_t getSessionId() const;

    int8_t getActiveFileIndex() const; // 0-2, -1: STORAGE_FULL

    // 下位3bitを使用し、bit iが1ならFILE[i]は使用済み。
    uint8_t getUsedFileBitFlag() const;

    // FILE領域がfullかどうかを判定する
    bool isStorageFull() const;
private:
    // 定義
    union MapHeader{
        struct {
            uint32_t magic;       // MAP_VALID_MAGIC
            uint32_t crc32;
            uint32_t session_id;
            uint32_t update_count;
            int32_t origin_lat_1e7;
            int32_t origin_lon_1e7;
            uint8_t format_version;
            uint8_t bits_per_cell;
        } __attribute__((packed)) data;
        uint8_t raw[MAP_HEADER_SIZE];
    };

    struct LogHeader{
        uint32_t magic; // LOG_VALID_MAGIC
        uint32_t crc32;
        uint32_t session_id;
        uint32_t record_count;
    } __attribute__((packed));

    union LogRecord{
        struct {
            LogHeader header;
            Flash::LogFrame packet;
        } __attribute__((packed)) data;
        uint8_t raw[LOG_RECORD_SIZE];
    };

    static_assert(sizeof(MapHeader) == MAP_HEADER_SIZE, "MapHeader size mismatch");
    static_assert(sizeof(Flash::LogFrame) <= LOG_RECORD_SIZE - sizeof(LogHeader),
                "LogFrame does not fit in the fixed Flash record");
    static_assert(sizeof(LogRecord) == LOG_RECORD_SIZE, "LogRecord size mismatch");
    // object
    HalS25FL128L flash;
    // scan
    void scanSessionId();
    void scanMap();
    void scanFiles();
    
    // フィールド南西原点の緯度経度
    int32_t origin_lat_1e7;
    int32_t origin_lon_1e7;
    // MAP,FILE共通
    uint32_t current_session_id;
    
    //MAP管理
    uint8_t active_map_region;
    uint32_t current_update_count;

    //FILE管理
    int8_t active_file_idx; // -1: 満杯
    uint32_t current_log_record_count;
    uint8_t used_file_bit_flag; // 下位3bitだけを使うFILE使用済みフラグ

    uint32_t calculateMapAddress(uint8_t region, uint32_t update_cnt);
    uint32_t calculateLogAddress(uint8_t file_idx, uint32_t record_cnt);
    uint32_t calculateMapCRC32(const uint8_t* data);
    uint32_t calculateLogCRC32(const uint8_t* data);
};
