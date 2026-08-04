#pragma once
#include <Arduino.h>
#include <esp_rom_crc.h>
#include "hal_S25FL128L.h"


// S25FL128Lは16MB = 16 * 1024 KB = 16 * 1024 * 1024 

// 1回のMAPは2048B = 2KB(64x64マップ、1セル4bit)
// MAP保存領域のセクタは header 256B x3 + MAP 1024B x3 + padding 256B = 4096B = 4KBとする
// 1回のログは256B

// MAP保存領域は128KB = 4KB * 32セクタより、96回のMAPを保存可能
// FILEは5120KB = 1280 * 4KB = 5MBより、20480個のログを保存可能

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
#define LOG_RECORD_SIZE 256     // 1回のログのサイズ
#define MAX_LOGS_PER_FILE (FILE_SIZE / LOG_RECORD_SIZE) // 1FILEあたり20480ログ
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
    constexpr uint8_t LOG_FORMAT_VERSION = 5;

    // 100 ms周期の新版ログ。packedにしてPC側とoffsetを固定する。
    struct LogFrame {
        uint8_t format_version;
        int8_t flash_file_index;
        uint8_t mission_state;
        uint8_t boot_mode;
        uint32_t message_number;
        uint32_t timestamp_ms;
        uint16_t valid_flags;
        uint16_t fusion_status_flags;

        int32_t lat_1e7;
        int32_t lng_1e7;
        int32_t gps_x_mm;
        int32_t gps_y_mm;
        int32_t x_mm;
        int32_t y_mm;
        uint16_t yaw_deg_1e2;
        int16_t forward_velocity_mm_s;
        int16_t yaw_rate_rad_s_x1000;
        uint32_t position_std_mm;
        uint16_t yaw_std_mrad;

        uint8_t gps_fix_type;
        uint8_t gps_satellites;
        uint32_t gps_horizontal_accuracy_mm;
        int32_t gps_velocity_east_mm_s;
        int32_t gps_velocity_north_mm_s;
        uint32_t gps_speed_accuracy_mm_s;

        int16_t acc_x_mg;
        int16_t acc_y_mg;
        int16_t acc_z_mg;
        int16_t gyro_z_rad_s_x1000;
        uint16_t magnetic_yaw_deg_1e2;
        int32_t pressure_pa;
        int32_t encoder_left_mm;
        int32_t encoder_right_mm;
        uint16_t imu_age_ms;
        uint16_t magnetic_age_ms;
        uint16_t encoder_age_ms;
        uint16_t gps_age_ms;

        int8_t cell_x;
        int8_t cell_y;
        uint8_t attitude;
        uint8_t stuck_reason;
        uint8_t stuck_cell_x;
        uint8_t stuck_cell_y;
        int16_t jog_velocity_mm_s;
        int16_t jog_omega_rad_s_x100;
        uint16_t jog_remain_ms;

        uint8_t camera_valid;
        uint8_t camera_target_found;
        uint8_t camera_confidence;
        uint16_t camera_occupancy_permille;
        int16_t camera_angle_error_deg10;
        uint8_t rasp_state;
        uint8_t gps_state;
        uint8_t flash_used_flags;
        uint8_t flash_storage_full;
        uint32_t grid_map_update_count;
        uint8_t fusion_quality;
        uint8_t gps_health;
        uint8_t encoder_health;
        uint8_t imu_health;
        uint8_t magnetic_health;
        uint16_t motion_anomaly_flags;
        uint16_t motion_anomaly_age_ms;

        uint16_t stuck_score_wheel_blocked;
        uint16_t stuck_score_wheel_slip;
        uint16_t stuck_score_rotation_blocked;
        uint16_t stuck_score_body_trapped;
        uint8_t stuck_verification_phase;
        uint8_t stuck_trigger_reason;
        uint8_t stuck_recurrence_count;
        uint8_t stuck_verification_result;
        uint8_t stuck_hash_distance_bits;
        int16_t stuck_probe_left_delta_mm;
        int16_t stuck_probe_right_delta_mm;
        int16_t stuck_probe_gyro_angle_mrad;
        uint16_t stuck_tilt_deg_x10;
        uint16_t stuck_gps_max_radius_mm;
        uint16_t stuck_gps_encoder_distance_mm;
        uint8_t stuck_gps_sample_count;
        uint8_t stuck_diagnostics_valid;
        int16_t encoder_left_velocity_mm_s;
        int16_t encoder_right_velocity_mm_s;
        uint64_t camera_scene_hash;
    } __attribute__((packed));

    static_assert(sizeof(LogFrame) == 175, "Flash LogFrame v5 size mismatch");
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
