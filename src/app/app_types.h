#pragma once 
#include <Arduino.h> 
#include "service/Can/srv_can.h"
#include "algorithm/astar.h"
#include "domain/fusion/fusion_types.h"
#include "domain/motion/motion_types.h"

enum class SystemState : uint8_t {
    // サブキャリアの装着を行うフェーズ　TWELITEのみONにし、TWECOMMANDはサーボロック、開放、シーケンス開始、シーケンスリセットのみ受け付ける。
    // TWELITEでシーケンス開始を送ると、AWAIT_ASCENTに遷移する。
    // CAN送信はサーボロック、開放、シーケンス開始、シーケンスリセットのみ　
    // CAN受信は行わない
    STATE_PRELAUNCH = 0, 
    // シーケンス開始前の待機フェーズ　GPSをONにし、TWELITEの電源を切る。
    // CAN送信は行わない
    // CAN受信は行う
    STATE_AWAIT_ASCENT, 
    // シーケンス開始後の空中フェーズ　
    // CAN送信は行わない
    // CAN受信は行う
    STATE_ASCENT_TO_LANDING,
    // ランディング検出後の地上フェーズ TWELITEの電源を入れる
    // CAN送信は分離要求のみ
    // CAN受信は行う
    STATE_SEPARATION,
    // GPS+MAG+ENCを用いた自律航行フェーズ RASPの電源は入れるが、カメラ要求はしない
    // CAN送信は走行指令、スタック検出のみ
    // CAN受信は行う
    STATE_GPS_NAV,
    // カメラを用いた自律航行フェーズ カメラ要求をして、そこから操作する
    // CAN送信は走行指令、スタック検出のみ
    // CAN受信は行う
    STATE_CAMERA_NAV,
    // 通常スタック検出時の脱出フェーズ
    // CAN送信は行わない
    // CAN受信は行う 
    STATE_ESCAPE,
    // ゴールフェーズ　CAMERA_NAVでゴール検知を閾値以上に行った場合に遷移する。ゴール検知後は、TWELITE,GPS,RASPの電源を切り、FLASHのログ保存を終了する。
    // CAN送信は行わない
    // CAN受信は行わない
    STATE_GOAL,
    // 反転検出後、正常姿勢500ms確認とU送信、完了/失敗通知待ちを行う。
    // 既存ログとの互換性を保つため、従来状態0-7の末尾へ追加する。
    STATE_UPRIGHT_RECOVERY = 8
};

enum class BootMode : uint8_t {
    SEQUENCE = 0,
    MANUAL, // どの状態でもいかなるTWEコマンドも受け付ける。ただし実行可能かはtask_state側で判定　通常の自律走行に介入して操作するフェーズ
    DEBUG // GPS・RASP・TWEと融合処理は停止。SerialでFlashを操作し、CANはReset/SequenceStart送信と受信破棄だけ行う。
};

struct SystemData{
    SystemState state;
    BootMode boot_mode;
};


enum class SystemCmdType : uint8_t {
    None = 0,
    
    // GPS NAV -> CAMERA NAV
    StartCameraNav,
    // CAMERA NAV -> GPS NAV
    StartGpsNav,
    
    // Can Transmit ot Manual Inject
    Reset,
    StartSequence,
    NotifyGoal, 
    NotifySeparation, 
    NotifyStuck, 
    NotifyFlipped,
    ConfirmUpright,
    ServoUnlock, 
    ServoLock, 
    
    // Can Receive or Manual Inject
    StuckResolved,      
    UprightRecoveryFailed,
    SeparationFinished, 
    AscentDetected,    
    LandingDetected,    

    // MANUAL Mode Command
    ForceGpsNav, 
    ForceCameraNav, 
    ForceEscape
};

enum class JogSource : uint8_t {
    Navigation = 0,
    Manual
};

// 候補指令はすべてArbiterへ送り、CAN用mailboxを直接上書きしない。
enum class MotionCommandSource : uint8_t {
    Stop = 0,
    GpsNavigation,
    CameraNavigation,
    Escape,
    Manual,
    Safety
};

struct MotionCommandRequest {
    MotionCommandSource source;
    int16_t velocity_mm_s;
    int16_t omega_rad_s_x100;
    uint16_t duration_ms;
    uint32_t timestamp_ms;
};

struct JogData : public Can::Command::Velocity {
    uint32_t duration_ms;
    uint32_t timestamp_ms;

    // EKFの状態と不確かさ。ログ側は推定器内部へ直接アクセスしない。
    JogSource source;
};

enum CordinateSourceFlag : uint8_t {
    CORD_SRC_NONE    = 0,
    CORD_SRC_GPS     = 1 << 0,
    CORD_SRC_ENCODER = 1 << 1,
    CORD_SRC_HEADING = 1 << 2,
    CORD_SRC_HOLD    = 1 << 3,
};

enum class Attitude : uint8_t {
    Normal = 0,
    Flipped,
    HighTilt,
    Unknown
};

struct Coordinate {
    int32_t x_mm;
    int32_t y_mm;
    float heading_rad;      // rad, 東0度・反時計回り正
    Attitude attitude;      // Normal, Flipped, HighTilt, Unknown
    
    uint8_t source_flags;   // CordinateSourceFlagのbitflags

    int32_t gps_x_mm;          // mm, 東を0度として反時計回り正の角度とする座標系
    int32_t gps_y_mm;          // mm, 東を0度として反時計回り正の角度とする座標系
    int32_t encoder_left_mm;   // mm
    int32_t encoder_right_mm;  // mm

    uint32_t timestamp_ms;

    // EKFの状態と不確かさ。ログ側は推定器内部へ直接アクセスしない。
    float forward_velocity_mm_s;
    float yaw_rate_rad_s;
    uint32_t position_std_mm;
    float yaw_std_rad;
    uint16_t fusion_status_flags;
    Domain::Fusion::Quality fusion_quality;
    Domain::Fusion::SensorHealth gps_health;
    Domain::Fusion::SensorHealth encoder_health;
    Domain::Fusion::SensorHealth imu_health;
    Domain::Fusion::SensorHealth magnetic_health;
    uint16_t motion_anomaly_flags;
    uint32_t motion_anomaly_since_ms;

    bool is_first_gps_valid; // 最初にGPSが有効にならないと、ゴールとの位置関係が全く分からないため
};

// taskStuckからtaskNavへ渡す地図更新要求
// grid_mapの書き換えはtaskNavだけが行う。
struct MapUpdate {
    int32_t world_x_mm;
    int32_t world_y_mm;
    int8_t evidence_delta;
    uint8_t radius_cells;
    uint8_t maximum_value;
};

struct NavigationProgress {
    uint32_t timestamp_ms;
    uint32_t path_revision;
    uint16_t nearest_index;
    uint16_t target_index;
    float distance_to_goal_mm;
    bool valid;
};

using StuckReason = Domain::Motion::Reason;

// taskStuckが確定した理由と障害物セルをtaskLogへ共有する。
struct StuckStatus {
    StuckReason reason;
    uint8_t obstacle_cell_x; // 0-63、障害物登録なしは255
    uint8_t obstacle_cell_y; // 0-63、障害物登録なしは255
    uint32_t timestamp_ms;
};

enum class FlashDebugRequestType : uint8_t {
    None = 0,
    ListFiles,
    ReadLog,
    ReadLatestMap,
    EraseFile,
    EraseAllFiles,
    EraseAllMaps
};

enum class FlashDebugResult : uint8_t {
    Ok = 0,
    NotReady,
    NotFound,
    EndOfFile,
    InvalidArgument,
    OriginMismatch,
    Corrupt,
    ReadError,
    EraseError
};

enum class FlashResponseTarget : uint8_t {
    Debug = 0,
    Navigation
};

struct FlashDebugRequest {
    FlashDebugRequestType type;
    FlashResponseTarget response_target;
    uint8_t file_index;
    uint32_t log_index;
    uint32_t request_id;
};

// 最大のマップ1件を応答に収め、taskDebug側で16byteずつHex表示する。
struct FlashDebugResponse {
    FlashDebugRequestType type;
    FlashDebugResult result;
    uint32_t request_id;
    uint8_t file_index;
    uint32_t log_index;
    uint8_t used_file_flags; // 下位3bit。bit iが1ならFILE[i]は使用済み
    uint16_t data_size;
    uint8_t data[AStar::MAP_BYTES];
};

// LED_MAPへ通知するFlash保存イベント。LogFileFullは現在の1ファイル満杯を表す。
enum class FlashLedEvent : uint8_t {
    None = 0,
    LogSaved,
    MapSaved,
    WriteError,
    LogFileFull,
    InitError
};

// Flashデバイスの状態もtaskFlashだけが取得し、他タスクへmailboxで共有する。
struct FlashStatus {
    bool initialized;
    int8_t active_file_index;
    uint8_t used_file_flags; // 下位3bit。bit iが1ならFILE[i]は使用済み
    bool storage_full;
    FlashLedEvent last_event;
    uint32_t event_timestamp_ms;
    uint32_t timestamp_ms;
};
