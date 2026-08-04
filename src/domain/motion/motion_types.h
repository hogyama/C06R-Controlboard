#pragma once

#include <stdint.h>

namespace Domain::Motion {

enum class Reason : uint8_t {
    None = 0,
    WheelBlocked,
    // Wire/log compatibility with older firmware.
    TranslationBlocked = WheelBlocked,
    RotationBlocked,
    EncoderGpsMismatch,
    Flipped,
    MotionUnobservable,
    GpsNoProgress,
    LeftWheelBlocked,
    RightWheelBlocked,
    WheelSlip,
    SensorFault,
    PathNoProgress,
    Oscillation,
    BodyTrapped
};

struct StuckScores {
    uint16_t wheel_blocked;
    uint16_t wheel_slip;
    uint16_t rotation_blocked;
    uint16_t body_trapped;
};

struct DetectorDiagnostics {
    uint16_t tilt_deg_x10;
    uint16_t gps_window_age_ms;
    uint16_t gps_max_radius_mm;
    uint16_t gps_encoder_distance_mm;
    uint8_t gps_sample_count;
};

enum EvidenceFlag : uint32_t {
    EVIDENCE_NONE                 = 0,
    EVIDENCE_ENCODER_NOT_MOVING   = 1U << 0,
    EVIDENCE_ENCODER_MOVING       = 1U << 1,
    EVIDENCE_GYRO_NOT_ROTATING    = 1U << 2,
    EVIDENCE_GYRO_ROTATING        = 1U << 3,
    EVIDENCE_GPS_NOT_MOVING       = 1U << 4,
    EVIDENCE_GPS_MOVING           = 1U << 5,
    EVIDENCE_LEFT_RIGHT_MISMATCH  = 1U << 6,
    EVIDENCE_ENCODER_GYRO_MISMATCH = 1U << 7,
    EVIDENCE_ENCODER_GPS_MISMATCH = 1U << 8,
    EVIDENCE_ENCODER_UNAVAILABLE  = 1U << 9,
    EVIDENCE_GPS_UNAVAILABLE      = 1U << 10,
    EVIDENCE_GYRO_UNAVAILABLE     = 1U << 11,
    EVIDENCE_FUSION_NOT_MOVING    = 1U << 12,
    EVIDENCE_FUSION_MOVING        = 1U << 13,
    EVIDENCE_FUSION_UNAVAILABLE   = 1U << 14,
    EVIDENCE_PATH_NOT_PROGRESSING = 1U << 15,
    EVIDENCE_COMMAND_OSCILLATING  = 1U << 16
};

enum class Condition : uint8_t {
    Normal = 0,
    Suspected,
    Stuck,
    SensorFault
};

struct Assessment {
    uint32_t timestamp_ms;
    uint32_t suspected_since_ms;
    Condition condition;
    Reason reason;
    uint32_t evidence_flags;
    bool encoder_trusted;
    bool gps_trusted;
    bool gyro_trusted;
    StuckScores scores;
    bool suspend_requested;
};

} // namespace Domain::Motion
