#include "domain/navigation/camera_navigator.h"
#include "domain/sensor/sensor_freshness.h"

#include <Arduino.h>
#include <math.h>
#include <string.h>

namespace CameraNavigation {
namespace {

constexpr uint32_t NAV_PERIOD_MS = 10;
constexpr uint32_t JOG_DURATION_MS = 300;

constexpr uint32_t CAMERA_COMM_TIMEOUT_MS = 500;
constexpr uint32_t CAMERA_CONTROL_FRAME_TIMEOUT_MS = 400;
constexpr uint32_t CAMERA_GYRO_TIMEOUT_MS = 100;
constexpr uint32_t CAMERA_GYRO_MAX_DT_MS = 100;

// 検出直後はカメラ値が揺れやすいため、連続検出を確認するまで停止する。
// 連続検出はやめ、1枚にする

constexpr uint8_t CAMERA_TARGET_CONFIRM_FRAMES = 1;

// 探索走行中の一瞬の誤検知を採用しないため、完全静止後の新規画像2枚で再確認する。
constexpr uint8_t CAMERA_SEARCH_DETECTION_CONFIRM_FRAMES = 2;
constexpr uint8_t CAMERA_SEARCH_DETECTION_OBSERVE_FRAMES = 6;
constexpr int16_t CAMERA_SEARCH_DETECTION_ANGLE_TOLERANCE_DEG10 = 200;
constexpr float CAMERA_SEARCH_REWIND_MIN_RAD = 20.0f * (M_PI / 180.0f);
constexpr float CAMERA_SEARCH_REWIND_MAX_RAD = 45.0f * (M_PI / 180.0f);

// この機体は微小旋回が難しいため、±15度は正面として静止画像2枚で確認する。
constexpr int16_t CAMERA_FORWARD_DEAD_BAND_DEG10 = 150;
// 逆補正を使った後だけは、±30度までを粗い正面として低速前進を許可する。
constexpr int16_t CAMERA_COARSE_FORWARD_DEAD_BAND_DEG10 = 300;
constexpr int16_t CAMERA_GOAL_MAX_ANGLE_DEG10 = 100;
constexpr uint8_t CAMERA_GOAL_MIN_CONFIDENCE = 70;
constexpr uint16_t CAMERA_GOAL_OCCUPANCY_PERMILLE = 700;
constexpr uint8_t CAMERA_GOAL_REQUIRED_FRAMES = 5;
constexpr uint8_t CAMERA_ANGLE_HISTORY_SIZE = 2;

constexpr float CAMERA_FAR_FORWARD_SPEED_MM_S = 800.0f;
constexpr float CAMERA_NEAR_FORWARD_SPEED_MM_S = 450.0f;
constexpr uint16_t CAMERA_NEAR_OCCUPANCY_PERMILLE = 250;

// 前進開始はトルク確保のため即時、減速だけを緩やかにする。
// 300 mm/s未満は実機で駆動力を得にくいため、その時点で停止指令へ切り替える。
constexpr float CAMERA_FORWARD_MAX_DECELERATION_MM_S2 = 1000.0f;
constexpr float CAMERA_FORWARD_MAX_JERK_MM_S3 = 4000.0f;
constexpr float CAMERA_FORWARD_DECEL_STOP_SPEED_MM_S = 50.0f;
constexpr float CAMERA_FORWARD_DECEL_STOP_ACCEL_MM_S2 = 100.0f;
constexpr uint32_t CAMERA_FORWARD_DECEL_MAX_DT_MS = 100;

// taskNavが停止判断できなくなった場合にも短時間でJogを失効させる。
constexpr uint32_t CAMERA_CONTROL_COMMAND_MS = 80;

// 静止摩擦を確実に越えるため、通常補正・探索とも旋回角速度を固定する。
constexpr float CAMERA_TURN_OMEGA_RAD_S = 3.3f;
constexpr float CAMERA_TURN_REACHED_RAD = 5.0f * (M_PI / 180.0f);
constexpr float CAMERA_SEARCH_REACHED_RAD = 2.0f * (M_PI / 180.0f);
constexpr float CAMERA_MAX_GYRO_RATE_RAD_S = 20.0f;
constexpr float CAMERA_COAST_PREDICTION_S = 0.06f;
constexpr float CAMERA_STILL_GYRO_RAD_S = 0.30f;
constexpr float CAMERA_DELAY_GYRO_DEADBAND_RAD_S = 0.05f;
constexpr float CAMERA_STILL_ENCODER_MM_S = 100.0f;
constexpr uint32_t CAMERA_STILL_CONFIRM_MS = 200;
constexpr uint32_t CAMERA_STILL_NO_SENSOR_MS = 400;
constexpr uint32_t CAMERA_ENCODER_TIMEOUT_MS = 100;
constexpr float CAMERA_TRACK_WIDTH_MM = 180.0f;

// 明示的な未検出ごとに45度旋回する
// 探索では逆方向の補正旋回を行わない。
constexpr float CAMERA_SEARCH_STEP_RAD = 45.0f * (M_PI / 180.0f);
constexpr float CAMERA_TARGET_MAX_TURN_RAD = 45.0f * (M_PI / 180.0f);
constexpr float CAMERA_SEARCH_FULL_TURN_RAD = 2.0f * M_PI;
constexpr float CAMERA_SEARCH_LEG_BASE_MM = 500.0f;
constexpr float CAMERA_SEARCH_LEG_MAX_MM = 1500.0f;
constexpr float CAMERA_SEARCH_MOVE_SPEED_MM_S = 800.0f;
constexpr float CAMERA_SEARCH_CORNER_RAD = 90.0f * (M_PI / 180.0f);

constexpr uint8_t CAMERA_GYRO_HISTORY_SIZE = 64;

enum class CameraPhase : uint8_t {
    WaitFrame,
    TurnDriving,
    TurnCoasting,
    StopSettling,
    Forward,
    TranslationDecelerating,
    GoalConfirm,
    SearchDetectionConfirm,
    SearchTranslate,
    LinkHold,
};

enum class CameraTurnPurpose : uint8_t {
    None,
    Target,
    SearchStep,
    SearchCorner,
    SearchReacquire,
};

enum class CameraAfterStop : uint8_t {
    WaitFrame,
    StartSearch,
    StartGoalConfirm,
    StartSearchDetectionConfirm,
};

enum class CameraSearchResume : uint8_t {
    None,
    SearchStep,
    SearchCorner,
    Translation,
};

struct CameraGyroSample {
    uint32_t timestamp_ms;
    float z_rad_s;
};

// カメラジャイロ履歴
struct CameraGyroHistory {
    CameraGyroSample samples[CAMERA_GYRO_HISTORY_SIZE]{};
    uint8_t count = 0;
    uint8_t write_index = 0;

    void clear()
    {
        count = 0;
        write_index = 0;
    }

    void add(const Sensor::GyroscopeData& gyro)
    {
        const uint32_t timestamp_ms = static_cast<uint32_t>(
            gyro.metadata.timestamp_us / 1000ULL);
        if (count > 0) {
            // 最新データのタイムスタンプが同じ場合は追加しない
            const uint8_t newest_index =
                (write_index + CAMERA_GYRO_HISTORY_SIZE - 1U) %
                CAMERA_GYRO_HISTORY_SIZE;
            if (samples[newest_index].timestamp_ms == timestamp_ms) return;
        }
        samples[write_index] = {timestamp_ms, gyro.z_rad_s};
        write_index = (write_index + 1U) % CAMERA_GYRO_HISTORY_SIZE;
        if (count < CAMERA_GYRO_HISTORY_SIZE) ++count;
        // カウントが最大に達した場合、古いデータは上書きされる
    }

    bool integrateDelay(
        uint32_t from_ms,
        uint32_t to_ms,
        float& angle_rad) const
    {
        angle_rad = 0.0f;
        if (count < 2 || from_ms == 0 || to_ms <= from_ms) return false;

        const uint8_t first =
            (write_index + CAMERA_GYRO_HISTORY_SIZE - count) %
            CAMERA_GYRO_HISTORY_SIZE;
        bool covered_from = false;
        bool used_interval = false;

        for (uint8_t i = 0; i + 1U < count; ++i) {
            // 2点の間の積分区間が、指定されたfrom_ms～to_msに重なっている場合のみ積分する
            const CameraGyroSample& a =
                samples[(first + i) % CAMERA_GYRO_HISTORY_SIZE];
            const CameraGyroSample& b =
                samples[(first + i + 1U) % CAMERA_GYRO_HISTORY_SIZE];
            if (b.timestamp_ms <= a.timestamp_ms) continue;
            if (b.timestamp_ms < from_ms || a.timestamp_ms > to_ms) continue;

            // 積分区間の開始・終了点
            const uint32_t segment_start =
                a.timestamp_ms < from_ms ? from_ms : a.timestamp_ms;
            const uint32_t segment_end =
                b.timestamp_ms > to_ms ? to_ms : b.timestamp_ms;
            if (segment_end <= segment_start) continue;

            // 2点間の時間差（端以外は積分区間）
            const float interval_ms =
                static_cast<float>(b.timestamp_ms - a.timestamp_ms);

            // 積分区間の開始・終了点の角速度を線形補間する
            const float start_ratio =
                (segment_start - a.timestamp_ms) / interval_ms;
            const float end_ratio =
                (segment_end - a.timestamp_ms) / interval_ms;
            
            // 積分区間の開始・終了点の角速度（端は線形補完）
            float start_rate =
                a.z_rad_s + (b.z_rad_s - a.z_rad_s) * start_ratio;
            float end_rate =
                a.z_rad_s + (b.z_rad_s - a.z_rad_s) * end_ratio;

            // 画像遅延補正は静止ノイズを角度へ積算しない。
            if (fabsf(start_rate) < CAMERA_DELAY_GYRO_DEADBAND_RAD_S) {
                start_rate = 0.0f;
            }
            if (fabsf(end_rate) < CAMERA_DELAY_GYRO_DEADBAND_RAD_S) {
                end_rate = 0.0f;
            }

            // 台形積分
            angle_rad += 0.5f * (start_rate + end_rate) *
                ((segment_end - segment_start) / 1000.0f);
            covered_from = covered_from || a.timestamp_ms <= from_ms;
            used_interval = true;
        }
        return covered_from && used_interval;
    }
};

struct State {
    uint32_t last_camera_msg_number = 0;
    uint32_t last_camera_frame_ms = 0;
    uint8_t camera_target_confirm_count = 0;
    uint8_t camera_goal_detect_count = 0;
    int16_t camera_angle_history[CAMERA_ANGLE_HISTORY_SIZE] = {};
    uint8_t camera_angle_history_count = 0;
    uint8_t camera_angle_history_write_index = 0;
    int16_t camera_filtered_angle_deg10 = 0;
    CameraPhase camera_phase = CameraPhase::WaitFrame;
    CameraAfterStop camera_after_stop = CameraAfterStop::WaitFrame;
    CameraTurnPurpose camera_turn_purpose = CameraTurnPurpose::None;
    float camera_turn_target_rad = 0.0f;
    float camera_accumulated_turn_rad = 0.0f;
    float camera_previous_gyro_z_rad_s = 0.0f;
    uint32_t camera_previous_gyro_timestamp_ms = 0;
    uint32_t camera_turn_started_ms = 0;
    uint32_t camera_still_started_ms = 0;
    // 正面へ収まるまで、停止後画像による逆方向補正は1回だけ許可する。
    int8_t camera_last_target_turn_direction = 0;
    bool camera_target_reverse_used = false;
    bool camera_coarse_forward_active = false;
    uint8_t camera_large_error_confirm_count = 0;
    int8_t camera_large_error_direction = 0;
    CameraGyroHistory camera_gyro_history{};

    Can::Data::Encoder camera_previous_encoder{};
    bool camera_have_previous_encoder = false;
    float camera_left_velocity_mm_s = 0.0f;
    float camera_right_velocity_mm_s = 0.0f;
    uint32_t camera_encoder_velocity_ms = 0;

    // CAMERA_NAVの直進指令だけを保持し、停止前の減速に使用する。
    float camera_forward_command_mm_s = 0.0f;
    float camera_forward_acceleration_mm_s2 = 0.0f;
    uint32_t camera_forward_command_ms = 0;

    bool camera_search_requested = false;
    float camera_search_accumulated_rad = 0.0f;
    uint8_t camera_search_leg_index = 0;
    bool camera_search_detection_pending = false;
    uint8_t camera_search_detection_frame_count = 0;
    uint8_t camera_search_detection_confirm_count = 0;
    int16_t camera_search_detection_first_angle_deg10 = 0;
    bool camera_search_detection_rewind_used = false;
    float camera_search_detection_delay_turn_rad = 0.0f;
    float camera_search_detection_turn_at_trigger_rad = 0.0f;
    CameraSearchResume camera_search_resume = CameraSearchResume::None;
    float camera_search_resume_turn_rad = 0.0f;
    int32_t camera_search_start_left_mm = 0;
    int32_t camera_search_start_right_mm = 0;
    float camera_search_target_distance_mm = 0.0f;
    uint32_t camera_search_translate_started_ms = 0;
    uint32_t camera_search_translate_duration_ms = 0;
};

State state{};
Output current_output{};

void publishJog(
    float velocity_mm_s,
    float omega_rad_s,
    uint32_t duration_ms = JOG_DURATION_MS)
{
    current_output.velocity_mm_s = velocity_mm_s;
    current_output.omega_rad_s = omega_rad_s;
    current_output.duration_ms = static_cast<uint16_t>(duration_ms);
}

void publishStop()
{
    publishJog(0.0f, 0.0f);
}

bool cameraGyroIsUsable(
    const Sensor::GyroscopeData& angular_velocity,
    uint64_t now_us)
{
    return
        Sensor::sampleIsFresh(
            angular_velocity.metadata,
            now_us,
            CAMERA_GYRO_TIMEOUT_MS * 1000ULL) &&
        isfinite(angular_velocity.z_rad_s) &&
        fabsf(angular_velocity.z_rad_s) <= CAMERA_MAX_GYRO_RATE_RAD_S;
}

bool driveCameraTurn(
    float target_turn_rad,
    float accumulated_turn_rad,
    float gyro_z_rad_s,
    float reached_rad = CAMERA_TURN_REACHED_RAD)
{
    const float remaining_turn_rad =
        target_turn_rad - accumulated_turn_rad;
    const float abs_remaining_turn_rad = fabsf(remaining_turn_rad);
    const float turn_direction = remaining_turn_rad >= 0.0f ? 1.0f : -1.0f;
    const float same_direction_rate =
        fmaxf(0.0f, turn_direction * gyro_z_rad_s);
    const float predicted_coast_angle =
        same_direction_rate * CAMERA_COAST_PREDICTION_S;
    const float stop_angle = reached_rad + predicted_coast_angle;

    if (abs_remaining_turn_rad <= stop_angle) {
        publishStop();
        return true;
    }

    const float omega_rad_s =
        turn_direction * CAMERA_TURN_OMEGA_RAD_S;

    publishJog(0.0f, omega_rad_s, CAMERA_CONTROL_COMMAND_MS);
    return false;
}

} // namespace

void reset()
{
    state = State{};
}

Output update(const Input& input)
{
    current_output = Output{};
    const uint32_t now_ms = input.now_ms;

            const Sensor::GyroscopeData& camera_gyro = input.gyroscope;
            const bool camera_gyro_usable =
                input.has_gyroscope &&
                cameraGyroIsUsable(camera_gyro, input.now_us);
            const uint32_t camera_gyro_timestamp_ms =
                static_cast<uint32_t>(camera_gyro.metadata.timestamp_us / 1000ULL);
            if (camera_gyro_usable) {
                state.camera_gyro_history.add(camera_gyro);
                if ((state.camera_phase == CameraPhase::TurnDriving ||
                     state.camera_phase == CameraPhase::TurnCoasting) &&
                    state.camera_previous_gyro_timestamp_ms != 0 &&
                    camera_gyro_timestamp_ms != state.camera_previous_gyro_timestamp_ms) {
                    const uint32_t gyro_dt_ms = static_cast<uint32_t>(
                        camera_gyro_timestamp_ms - state.camera_previous_gyro_timestamp_ms);
                    if (gyro_dt_ms <= CAMERA_GYRO_MAX_DT_MS) {
                        state.camera_accumulated_turn_rad += 0.5f *
                            (state.camera_previous_gyro_z_rad_s +
                             camera_gyro.z_rad_s) *
                            (gyro_dt_ms / 1000.0f);
                    }
                }
                if (state.camera_phase == CameraPhase::TurnDriving ||
                    state.camera_phase == CameraPhase::TurnCoasting) {
                    state.camera_previous_gyro_z_rad_s = camera_gyro.z_rad_s;
                    state.camera_previous_gyro_timestamp_ms = camera_gyro_timestamp_ms;
                }
            }

            // 累積エンコーダの実timestamp差から左右速度を求める。
            const Can::Data::Encoder& camera_encoder = input.encoder;
            const bool camera_has_encoder = input.has_encoder &&
                Sensor::sampleIsFresh(
                    camera_encoder.metadata,
                    input.now_us,
                    CAMERA_ENCODER_TIMEOUT_MS * 1000ULL);
            if (camera_has_encoder &&
                (!state.camera_have_previous_encoder ||
                 camera_encoder.metadata.timestamp_us !=
                    state.camera_previous_encoder.metadata.timestamp_us)) {
                if (state.camera_have_previous_encoder &&
                    camera_encoder.metadata.timestamp_us >
                        state.camera_previous_encoder.metadata.timestamp_us) {
                    const uint32_t encoder_dt_ms = static_cast<uint32_t>(
                        (camera_encoder.metadata.timestamp_us -
                         state.camera_previous_encoder.metadata.timestamp_us) / 1000ULL);
                    if (encoder_dt_ms <= CAMERA_ENCODER_TIMEOUT_MS) {
                        const float delta_left_mm = static_cast<float>(
                            camera_encoder.left_mm -
                            state.camera_previous_encoder.left_mm);
                        const float delta_right_mm = static_cast<float>(
                            camera_encoder.right_mm -
                            state.camera_previous_encoder.right_mm);
                        state.camera_left_velocity_mm_s =
                            delta_left_mm * (1000.0f / encoder_dt_ms);
                        state.camera_right_velocity_mm_s =
                            delta_right_mm * (1000.0f / encoder_dt_ms);
                        if (!camera_gyro_usable &&
                            (state.camera_phase == CameraPhase::TurnDriving ||
                             state.camera_phase == CameraPhase::TurnCoasting)) {
                            state.camera_accumulated_turn_rad +=
                                (delta_right_mm - delta_left_mm) /
                                CAMERA_TRACK_WIDTH_MM;
                        }
                        state.camera_encoder_velocity_ms = static_cast<uint32_t>(
                            camera_encoder.metadata.received_us / 1000ULL);
                    } else {
                        state.camera_encoder_velocity_ms = 0;
                    }
                }
                state.camera_previous_encoder = camera_encoder;
                state.camera_have_previous_encoder = true;
            }

            const bool camera_encoder_velocity_usable =
                state.camera_encoder_velocity_ms != 0 &&
                static_cast<uint32_t>(
                    now_ms - state.camera_encoder_velocity_ms) <=
                    CAMERA_ENCODER_TIMEOUT_MS;
            // Either motion sensor can confirm a stop. If both are available,
            // both must agree; failure of one sensor must not freeze CameraNav.
            const bool camera_has_still_reference =
                camera_gyro_usable || camera_encoder_velocity_usable;
            const bool camera_gyro_still = !camera_gyro_usable ||
                fabsf(camera_gyro.z_rad_s) < CAMERA_STILL_GYRO_RAD_S;
            const bool camera_encoder_still = !camera_encoder_velocity_usable ||
                (fabsf(state.camera_left_velocity_mm_s) <
                     CAMERA_STILL_ENCODER_MM_S &&
                 fabsf(state.camera_right_velocity_mm_s) <
                     CAMERA_STILL_ENCODER_MM_S);
            const bool camera_is_still_now = !camera_has_still_reference ||
                (camera_gyro_still && camera_encoder_still);
            if (camera_is_still_now) {
                if (state.camera_still_started_ms == 0) {
                    state.camera_still_started_ms = now_ms;
                }
            } else {
                state.camera_still_started_ms = 0;
            }
            const uint32_t still_confirm_ms = camera_has_still_reference
                ? CAMERA_STILL_CONFIRM_MS : CAMERA_STILL_NO_SENSOR_MS;
            const bool camera_still_confirmed =
                state.camera_still_started_ms != 0 &&
                static_cast<uint32_t>(now_ms - state.camera_still_started_ms) >=
                    still_confirm_ms;

            auto startCameraTurn = [&](
                CameraTurnPurpose purpose,
                float target_rad) {
                state.camera_phase = CameraPhase::TurnDriving;
                state.camera_turn_purpose = purpose;
                state.camera_turn_target_rad = target_rad;
                state.camera_accumulated_turn_rad = 0.0f;
                state.camera_turn_started_ms = now_ms;
                state.camera_still_started_ms = 0;
                state.camera_previous_gyro_z_rad_s =
                    camera_gyro_usable ? camera_gyro.z_rad_s : 0.0f;
                state.camera_previous_gyro_timestamp_ms =
                    camera_gyro_usable ? camera_gyro_timestamp_ms : 0;
            };

            auto startSearchTranslation = [&]() {
                const uint8_t expansion =
                    static_cast<uint8_t>(state.camera_search_leg_index / 2U + 1U);
                state.camera_search_target_distance_mm = fminf(
                    CAMERA_SEARCH_LEG_BASE_MM * expansion,
                    CAMERA_SEARCH_LEG_MAX_MM);
                if (camera_has_encoder) {
                    state.camera_search_start_left_mm = camera_encoder.left_mm;
                    state.camera_search_start_right_mm = camera_encoder.right_mm;
                }
                state.camera_search_translate_started_ms = now_ms;
                state.camera_search_translate_duration_ms =
                    static_cast<uint32_t>(lroundf(
                        state.camera_search_target_distance_mm * 1000.0f /
                        CAMERA_SEARCH_MOVE_SPEED_MM_S));
                state.camera_phase = CameraPhase::SearchTranslate;
                state.camera_still_started_ms = 0;
            };

            auto startTranslationDeceleration = [&] (
                CameraAfterStop action) {
                state.camera_phase = CameraPhase::TranslationDecelerating;
                state.camera_after_stop = action;
                state.camera_turn_purpose = CameraTurnPurpose::None;
                state.camera_still_started_ms = 0;
                // 減速開始時は加速度0から始め、急な制動トルクを避ける。
                state.camera_forward_acceleration_mm_s2 = 0.0f;
                state.camera_forward_command_ms = now_ms;
            };

            auto updateCameraForwardSpeed = [&] (float target_speed_mm_s) {
                const uint32_t elapsed_ms = state.camera_forward_command_ms != 0U
                    ? static_cast<uint32_t>(now_ms - state.camera_forward_command_ms)
                    : NAV_PERIOD_MS;
                const uint32_t limited_elapsed_ms = elapsed_ms >
                        CAMERA_FORWARD_DECEL_MAX_DT_MS
                    ? CAMERA_FORWARD_DECEL_MAX_DT_MS
                    : elapsed_ms;
                const float dt_s =
                    static_cast<float>(limited_elapsed_ms) * 0.001f;

                if (state.camera_forward_command_mm_s <= target_speed_mm_s) {
                    state.camera_forward_command_mm_s = target_speed_mm_s;
                    state.camera_forward_acceleration_mm_s2 = 0.0f;
                    state.camera_forward_command_ms = now_ms;
                    return;
                }

                // 現在の負加速度を0へ戻す間に失う速度を見積もり、
                // 停止直前では先に制動を緩めてS字状に速度を落とす。
                const float speed_to_remove_mm_s =
                    state.camera_forward_command_mm_s - target_speed_mm_s;
                const float deceleration_mm_s2 = fmaxf(
                    0.0f,
                    -state.camera_forward_acceleration_mm_s2);
                const float release_speed_mm_s =
                    deceleration_mm_s2 * deceleration_mm_s2 /
                    (2.0f * CAMERA_FORWARD_MAX_JERK_MM_S3);
                const float target_acceleration_mm_s2 =
                    speed_to_remove_mm_s <= release_speed_mm_s
                        ? 0.0f
                        : -CAMERA_FORWARD_MAX_DECELERATION_MM_S2;
                const float max_acceleration_change =
                    CAMERA_FORWARD_MAX_JERK_MM_S3 * dt_s;
                const float previous_acceleration_mm_s2 =
                    state.camera_forward_acceleration_mm_s2;
                state.camera_forward_acceleration_mm_s2 += constrain(
                    target_acceleration_mm_s2 -
                        state.camera_forward_acceleration_mm_s2,
                    -max_acceleration_change,
                    max_acceleration_change);

                state.camera_forward_command_mm_s +=
                    0.5f *
                    (previous_acceleration_mm_s2 +
                     state.camera_forward_acceleration_mm_s2) *
                    dt_s;
                if (state.camera_forward_command_mm_s <= target_speed_mm_s) {
                    state.camera_forward_command_mm_s = target_speed_mm_s;
                    state.camera_forward_acceleration_mm_s2 = 0.0f;
                }
                state.camera_forward_command_ms = now_ms;
            };

            auto resetSearchDetectionObservation = [&]() {
                state.camera_search_detection_frame_count = 0;
                state.camera_search_detection_confirm_count = 0;
                state.camera_search_detection_first_angle_deg10 = 0;
            };

            auto resetSearchProgress = [&]() {
                state.camera_search_requested = false;
                state.camera_search_accumulated_rad = 0.0f;
                state.camera_search_leg_index = 0;
                state.camera_search_detection_pending = false;
                resetSearchDetectionObservation();
                state.camera_search_detection_rewind_used = false;
                state.camera_search_detection_delay_turn_rad = 0.0f;
                state.camera_search_detection_turn_at_trigger_rad = 0.0f;
                state.camera_search_resume = CameraSearchResume::None;
                state.camera_search_resume_turn_rad = 0.0f;
            };

            auto completeSearchStep = [&]() {
                if (state.camera_search_accumulated_rad >=
                    CAMERA_SEARCH_FULL_TURN_RAD) {
                    state.camera_search_accumulated_rad = 0.0f;
                    if (state.camera_search_leg_index == 0) {
                        startSearchTranslation();
                    } else {
                        startCameraTurn(
                            CameraTurnPurpose::SearchCorner,
                            CAMERA_SEARCH_CORNER_RAD);
                    }
                } else {
                    state.camera_phase = CameraPhase::WaitFrame;
                }
            };

            auto resumeInterruptedSearch = [&]() {
                const CameraSearchResume resume = state.camera_search_resume;
                const float remaining_turn_rad =
                    state.camera_search_resume_turn_rad;
                state.camera_search_detection_pending = false;
                resetSearchDetectionObservation();
                state.camera_search_detection_rewind_used = false;
                state.camera_search_detection_delay_turn_rad = 0.0f;
                state.camera_search_detection_turn_at_trigger_rad = 0.0f;
                state.camera_search_resume = CameraSearchResume::None;
                state.camera_search_resume_turn_rad = 0.0f;

                if (resume == CameraSearchResume::SearchStep) {
                    if (fabsf(remaining_turn_rad) >
                        CAMERA_SEARCH_REACHED_RAD) {
                        startCameraTurn(
                            CameraTurnPurpose::SearchStep,
                            remaining_turn_rad);
                    } else {
                        completeSearchStep();
                    }
                } else if (resume == CameraSearchResume::SearchCorner) {
                    if (fabsf(remaining_turn_rad) >
                        CAMERA_SEARCH_REACHED_RAD) {
                        startCameraTurn(
                            CameraTurnPurpose::SearchCorner,
                            remaining_turn_rad);
                    } else {
                        startSearchTranslation();
                    }
                } else if (resume == CameraSearchResume::Translation) {
                    // 探索開始位置と目標距離を保持したまま、残りの辺を走る。
                    state.camera_phase = CameraPhase::SearchTranslate;
                    state.camera_still_started_ms = 0;
                } else {
                    state.camera_phase = CameraPhase::WaitFrame;
                }
            };

            auto finishSearchDetectionObservation = [&]() {
                const bool interrupted_turn =
                    state.camera_search_resume == CameraSearchResume::SearchStep ||
                    state.camera_search_resume == CameraSearchResume::SearchCorner;
                const float delayed_turn_abs =
                    fabsf(state.camera_search_detection_delay_turn_rad);

                if (interrupted_turn &&
                    !state.camera_search_detection_rewind_used &&
                    delayed_turn_abs > CAMERA_SEARCH_REACHED_RAD) {
                    // 画像要求から停止までに通り過ぎた分だけ、一度だけ逆へ探し直す。
                    const float rewind_magnitude = constrain(
                        delayed_turn_abs,
                        CAMERA_SEARCH_REWIND_MIN_RAD,
                        CAMERA_SEARCH_REWIND_MAX_RAD);
                    const float rewind_direction =
                        state.camera_search_detection_delay_turn_rad >= 0.0f
                            ? -1.0f
                            : 1.0f;
                    state.camera_search_detection_rewind_used = true;
                    resetSearchDetectionObservation();
                    startCameraTurn(
                        CameraTurnPurpose::SearchReacquire,
                        rewind_direction * rewind_magnitude);
                } else {
                    resumeInterruptedSearch();
                }
            };

            const Rasp::CameraData& camera_data = input.camera;
            const bool has_frame = input.has_camera;
            const Rasp::Frame& frame = camera_data.frame;
            const bool new_frame = has_frame && frame.msg_number != state.last_camera_msg_number;

            if (new_frame) {
                state.last_camera_msg_number = frame.msg_number;
                state.last_camera_frame_ms = camera_data.received_ms;
                if (state.camera_phase == CameraPhase::LinkHold) {
                    state.camera_phase = CameraPhase::WaitFrame;
                }

                float delay_yaw_rad = 0.0f;
                const uint32_t estimated_capture_ms =
                    camera_data.requested_ms - frame.capture_age_ms;
                state.camera_gyro_history.integrateDelay(
                    estimated_capture_ms,
                    camera_data.received_ms,
                    delay_yaw_rad);
                const int32_t compensated_angle_deg10 = static_cast<int32_t>(
                    lroundf(frame.angle_error_deg10 -
                        delay_yaw_rad * (1800.0f / M_PI)));
                const int16_t latest_angle_deg10 = static_cast<int16_t>(
                    constrain(compensated_angle_deg10, -1800, 1800));

                if (state.camera_search_detection_pending &&
                    state.camera_phase != CameraPhase::SearchDetectionConfirm) {
                    // 探索中の走行画像は停止要求にだけ使う。
                    // 完全静止までは後続画像の角度・検出結果を制御へ混ぜない。
                } else if (frame.target_found != 0) {
                    // 低周期画像の遅れを抑えるため、最新90%・直前10%だけを混ぜる。
                    if (state.camera_angle_history_count > 0) {
                        const uint8_t previous_index =
                            (state.camera_angle_history_write_index +
                             CAMERA_ANGLE_HISTORY_SIZE - 1U) %
                            CAMERA_ANGLE_HISTORY_SIZE;
                        const int16_t previous_angle_deg10 =
                            state.camera_angle_history[previous_index];
                        const bool same_direction =
                            (latest_angle_deg10 >= 0) ==
                            (previous_angle_deg10 >= 0);
                        state.camera_filtered_angle_deg10 = same_direction
                            ? static_cast<int16_t>(
                                (9 * static_cast<int32_t>(latest_angle_deg10) +
                                 previous_angle_deg10) / 10)
                            : latest_angle_deg10;
                    } else {
                        state.camera_filtered_angle_deg10 = latest_angle_deg10;
                    }

                    state.camera_angle_history[state.camera_angle_history_write_index] =
                        latest_angle_deg10;
                    state.camera_angle_history_write_index =
                        (state.camera_angle_history_write_index + 1U) %
                        CAMERA_ANGLE_HISTORY_SIZE;
                    if (state.camera_angle_history_count < CAMERA_ANGLE_HISTORY_SIZE) {
                        ++state.camera_angle_history_count;
                    }

                    const bool goal_candidate =
                        frame.confidence >= CAMERA_GOAL_MIN_CONFIDENCE &&
                        frame.occupancy_permille >= CAMERA_GOAL_OCCUPANCY_PERMILLE &&
                        abs(state.camera_filtered_angle_deg10) <=
                            CAMERA_GOAL_MAX_ANGLE_DEG10;

                    const bool search_turn_moving =
                        state.camera_phase == CameraPhase::TurnDriving &&
                        (state.camera_turn_purpose ==
                             CameraTurnPurpose::SearchStep ||
                         state.camera_turn_purpose ==
                             CameraTurnPurpose::SearchCorner);
                    if (search_turn_moving) {
                        // 旋回中の1枚は角度制御には使わず、惰性停止の開始だけに使う。
                        state.camera_search_detection_pending = true;
                        resetSearchDetectionObservation();
                        state.camera_search_detection_rewind_used = false;
                        state.camera_search_detection_delay_turn_rad =
                            delay_yaw_rad;
                        state.camera_search_detection_turn_at_trigger_rad =
                            state.camera_accumulated_turn_rad;
                        state.camera_search_resume =
                            state.camera_turn_purpose ==
                                    CameraTurnPurpose::SearchStep
                                ? CameraSearchResume::SearchStep
                                : CameraSearchResume::SearchCorner;
                        state.camera_search_resume_turn_rad = 0.0f;
                        memset(
                            state.camera_angle_history,
                            0,
                            sizeof(state.camera_angle_history));
                        state.camera_angle_history_count = 0;
                        state.camera_angle_history_write_index = 0;
                        publishStop();
                        state.camera_phase = CameraPhase::TurnCoasting;
                        state.camera_still_started_ms = 0;
                        return current_output;
                    }

                    if (state.camera_phase == CameraPhase::SearchTranslate) {
                        // 探索直進中も、走行画像は停止要求としてだけ受理する。
                        state.camera_search_detection_pending = true;
                        resetSearchDetectionObservation();
                        state.camera_search_detection_rewind_used = false;
                        state.camera_search_detection_delay_turn_rad = 0.0f;
                        state.camera_search_detection_turn_at_trigger_rad = 0.0f;
                        state.camera_search_resume =
                            CameraSearchResume::Translation;
                        state.camera_search_resume_turn_rad = 0.0f;
                        memset(
                            state.camera_angle_history,
                            0,
                            sizeof(state.camera_angle_history));
                        state.camera_angle_history_count = 0;
                        state.camera_angle_history_write_index = 0;
                        startTranslationDeceleration(
                            CameraAfterStop::StartSearchDetectionConfirm);
                        return current_output;
                    }

                    if (state.camera_phase ==
                        CameraPhase::SearchDetectionConfirm) {
                        if (state.camera_search_detection_frame_count < UINT8_MAX) {
                            ++state.camera_search_detection_frame_count;
                        }

                        if (state.camera_search_detection_confirm_count == 0) {
                            state.camera_search_detection_first_angle_deg10 =
                                latest_angle_deg10;
                            state.camera_search_detection_confirm_count = 1;
                        } else if (abs(
                                       latest_angle_deg10 -
                                       state.camera_search_detection_first_angle_deg10) <=
                                   CAMERA_SEARCH_DETECTION_ANGLE_TOLERANCE_DEG10) {
                            ++state.camera_search_detection_confirm_count;
                        } else {
                            // 方向が大きく異なる検知は別候補として数え直す。
                            state.camera_search_detection_first_angle_deg10 =
                                latest_angle_deg10;
                            state.camera_search_detection_confirm_count = 1;
                        }

                        if (state.camera_search_detection_confirm_count >=
                            CAMERA_SEARCH_DETECTION_CONFIRM_FRAMES) {
                            // 操舵には静止後に一致した2枚の平均角度だけを使う。
                            state.camera_filtered_angle_deg10 =
                                static_cast<int16_t>(
                                    (static_cast<int32_t>(
                                         state.camera_search_detection_first_angle_deg10) +
                                     latest_angle_deg10) /
                                    2);

                            // 静止画像で確定した時点だけ、探索履歴を破棄する。
                            resetSearchProgress();
                            state.camera_phase = CameraPhase::WaitFrame;
                            state.camera_target_confirm_count = 0;
                        } else if (state.camera_search_detection_frame_count >=
                                   CAMERA_SEARCH_DETECTION_OBSERVE_FRAMES) {
                            publishStop();
                            finishSearchDetectionObservation();
                            return current_output;
                        } else {
                            publishStop();
                            return current_output;
                        }
                    }

                    if (state.camera_phase == CameraPhase::GoalConfirm) {
                        if (goal_candidate && camera_still_confirmed) {
                            if (state.camera_goal_detect_count <
                                CAMERA_GOAL_REQUIRED_FRAMES) {
                                ++state.camera_goal_detect_count;
                            }
                            if (state.camera_goal_detect_count >=
                                CAMERA_GOAL_REQUIRED_FRAMES) {
                                publishStop();
                                current_output.goal_reached = true;
                                state.camera_goal_detect_count = 0;
                                return current_output;
                            }
                        } else {
                            state.camera_goal_detect_count = 0;
                            state.camera_phase = CameraPhase::WaitFrame;
                        }
                    }

                    if (goal_candidate &&
                        state.camera_phase != CameraPhase::GoalConfirm) {
                        // 走行中の最初の候補は数えず、完全静止後の次フレームから数える。
                        if (state.camera_phase == CameraPhase::Forward ||
                            state.camera_phase == CameraPhase::SearchTranslate ||
                            state.camera_phase ==
                                CameraPhase::TranslationDecelerating) {
                            // 通常走行中のGOAL候補は急停止せず、制御基板の減速列を通す。
                            startTranslationDeceleration(
                                CameraAfterStop::StartGoalConfirm);
                        } else {
                            // 旋回中や既に停止中は並進減速の対象ではない。
                            publishStop();
                            state.camera_forward_command_mm_s = 0.0f;
                            state.camera_forward_command_ms = now_ms;
                            state.camera_phase = CameraPhase::StopSettling;
                            state.camera_after_stop =
                                CameraAfterStop::StartGoalConfirm;
                            state.camera_turn_purpose = CameraTurnPurpose::None;
                            state.camera_still_started_ms = 0;
                        }
                        state.camera_goal_detect_count = 0;
                        state.camera_target_confirm_count = 0;
                    } else if (state.camera_phase == CameraPhase::WaitFrame) {
                        const int16_t abs_angle =
                            abs(state.camera_filtered_angle_deg10);
                        if (frame.occupancy_permille >=
                                CAMERA_GOAL_OCCUPANCY_PERMILLE &&
                            frame.confidence < CAMERA_GOAL_MIN_CONFIDENCE) {
                            publishStop();
                            state.camera_target_confirm_count = 0;
                        } else if (abs_angle >
                                   CAMERA_FORWARD_DEAD_BAND_DEG10) {
                            const int8_t requested_direction =
                                state.camera_filtered_angle_deg10 >= 0 ? 1 : -1;
                            const bool reversing =
                                state.camera_last_target_turn_direction != 0 &&
                                requested_direction !=
                                    state.camera_last_target_turn_direction;
                            if (state.camera_target_reverse_used &&
                                abs_angle <=
                                    CAMERA_COARSE_FORWARD_DEAD_BAND_DEG10) {
                                // 逆補正後の小さな行き過ぎは、再旋回せず低速前進で吸収する。
                                state.camera_large_error_confirm_count = 0;
                                state.camera_large_error_direction = 0;
                                state.camera_coarse_forward_active = true;
                                if (state.camera_target_confirm_count <
                                    CAMERA_TARGET_CONFIRM_FRAMES) {
                                    ++state.camera_target_confirm_count;
                                }
                                if (state.camera_target_confirm_count >=
                                    CAMERA_TARGET_CONFIRM_FRAMES) {
                                    resetSearchProgress();
                                    state.camera_phase = CameraPhase::Forward;
                                }
                            } else if (state.camera_target_reverse_used) {
                                // 2回目の逆補正は即実行せず、同方向の停止画像2枚で再取得する。
                                publishStop();
                                state.camera_target_confirm_count = 0;
                                state.camera_coarse_forward_active = false;
                                if (state.camera_large_error_direction ==
                                    requested_direction) {
                                    if (state.camera_large_error_confirm_count <
                                        CAMERA_TARGET_CONFIRM_FRAMES) {
                                        ++state.camera_large_error_confirm_count;
                                    }
                                } else {
                                    state.camera_large_error_direction =
                                        requested_direction;
                                    state.camera_large_error_confirm_count = 1;
                                }
                                if (state.camera_large_error_confirm_count >=
                                    CAMERA_TARGET_CONFIRM_FRAMES) {
                                    // 補正履歴を解除し、この画像を新しい補正サイクルの開始にする。
                                    state.camera_last_target_turn_direction =
                                        requested_direction;
                                    state.camera_target_reverse_used = false;
                                    state.camera_large_error_confirm_count = 0;
                                    state.camera_large_error_direction = 0;
                                    const float requested_turn_rad =
                                        state.camera_filtered_angle_deg10 *
                                        (M_PI / 1800.0f);
                                    startCameraTurn(
                                        CameraTurnPurpose::Target,
                                        constrain(
                                            requested_turn_rad,
                                            -CAMERA_TARGET_MAX_TURN_RAD,
                                            CAMERA_TARGET_MAX_TURN_RAD));
                                }
                            } else {
                                state.camera_target_confirm_count = 0;
                                state.camera_large_error_confirm_count = 0;
                                state.camera_large_error_direction = 0;
                                state.camera_coarse_forward_active = false;
                                if (reversing) {
                                    state.camera_target_reverse_used = true;
                                }
                                state.camera_last_target_turn_direction =
                                    requested_direction;
                                const float requested_turn_rad =
                                    state.camera_filtered_angle_deg10 *
                                    (M_PI / 1800.0f);
                                startCameraTurn(
                                    CameraTurnPurpose::Target,
                                    constrain(
                                        requested_turn_rad,
                                        -CAMERA_TARGET_MAX_TURN_RAD,
                                        CAMERA_TARGET_MAX_TURN_RAD));
                            }
                        } else {
                            state.camera_last_target_turn_direction = 0;
                            state.camera_target_reverse_used = false;
                            state.camera_coarse_forward_active = false;
                            state.camera_large_error_confirm_count = 0;
                            state.camera_large_error_direction = 0;
                            if (state.camera_target_confirm_count <
                                CAMERA_TARGET_CONFIRM_FRAMES) {
                                ++state.camera_target_confirm_count;
                            }
                            if (state.camera_target_confirm_count >=
                                CAMERA_TARGET_CONFIRM_FRAMES) {
                                resetSearchProgress();
                                state.camera_phase = CameraPhase::Forward;
                            }
                        }
                    } else if (state.camera_phase == CameraPhase::Forward) {
                        if (state.camera_coarse_forward_active &&
                            abs(state.camera_filtered_angle_deg10) <=
                                CAMERA_FORWARD_DEAD_BAND_DEG10) {
                            // 低速前進中に正面へ戻ったら、次の制御周期から通常前進へ戻す。
                            state.camera_coarse_forward_active = false;
                            state.camera_last_target_turn_direction = 0;
                            state.camera_target_reverse_used = false;
                        }
                        const int16_t allowed_angle_deg10 =
                            state.camera_coarse_forward_active
                                ? CAMERA_COARSE_FORWARD_DEAD_BAND_DEG10
                                : CAMERA_FORWARD_DEAD_BAND_DEG10;
                        if (frame.occupancy_permille >=
                                CAMERA_GOAL_OCCUPANCY_PERMILLE) {
                            // GOAL付近の通常停止も同じ減速列を通し、前傾を抑える。
                            startTranslationDeceleration(
                                CameraAfterStop::WaitFrame);
                            state.camera_target_confirm_count = 0;
                        } else if (abs(state.camera_filtered_angle_deg10) >
                                   allowed_angle_deg10) {
                            // 通常の再撮影停止は急停止せず、短い減速を挟む。
                            startTranslationDeceleration(
                                CameraAfterStop::WaitFrame);
                            state.camera_target_confirm_count = 0;
                        }
                    } else if (
                        state.camera_phase == CameraPhase::SearchTranslate) {
                        startTranslationDeceleration(
                            CameraAfterStop::WaitFrame);
                    }
                } else {
                    if (state.camera_phase ==
                        CameraPhase::SearchDetectionConfirm) {
                        // 1枚の未検出では諦めず、最大6枚の観測窓を使う。
                        if (state.camera_search_detection_frame_count < UINT8_MAX) {
                            ++state.camera_search_detection_frame_count;
                        }
                        publishStop();
                        if (state.camera_search_detection_frame_count >=
                            CAMERA_SEARCH_DETECTION_OBSERVE_FRAMES) {
                            finishSearchDetectionObservation();
                        }
                        return current_output;
                    }

                    // 探索へ進むのは、通信の鮮度切れではなく明示的な未検出だけ。
                    state.camera_goal_detect_count = 0;
                    state.camera_target_confirm_count = 0;
                    state.camera_last_target_turn_direction = 0;
                    state.camera_target_reverse_used = false;
                    state.camera_coarse_forward_active = false;
                    state.camera_large_error_confirm_count = 0;
                    state.camera_large_error_direction = 0;
                    state.camera_search_requested = true;
                    if (state.camera_phase == CameraPhase::Forward ||
                        state.camera_phase == CameraPhase::SearchTranslate ||
                        state.camera_phase == CameraPhase::GoalConfirm) {
                        if (state.camera_phase == CameraPhase::GoalConfirm) {
                            publishStop();
                            state.camera_forward_command_mm_s = 0.0f;
                            state.camera_forward_command_ms = now_ms;
                            state.camera_phase = CameraPhase::StopSettling;
                            state.camera_after_stop = CameraAfterStop::StartSearch;
                            state.camera_turn_purpose = CameraTurnPurpose::None;
                            state.camera_still_started_ms = 0;
                        } else {
                            startTranslationDeceleration(
                                CameraAfterStop::StartSearch);
                        }
                    } else if (state.camera_phase == CameraPhase::WaitFrame) {
                        if (camera_still_confirmed) {
                            startCameraTurn(
                                CameraTurnPurpose::SearchStep,
                                CAMERA_SEARCH_STEP_RAD);
                            state.camera_search_requested = false;
                        } else {
                            state.camera_phase = CameraPhase::StopSettling;
                            state.camera_after_stop =
                                CameraAfterStop::StartSearch;
                            state.camera_still_started_ms = 0;
                        }
                    }
                }
            }

            const uint32_t camera_frame_age_ms =
                state.last_camera_frame_ms != 0
                    ? static_cast<uint32_t>(now_ms - state.last_camera_frame_ms)
                    : UINT32_MAX;

            // 500ms通信断では途中の制御判断を破棄し、復帰後の新規フレームを待つ。
            if (camera_frame_age_ms > CAMERA_COMM_TIMEOUT_MS) {
                publishStop();
                current_output.link_lost = true;
                state.camera_forward_command_mm_s = 0.0f;
                state.camera_forward_acceleration_mm_s2 = 0.0f;
                state.camera_forward_command_ms = now_ms;
                state.camera_phase = CameraPhase::LinkHold;
                state.camera_after_stop = CameraAfterStop::WaitFrame;
                state.camera_turn_purpose = CameraTurnPurpose::None;
                state.camera_turn_target_rad = 0.0f;
                state.camera_accumulated_turn_rad = 0.0f;
                state.camera_previous_gyro_timestamp_ms = 0;
                state.camera_target_confirm_count = 0;
                state.camera_goal_detect_count = 0;
                state.camera_last_target_turn_direction = 0;
                state.camera_target_reverse_used = false;
                state.camera_coarse_forward_active = false;
                state.camera_large_error_confirm_count = 0;
                state.camera_large_error_direction = 0;
                state.camera_search_requested = false;
                state.camera_search_detection_pending = false;
                state.camera_search_detection_frame_count = 0;
                state.camera_search_detection_confirm_count = 0;
                state.camera_search_detection_first_angle_deg10 = 0;
                state.camera_search_detection_rewind_used = false;
                state.camera_search_detection_delay_turn_rad = 0.0f;
                state.camera_search_detection_turn_at_trigger_rad = 0.0f;
                state.camera_search_resume = CameraSearchResume::None;
                state.camera_search_resume_turn_rad = 0.0f;
                return current_output;
            }

            // 400ms鮮度切れは移動だけを止める。未検出や探索開始とは解釈しない。
            if (camera_frame_age_ms > CAMERA_CONTROL_FRAME_TIMEOUT_MS &&
                (state.camera_phase == CameraPhase::TurnDriving ||
                 state.camera_phase == CameraPhase::Forward ||
                 state.camera_phase == CameraPhase::SearchTranslate ||
                 state.camera_phase == CameraPhase::TranslationDecelerating)) {
                publishStop();
                state.camera_forward_command_mm_s = 0.0f;
                state.camera_forward_acceleration_mm_s2 = 0.0f;
                state.camera_forward_command_ms = now_ms;
                state.camera_phase = CameraPhase::StopSettling;
                state.camera_after_stop = CameraAfterStop::WaitFrame;
                state.camera_turn_purpose = CameraTurnPurpose::None;
                state.camera_previous_gyro_timestamp_ms = 0;
                state.camera_still_started_ms = 0;
                state.camera_search_detection_pending = false;
                state.camera_search_detection_frame_count = 0;
                state.camera_search_detection_confirm_count = 0;
                state.camera_search_detection_first_angle_deg10 = 0;
                state.camera_search_detection_rewind_used = false;
                state.camera_search_detection_delay_turn_rad = 0.0f;
                state.camera_search_detection_turn_at_trigger_rad = 0.0f;
                state.camera_search_resume = CameraSearchResume::None;
                state.camera_search_resume_turn_rad = 0.0f;
            }

            if (state.camera_phase == CameraPhase::TurnDriving) {
                const float reached_rad =
                    state.camera_turn_purpose == CameraTurnPurpose::SearchStep
                        ? CAMERA_SEARCH_REACHED_RAD
                        : CAMERA_TURN_REACHED_RAD;
                const bool encoder_turn_usable =
                    camera_encoder_velocity_usable;
                float yaw_rate_rad_s = 0.0f;
                if (camera_gyro_usable) {
                    yaw_rate_rad_s = camera_gyro.z_rad_s;
                } else if (encoder_turn_usable) {
                    yaw_rate_rad_s =
                        (state.camera_right_velocity_mm_s -
                         state.camera_left_velocity_mm_s) /
                        CAMERA_TRACK_WIDTH_MM;
                } else {
                    const float direction =
                        state.camera_turn_target_rad >= 0.0f ? 1.0f : -1.0f;
                    state.camera_accumulated_turn_rad = direction *
                        CAMERA_TURN_OMEGA_RAD_S *
                        static_cast<float>(now_ms - state.camera_turn_started_ms) *
                        0.001f;
                }
                if (driveCameraTurn(
                        state.camera_turn_target_rad,
                        state.camera_accumulated_turn_rad,
                        yaw_rate_rad_s,
                        reached_rad)) {
                    state.camera_phase = CameraPhase::TurnCoasting;
                    state.camera_still_started_ms = 0;
                }
                return current_output;
            }

            if (state.camera_phase == CameraPhase::TurnCoasting) {
                publishStop();
                if (!camera_still_confirmed) return current_output;

                const CameraTurnPurpose completed_purpose =
                    state.camera_turn_purpose;
                const float completed_turn_rad =
                    state.camera_accumulated_turn_rad;
                const float completed_target_rad =
                    state.camera_turn_target_rad;
                state.camera_turn_purpose = CameraTurnPurpose::None;
                state.camera_previous_gyro_timestamp_ms = 0;
                state.camera_still_started_ms = 0;

                if (state.camera_search_detection_pending &&
                    (completed_purpose ==
                         CameraTurnPurpose::SearchStep ||
                     completed_purpose ==
                         CameraTurnPurpose::SearchCorner)) {
                    state.camera_search_detection_delay_turn_rad +=
                        completed_turn_rad -
                        state.camera_search_detection_turn_at_trigger_rad;
                    if (completed_purpose ==
                        CameraTurnPurpose::SearchStep) {
                        // 検知までに実際に回った角度も、一周探索の進捗へ残す。
                        state.camera_search_accumulated_rad +=
                            fabsf(completed_turn_rad);
                    }

                    float remaining_turn_rad =
                        completed_target_rad - completed_turn_rad;
                    if ((completed_target_rad >= 0.0f &&
                         remaining_turn_rad < 0.0f) ||
                        (completed_target_rad < 0.0f &&
                         remaining_turn_rad > 0.0f)) {
                        remaining_turn_rad = 0.0f;
                    }
                    state.camera_search_resume_turn_rad =
                        remaining_turn_rad;
                    resetSearchDetectionObservation();
                    state.camera_phase =
                        CameraPhase::SearchDetectionConfirm;
                    return current_output;
                }

                if (state.camera_search_detection_pending &&
                    completed_purpose ==
                        CameraTurnPurpose::SearchReacquire) {
                    // 巻き戻した実角度は探索進捗から差し引き、再開角度へ戻す。
                    if (state.camera_search_resume ==
                        CameraSearchResume::SearchStep) {
                        state.camera_search_accumulated_rad = fmaxf(
                            0.0f,
                            state.camera_search_accumulated_rad -
                                fabsf(completed_turn_rad));
                    }
                    state.camera_search_resume_turn_rad -= completed_turn_rad;
                    resetSearchDetectionObservation();
                    state.camera_phase =
                        CameraPhase::SearchDetectionConfirm;
                    return current_output;
                }

                if (completed_purpose == CameraTurnPurpose::SearchStep) {
                    state.camera_search_accumulated_rad +=
                        fabsf(completed_turn_rad);
                    completeSearchStep();
                } else if (
                    completed_purpose == CameraTurnPurpose::SearchCorner) {
                    startSearchTranslation();
                } else if (state.camera_search_requested) {
                    startCameraTurn(
                        CameraTurnPurpose::SearchStep,
                        CAMERA_SEARCH_STEP_RAD);
                    state.camera_search_requested = false;
                } else {
                    state.camera_phase = CameraPhase::WaitFrame;
                }
                return current_output;
            }

            if (state.camera_phase == CameraPhase::StopSettling) {
                publishStop();
                state.camera_forward_command_mm_s = 0.0f;
                state.camera_forward_acceleration_mm_s2 = 0.0f;
                state.camera_forward_command_ms = now_ms;
                if (!camera_still_confirmed) return current_output;

                const CameraAfterStop action = state.camera_after_stop;
                state.camera_after_stop = CameraAfterStop::WaitFrame;
                state.camera_still_started_ms = 0;
                if (action == CameraAfterStop::StartGoalConfirm) {
                    state.camera_goal_detect_count = 0;
                    state.camera_phase = CameraPhase::GoalConfirm;
                } else if (action ==
                           CameraAfterStop::StartSearchDetectionConfirm) {
                    resetSearchDetectionObservation();
                    state.camera_phase =
                        CameraPhase::SearchDetectionConfirm;
                } else if (action == CameraAfterStop::StartSearch) {
                    startCameraTurn(
                        CameraTurnPurpose::SearchStep,
                        CAMERA_SEARCH_STEP_RAD);
                    state.camera_search_requested = false;
                } else {
                    state.camera_phase = CameraPhase::WaitFrame;
                }
                return current_output;
            }

            if (state.camera_phase == CameraPhase::SearchTranslate) {
                float travelled_mm = 0.0f;
                bool translation_complete = false;
                if (camera_has_encoder && camera_encoder_velocity_usable) {
                    const float left_distance_mm = static_cast<float>(
                        camera_encoder.left_mm - state.camera_search_start_left_mm);
                    const float right_distance_mm = static_cast<float>(
                        camera_encoder.right_mm - state.camera_search_start_right_mm);
                    travelled_mm = fabsf(
                        0.5f * (left_distance_mm + right_distance_mm));
                    translation_complete =
                        travelled_mm >= state.camera_search_target_distance_mm;
                } else {
                    translation_complete = static_cast<uint32_t>(
                        now_ms - state.camera_search_translate_started_ms) >=
                        state.camera_search_translate_duration_ms;
                }

                if (translation_complete) {
                    if (state.camera_search_leg_index < UINT8_MAX) {
                        ++state.camera_search_leg_index;
                    }
                    state.camera_search_accumulated_rad = 0.0f;
                    startTranslationDeceleration(
                        CameraAfterStop::WaitFrame);
                } else {
                    state.camera_forward_command_mm_s =
                        CAMERA_SEARCH_MOVE_SPEED_MM_S;
                    state.camera_forward_acceleration_mm_s2 = 0.0f;
                    state.camera_forward_command_ms = now_ms;
                    publishJog(
                        state.camera_forward_command_mm_s,
                        0.0f,
                        CAMERA_CONTROL_COMMAND_MS);
                }
                return current_output;
            }

            if (state.camera_phase == CameraPhase::TranslationDecelerating) {
                updateCameraForwardSpeed(0.0f);

                if (state.camera_forward_command_mm_s <=
                        CAMERA_FORWARD_DECEL_STOP_SPEED_MM_S &&
                    fabsf(state.camera_forward_acceleration_mm_s2) <=
                        CAMERA_FORWARD_DECEL_STOP_ACCEL_MM_S2) {
                    state.camera_forward_command_mm_s = 0.0f;
                    state.camera_forward_acceleration_mm_s2 = 0.0f;
                    publishStop();
                    state.camera_phase = CameraPhase::StopSettling;
                    state.camera_still_started_ms = 0;
                } else {
                    publishJog(
                        state.camera_forward_command_mm_s,
                        0.0f,
                        CAMERA_CONTROL_COMMAND_MS);
                }
                return current_output;
            }

            if (state.camera_phase == CameraPhase::Forward) {
                if (!has_frame || frame.target_found == 0) {
                    publishStop();
                    state.camera_forward_command_mm_s = 0.0f;
                    state.camera_forward_command_ms = now_ms;
                } else {
                    const float target_forward_speed = state.camera_coarse_forward_active
                        ? CAMERA_NEAR_FORWARD_SPEED_MM_S
                        : (frame.occupancy_permille <
                            CAMERA_NEAR_OCCUPANCY_PERMILLE
                            ? CAMERA_FAR_FORWARD_SPEED_MM_S
                            : CAMERA_NEAR_FORWARD_SPEED_MM_S);

                    // 加速は即時、800->450など速度を下げる側だけ減速率を適用する。
                    if (state.camera_forward_command_mm_s > target_forward_speed) {
                        updateCameraForwardSpeed(target_forward_speed);
                    } else {
                        state.camera_forward_command_mm_s = target_forward_speed;
                        state.camera_forward_acceleration_mm_s2 = 0.0f;
                        state.camera_forward_command_ms = now_ms;
                    }
                    publishJog(
                        state.camera_forward_command_mm_s,
                        0.0f,
                        CAMERA_CONTROL_COMMAND_MS);
                }
                return current_output;
            }

            // WaitFrame・GoalConfirm・LinkHoldは、新規フレームによる遷移まで停止する。
            publishStop();
            return current_output;

    return current_output;
}

} // namespace CameraNavigation
