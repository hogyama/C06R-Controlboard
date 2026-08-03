#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>

namespace PurePursuit
{
    static constexpr float PI_RAD = 3.14159265358979323846f;

    struct Point
    {
        float x_mm;
        float y_mm;
    };

    struct Pose
    {
        float x_mm;
        float y_mm;
        float yaw_rad;
    };

    struct Config
    {
        // 通常速度 [mm/s]
        float base_speed_mm_s = 300.0f;

        // 最小速度 [mm/s]
        float min_speed_mm_s = 80.0f;

        // 最大速度 [mm/s]
        float max_speed_mm_s = 600.0f;

        // 左右タイヤ間距離 [mm]
        float wheel_base_mm = 180.0f;

        // 経路上で何mm先を見るか
        float lookahead_distance_mm = 500.0f;

        // ゴール到達判定距離 [mm]
        float goal_tolerance_mm = 300.0f;

        // 経路点探索を何点戻ることまで許すか
        // 基本は0でよい
        uint16_t backtrack_allow_count = 0;

        // 旋回時の減速係数
        float turn_slow_gain = 1.5f;

        // 最大角速度 [rad/s]
        float max_turn_rate_rad_s = 4.0f;
    };

    struct PathState
    {
        // 前回選んだ最近傍経路点
        uint16_t nearest_index = 0;

        // 前回選んだlookahead点
        uint16_t target_index = 0;

        bool initialized = false;
    };

    struct Output
    {
        Point target;

        uint16_t nearest_index;
        uint16_t target_index;

        float distance_to_goal_mm;
        float distance_to_target_mm;

        float target_angle_rad;
        float relative_angle_rad;

        float curvature_1_per_mm;

        float linear_velocity_mm_s;
        float angular_velocity_rad_s;

        float left_velocity_mm_s;
        float right_velocity_mm_s;

        bool reached_goal;
        bool valid;
    };

    // ============================================================
    // Utility
    // ============================================================

    inline float normalizeAngleRad(float angle)
    {
        while (angle > PI_RAD)
        {
            angle -= 2.0f * PI_RAD;
        }

        while (angle < -PI_RAD)
        {
            angle += 2.0f * PI_RAD;
        }

        return angle;
    }

    inline float clamp(float value, float min_value, float max_value)
    {
        if (value < min_value)
        {
            return min_value;
        }

        if (value > max_value)
        {
            return max_value;
        }

        return value;
    }

    inline float clampAbs(float value, float max_abs)
    {
        return clamp(value, -max_abs, max_abs);
    }

    inline float distanceMm(float x1, float y1, float x2, float y2)
    {
        const float dx = x2 - x1;
        const float dy = y2 - y1;
        return sqrtf(dx * dx + dy * dy);
    }

    inline float distanceMm(const Pose& pose, const Point& p)
    {
        return distanceMm(pose.x_mm, pose.y_mm, p.x_mm, p.y_mm);
    }

    inline float distanceMm(const Point& a, const Point& b)
    {
        return distanceMm(a.x_mm, a.y_mm, b.x_mm, b.y_mm);
    }

    // ============================================================
    // 現在位置に一番近い経路点を探す
    // ============================================================

    inline size_t findNearestPathIndex(
        const Pose& pose,
        const Point* path,
        size_t path_count,
        const PathState& state,
        const Config& config
    )
    {
        if (path == nullptr || path_count == 0)
        {
            return 0;
        }

        size_t start_index = 0;

        if (state.initialized)
        {
            if (state.nearest_index > config.backtrack_allow_count)
            {
                start_index = state.nearest_index - config.backtrack_allow_count;
            }
            else
            {
                start_index = 0;
            }
        }

        float best_distance = distanceMm(pose, path[start_index]);
        size_t best_index = start_index;

        for (size_t i = start_index + 1; i < path_count; i++)
        {
            const float d = distanceMm(pose, path[i]);

            if (d < best_distance)
            {
                best_distance = d;
                best_index = i;
            }
        }

        return best_index;
    }

    // ============================================================
    // lookahead点を選ぶ
    // ============================================================

    inline size_t selectLookaheadIndex(
        const Pose& pose,
        const Point* path,
        size_t path_count,
        size_t nearest_index,
        const Config& config
    )
    {
        if (path == nullptr || path_count == 0)
        {
            return 0;
        }

        if (nearest_index >= path_count)
        {
            return path_count - 1;
        }

        // 経路上をnearest_indexから先へ進み、
        // 累積距離がlookahead_distance_mmを超えた点を選ぶ
        float accumulated_distance = distanceMm(pose, path[nearest_index]);

        for (size_t i = nearest_index; i + 1 < path_count; i++)
        {
            accumulated_distance += distanceMm(path[i], path[i + 1]);

            if (accumulated_distance >= config.lookahead_distance_mm)
            {
                return i + 1;
            }
        }

        // 終端までlookahead距離が足りなければゴール点を返す
        return path_count - 1;
    }

    // ============================================================
    // Pure Pursuit本体
    // ============================================================

    inline Output calculateToTarget(
        const Pose& pose,
        const Point& target,
        const Point& goal,
        const Config& config
    )
    {
        Output out = {};

        out.target = target;

        const float dx = target.x_mm - pose.x_mm;
        const float dy = target.y_mm - pose.y_mm;

        const float distance_to_target = sqrtf(dx * dx + dy * dy);
        const float distance_to_goal = distanceMm(pose.x_mm, pose.y_mm, goal.x_mm, goal.y_mm);

        out.distance_to_target_mm = distance_to_target;
        out.distance_to_goal_mm = distance_to_goal;

        if (distance_to_goal <= config.goal_tolerance_mm)
        {
            out.reached_goal = true;
            out.valid = true;

            out.linear_velocity_mm_s = 0.0f;
            out.angular_velocity_rad_s = 0.0f;
            out.left_velocity_mm_s = 0.0f;
            out.right_velocity_mm_s = 0.0f;

            return out;
        }

        if (distance_to_target < 1.0f)
        {
            out.reached_goal = false;
            out.valid = false;

            out.linear_velocity_mm_s = 0.0f;
            out.angular_velocity_rad_s = 0.0f;
            out.left_velocity_mm_s = 0.0f;
            out.right_velocity_mm_s = 0.0f;

            return out;
        }

        const float target_angle = atan2f(dy, dx);
        const float relative_angle = normalizeAngleRad(target_angle - pose.yaw_rad);

        out.target_angle_rad = target_angle;
        out.relative_angle_rad = relative_angle;

        // Pure Pursuit曲率
        //
        // curvature = 2 * sin(alpha) / Ld
        //
        // alpha: 現在方位から見た目標点の相対角
        // Ld: lookahead点までの距離
        const float curvature = 2.0f * sinf(relative_angle) / distance_to_target;

        out.curvature_1_per_mm = curvature;

        float speed = config.base_speed_mm_s;

        // 旋回角が大きいほど速度を落とす
        speed /= 1.0f + config.turn_slow_gain * fabsf(relative_angle);

        speed = clamp(
            speed,
            config.min_speed_mm_s,
            config.max_speed_mm_s
        );

        // 目標点が真横〜後方にあるときは低速化
        if (fabsf(relative_angle) > PI_RAD * 0.5f)
        {
            speed = config.min_speed_mm_s;
        }

        float omega = speed * curvature;

        omega = clampAbs(
            omega,
            config.max_turn_rate_rad_s
        );

        out.linear_velocity_mm_s = speed;
        out.angular_velocity_rad_s = omega;

        const float half_base = config.wheel_base_mm * 0.5f;

        float left = speed - omega * half_base;
        float right = speed + omega * half_base;

        left = clamp(left, -config.max_speed_mm_s, config.max_speed_mm_s);
        right = clamp(right, -config.max_speed_mm_s, config.max_speed_mm_s);

        out.left_velocity_mm_s = left;
        out.right_velocity_mm_s = right;

        out.reached_goal = false;
        out.valid = true;

        return out;
    }

    // ============================================================
    // A*経路追従用 calculate
    // ============================================================

    inline Output calculatePathFollowing(
        const Pose& pose,
        const Point* path,
        size_t path_count,
        PathState& state,
        const Config& config
    )
    {
        Output out = {};

        if (path == nullptr || path_count == 0)
        {
            out.valid = false;
            out.reached_goal = false;
            return out;
        }
        if (path_count > UINT16_MAX)
        {
            path_count = UINT16_MAX;
        }

        const Point& goal = path[path_count - 1];

        const size_t nearest_index = findNearestPathIndex(
            pose,
            path,
            path_count,
            state,
            config
        );

        const size_t target_index = selectLookaheadIndex(
            pose,
            path,
            path_count,
            nearest_index,
            config
        );

        state.nearest_index = static_cast<uint16_t>(nearest_index);
        state.target_index = static_cast<uint16_t>(target_index);
        state.initialized = true;

        out = calculateToTarget(
            pose,
            path[target_index],
            goal,
            config
        );

        out.nearest_index = static_cast<uint16_t>(nearest_index);
        out.target_index = static_cast<uint16_t>(target_index);

        return out;
    }
} // namespace PurePursuit
