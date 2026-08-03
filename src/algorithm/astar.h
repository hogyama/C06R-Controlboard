#pragma once

#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include "pure_pursuit.h"

namespace AStar
{

static constexpr uint8_t MAP_W = 64;
static constexpr uint8_t MAP_H = 64;
static constexpr uint16_t CELL_COUNT = MAP_W * MAP_H;
static constexpr uint8_t CELL_BITS = 4;
static constexpr uint8_t CELLS_PER_BYTE = 8U / CELL_BITS;
static constexpr uint16_t MAP_BYTES = CELL_COUNT / CELLS_PER_BYTE;

// ============================================================
// Cell value: 4bit/cell.
// 0 is repeatedly confirmed free, 7 is unknown, and 15 is blocked.
// Values 8-14 remain traversable with an increasing A* penalty.
// ============================================================

enum CellValue : uint8_t
{
    CELL_FREE    = 0,
    CELL_VISITED = 2,
    CELL_UNKNOWN = 7,
    CELL_BLOCKED = 15
};

struct GridPos
{
    uint8_t x;
    uint8_t y;
};

struct Config
{
    float cell_size_mm = 1000.0f;

    // GPSゴール(0, 0)を64x64地図の中央セル中心へ配置する。
    float origin_x_mm = -2000.0f;
    float origin_y_mm = -2000.0f;

    bool allow_unknown = true;
    bool allow_diagonal = true;
    // Emergency planning only: treat value 15 as traversable with the
    // same cost as value 14. The packed map itself is not modified.
    bool allow_blocked_as_high_cost = false;

    bool simplify_path = true;

    // UNKNOWNを少し避けたい場合の追加コスト
    uint16_t unknown_extra_cost = 5;

};

struct Result
{
    bool found = false;
    bool start_blocked = false;
    bool goal_blocked = false;
    bool path_overflow = false;

    size_t path_count = 0;
    uint16_t searched_count = 0;
};

// ============================================================
// Memory-saving workspace
// ------------------------------------------------------------
// g_cost:
//   各セルの開始地点からのコスト
//
// parent_dir:
//   親セルがどの方向にあるかを保存
//   親indexをint16_tで持たず、方向だけ持つ
//
// state:
//   2bit/cell
//   0: none
//   1: open
//   2: closed
//
// open_list:
//   open中のセルindexだけを保存
// ============================================================

struct Work
{
    uint16_t g_cost[CELL_COUNT];              // 8192 byte
    uint8_t parent_dir[CELL_COUNT / 2];       // 2048 byte, 4 bit/cell
    uint8_t state[CELL_COUNT / 4];            // 1024 byte, 2 bit/cell
    uint16_t open_list[CELL_COUNT];           // 8192 byte
    uint16_t open_count = 0;
};

static_assert(MAP_BYTES == 2048U, "64x64 4-bit map must use 2048 bytes");
static_assert(sizeof(Work) <= 19460U,
              "A* work area must retain packed memory layout");

static constexpr uint8_t STATE_NONE   = 0;
static constexpr uint8_t STATE_OPEN   = 1;
static constexpr uint8_t STATE_CLOSED = 2;

static constexpr uint8_t PARENT_NONE = 0x0f;

// dir index
// 0: +x
// 1: -x
// 2: +y
// 3: -y
// 4: +x +y
// 5: +x -y
// 6: -x +y
// 7: -x -y

static constexpr int8_t DIR8[8][2] =
{
    {  1,  0 },
    { -1,  0 },
    {  0,  1 },
    {  0, -1 },
    {  1,  1 },
    {  1, -1 },
    { -1,  1 },
    { -1, -1 }
};

// 子へ進んだ方向dirに対して、
// 子から見た親の方向を返す
inline uint8_t oppositeDir(uint8_t dir)
{
    switch (dir)
    {
        case 0: return 1;
        case 1: return 0;
        case 2: return 3;
        case 3: return 2;
        case 4: return 7;
        case 5: return 6;
        case 6: return 5;
        case 7: return 4;
        default: return PARENT_NONE;
    }
}

// ============================================================
// Basic utility
// ============================================================

inline bool inBounds(int16_t x, int16_t y)
{
    return x >= 0 && x < MAP_W && y >= 0 && y < MAP_H;
}

inline uint16_t toIndex(uint8_t x, uint8_t y)
{
    return static_cast<uint16_t>(y) * MAP_W + x;
}

inline GridPos toGridPos(uint16_t index)
{
    GridPos p;
    p.x = static_cast<uint8_t>(index % MAP_W);
    p.y = static_cast<uint8_t>(index / MAP_W);
    return p;
}

// ============================================================
// 4bit packed map access
// ============================================================

inline uint8_t getCell(const uint8_t* map, uint8_t x, uint8_t y)
{
    if (map == nullptr)
    {
        return CELL_BLOCKED;
    }

    if (!inBounds(x, y))
    {
        return CELL_BLOCKED;
    }

    const uint16_t index = toIndex(x, y);
    const uint16_t byte_index = index >> 1;
    if ((index & 1U) == 0U)
    {
        return static_cast<uint8_t>(map[byte_index] & 0x0fU);
    }
    return static_cast<uint8_t>((map[byte_index] >> 4) & 0x0fU);
}

inline void setCell(uint8_t* map, uint8_t x, uint8_t y, uint8_t value)
{
    if (map == nullptr)
    {
        return;
    }

    if (!inBounds(x, y))
    {
        return;
    }

    const uint16_t index = toIndex(x, y);
    const uint16_t byte_index = index >> 1;
    value &= 0x0fU;
    if ((index & 1U) == 0U)
    {
        map[byte_index] = static_cast<uint8_t>(
            (map[byte_index] & 0xf0U) | value);
    }
    else
    {
        map[byte_index] = static_cast<uint8_t>(
            (map[byte_index] & 0x0fU) | (value << 4));
    }
}

inline uint8_t adjustCell(
    uint8_t* map,
    uint8_t x,
    uint8_t y,
    int8_t delta,
    uint8_t maximum_value = CELL_BLOCKED)
{
    if (map == nullptr || !inBounds(x, y))
    {
        return CELL_BLOCKED;
    }

    const uint8_t old_value = getCell(map, x, y);
    int16_t adjusted = static_cast<int16_t>(old_value) + delta;
    if (adjusted < CELL_FREE) adjusted = CELL_FREE;
    uint8_t effective_maximum = maximum_value;
    if (delta > 0 && old_value > effective_maximum)
    {
        effective_maximum = old_value;
    }
    if (adjusted > effective_maximum) adjusted = effective_maximum;
    if (adjusted > CELL_BLOCKED) adjusted = CELL_BLOCKED;
    const uint8_t new_value = static_cast<uint8_t>(adjusted);
    setCell(map, x, y, new_value);
    return new_value;
}

inline bool applyEvidenceKernel(
    uint8_t* map,
    GridPos center,
    int8_t center_delta,
    uint8_t radius_cells,
    uint8_t maximum_value = CELL_BLOCKED)
{
    if (map == nullptr || !inBounds(center.x, center.y) ||
        center_delta == 0)
    {
        return false;
    }

    bool changed = false;
    const int16_t radius = radius_cells;
    for (int16_t dy = -radius; dy <= radius; ++dy)
    {
        for (int16_t dx = -radius; dx <= radius; ++dx)
        {
            const int16_t x = static_cast<int16_t>(center.x) + dx;
            const int16_t y = static_cast<int16_t>(center.y) + dy;
            if (!inBounds(x, y)) continue;

            const uint8_t distance = static_cast<uint8_t>(
                (dx < 0 ? -dx : dx) + (dy < 0 ? -dy : dy));
            int8_t delta = center_delta;
            if (distance > 0U)
            {
                const int16_t magnitude =
                    center_delta < 0 ? -center_delta : center_delta;
                const int16_t scaled = magnitude / (distance + 1U);
                if (scaled == 0) continue;
                delta = static_cast<int8_t>(
                    center_delta < 0 ? -scaled : scaled);
            }

            const uint8_t ux = static_cast<uint8_t>(x);
            const uint8_t uy = static_cast<uint8_t>(y);
            const uint8_t old_value = getCell(map, ux, uy);
            const uint8_t new_value =
                adjustCell(map, ux, uy, delta, maximum_value);
            changed = changed || new_value != old_value;
        }
    }
    return changed;
}

inline bool isWalkableCellValue(uint8_t value, const Config& config)
{
    if (value == CELL_BLOCKED &&
        !config.allow_blocked_as_high_cost)
    {
        return false;
    }

    if (value == CELL_UNKNOWN && !config.allow_unknown)
    {
        return false;
    }

    return true;
}

inline bool isWalkable(const uint8_t* map, uint8_t x, uint8_t y, const Config& config)
{
    return isWalkableCellValue(getCell(map, x, y), config);
}

// ============================================================
// 4bit parent-direction access
// ============================================================

inline uint8_t getParent(const Work& work, uint16_t index)
{
    const uint16_t byte_index = index >> 1;
    if ((index & 1U) == 0U)
    {
        return static_cast<uint8_t>(work.parent_dir[byte_index] & 0x0fU);
    }
    return static_cast<uint8_t>((work.parent_dir[byte_index] >> 4) & 0x0fU);
}

inline void setParent(Work& work, uint16_t index, uint8_t value)
{
    const uint16_t byte_index = index >> 1;
    value &= 0x0fU;
    if ((index & 1U) == 0U)
    {
        work.parent_dir[byte_index] = static_cast<uint8_t>(
            (work.parent_dir[byte_index] & 0xf0U) | value);
    }
    else
    {
        work.parent_dir[byte_index] = static_cast<uint8_t>(
            (work.parent_dir[byte_index] & 0x0fU) | (value << 4));
    }
}

// ============================================================
// 2bit search-state access
// ============================================================

inline uint8_t getState(const Work& work, uint16_t index)
{
    const uint16_t byte_index = index >> 2;
    const uint8_t shift = static_cast<uint8_t>((index & 0x03U) * 2U);
    return static_cast<uint8_t>((work.state[byte_index] >> shift) & 0x03U);
}

inline void setState(Work& work, uint16_t index, uint8_t value)
{
    const uint16_t byte_index = index >> 2;
    const uint8_t shift = static_cast<uint8_t>((index & 0x03U) * 2U);
    const uint8_t mask = static_cast<uint8_t>(0x03U << shift);
    work.state[byte_index] = static_cast<uint8_t>(
        (work.state[byte_index] & ~mask) |
        ((value & 0x03U) << shift));
}

// ============================================================
// World/Grid conversion
// ============================================================

inline GridPos worldToGrid(float x_mm, float y_mm, const Config& config)
{
    int16_t gx = static_cast<int16_t>(floorf((x_mm - config.origin_x_mm) / config.cell_size_mm));
    int16_t gy = static_cast<int16_t>(floorf((y_mm - config.origin_y_mm) / config.cell_size_mm));

    if (gx < 0)
    {
        gx = 0;
    }
    else if (gx >= MAP_W)
    {
        gx = MAP_W - 1;
    }

    if (gy < 0)
    {
        gy = 0;
    }
    else if (gy >= MAP_H)
    {
        gy = MAP_H - 1;
    }

    GridPos p;
    p.x = static_cast<uint8_t>(gx);
    p.y = static_cast<uint8_t>(gy);
    return p;
}

// A*では範囲外を端セルへ丸めず、経路生成失敗として扱う。
inline bool worldToGridChecked(
    float x_mm,
    float y_mm,
    GridPos& out,
    const Config& config
)
{
    const int16_t gx = static_cast<int16_t>(floorf((x_mm - config.origin_x_mm) / config.cell_size_mm));
    const int16_t gy = static_cast<int16_t>(floorf((y_mm - config.origin_y_mm) / config.cell_size_mm));

    if (!inBounds(gx, gy))
    {
        return false;
    }

    out.x = static_cast<uint8_t>(gx);
    out.y = static_cast<uint8_t>(gy);
    return true;
}

inline PurePursuit::Point gridToWorldCenter(GridPos p, const Config& config)
{
    PurePursuit::Point out;
    out.x_mm = config.origin_x_mm + (static_cast<float>(p.x) + 0.5f) * config.cell_size_mm;
    out.y_mm = config.origin_y_mm + (static_cast<float>(p.y) + 0.5f) * config.cell_size_mm;
    return out;
}

// ============================================================
// Heuristic
// cost:
//   straight = 10
//   diagonal = 14
// ============================================================

inline uint16_t heuristic(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, bool diagonal)
{
    const uint16_t dx = static_cast<uint16_t>((x0 > x1) ? (x0 - x1) : (x1 - x0));
    const uint16_t dy = static_cast<uint16_t>((y0 > y1) ? (y0 - y1) : (y1 - y0));

    if (diagonal)
    {
        const uint16_t dmin = (dx < dy) ? dx : dy;
        const uint16_t dmax = (dx > dy) ? dx : dy;

        return static_cast<uint16_t>(14 * dmin + 10 * (dmax - dmin));
    }

    return static_cast<uint16_t>(10 * (dx + dy));
}

inline uint16_t cellTraversalExtraCost(
    uint8_t value,
    const Config& config)
{
    value &= 0x0fU;
    if (value <= 3U) return 0U;
    if (value == 4U) return 1U;
    if (value == 5U) return 2U;
    if (value == 6U) return 3U;
    if (value == CELL_UNKNOWN) return config.unknown_extra_cost;
    if (value == 8U) return 10U;
    if (value == 9U) return 15U;
    if (value == 10U) return 22U;
    if (value == 11U) return 31U;
    if (value == 12U) return 42U;
    if (value == 13U) return 55U;
    if (value == 14U ||
        (value == CELL_BLOCKED &&
         config.allow_blocked_as_high_cost)) {
        return 70U;
    }
    return UINT16_MAX;
}

inline uint16_t saturatingCostAdd(uint16_t left, uint16_t right)
{
    if (left > static_cast<uint16_t>(UINT16_MAX - right))
    {
        return UINT16_MAX;
    }
    return static_cast<uint16_t>(left + right);
}

inline uint16_t calcF(
    const Work& work,
    uint16_t index,
    GridPos goal,
    const Config& config
)
{
    const GridPos p = toGridPos(index);
    return saturatingCostAdd(
        work.g_cost[index],
        heuristic(p.x, p.y, goal.x, goal.y, config.allow_diagonal));
}

// ============================================================
// Work reset
// ============================================================

inline void resetWork(Work& work)
{
    for (uint16_t i = 0; i < CELL_COUNT; i++)
    {
        work.g_cost[i] = 0xffff;
    }

    for (uint16_t i = 0; i < CELL_COUNT / 2; i++)
    {
        work.parent_dir[i] = 0xff;
    }

    for (uint16_t i = 0; i < CELL_COUNT / 4; i++)
    {
        work.state[i] = 0;
    }

    work.open_count = 0;
}

// ============================================================
// Open list
// ------------------------------------------------------------
// open_listにはindexだけを入れる。
// bestを取り出すときに線形探索。
// 64x64ならこれで十分。
// ============================================================

inline bool pushOpen(Work& work, uint16_t index)
{
    if (work.open_count >= CELL_COUNT)
    {
        return false;
    }

    work.open_list[work.open_count++] = index;
    return true;
}

inline int16_t popBestOpen(Work& work, GridPos goal, const Config& config)
{
    if (work.open_count == 0)
    {
        return -1;
    }

    uint16_t best_pos = 0;
    uint16_t best_index = work.open_list[0];
    uint16_t best_f = calcF(work, best_index, goal, config);

    for (uint16_t i = 1; i < work.open_count; i++)
    {
        const uint16_t index = work.open_list[i];
        const uint16_t f = calcF(work, index, goal, config);

        if (f < best_f)
        {
            best_f = f;
            best_index = index;
            best_pos = i;
        }
        else if (f == best_f)
        {
            const GridPos a = toGridPos(index);
            const GridPos b = toGridPos(best_index);

            const uint16_t ha = heuristic(a.x, a.y, goal.x, goal.y, config.allow_diagonal);
            const uint16_t hb = heuristic(b.x, b.y, goal.x, goal.y, config.allow_diagonal);

            if (ha < hb)
            {
                best_f = f;
                best_index = index;
                best_pos = i;
            }
        }
    }

    work.open_count--;

    work.open_list[best_pos] = work.open_list[work.open_count];

    return static_cast<int16_t>(best_index);
}

// ============================================================
// Diagonal corner check
// ============================================================

inline bool canMoveDiagonal(
    const uint8_t* map,
    uint8_t x,
    uint8_t y,
    int8_t dx,
    int8_t dy,
    const Config& config
)
{
    if (dx == 0 || dy == 0)
    {
        return true;
    }

    const int16_t x1 = static_cast<int16_t>(x) + dx;
    const int16_t y1 = static_cast<int16_t>(y);

    const int16_t x2 = static_cast<int16_t>(x);
    const int16_t y2 = static_cast<int16_t>(y) + dy;

    if (!inBounds(x1, y1) || !inBounds(x2, y2))
    {
        return false;
    }

    return isWalkable(map, static_cast<uint8_t>(x1), static_cast<uint8_t>(y1), config) &&
           isWalkable(map, static_cast<uint8_t>(x2), static_cast<uint8_t>(y2), config);
}

// ============================================================
// Reconstruct path
// ------------------------------------------------------------
// parent_dirだけから経路を復元する。
// out_pathにはstart -> goal順で入る。
// ============================================================

inline size_t reconstructPathGrid(
    const Work& work,
    GridPos start,
    GridPos goal,
    GridPos* out_path,
    size_t out_path_max,
    bool* overflow
)
{
    if (overflow != nullptr)
    {
        *overflow = false;
    }

    if (out_path == nullptr || out_path_max == 0)
    {
        if (overflow != nullptr)
        {
            *overflow = true;
        }

        return 0;
    }

    size_t count = 0;

    GridPos current = goal;

    while (true)
    {
        if (count >= out_path_max)
        {
            if (overflow != nullptr)
            {
                *overflow = true;
            }

            break;
        }

        out_path[count++] = current;

        if (current.x == start.x && current.y == start.y)
        {
            break;
        }

        const uint16_t index = toIndex(current.x, current.y);
        const uint8_t dir = getParent(work, index);

        if (dir == PARENT_NONE || dir >= 8)
        {
            break;
        }

        current.x = static_cast<uint8_t>(static_cast<int16_t>(current.x) + DIR8[dir][0]);
        current.y = static_cast<uint8_t>(static_cast<int16_t>(current.y) + DIR8[dir][1]);
    }

    for (size_t i = 0; i < count / 2; i++)
    {
        GridPos tmp = out_path[i];
        out_path[i] = out_path[count - 1 - i];
        out_path[count - 1 - i] = tmp;
    }

    return count;
}

// ============================================================
// Simplify path
// ------------------------------------------------------------
// 同じ方向に進む中間点を削る。
// Pure Pursuitに渡すなら有効でよい。
// ============================================================

inline size_t simplifyPathGrid(GridPos* path, size_t path_count)
{
    if (path == nullptr)
    {
        return 0;
    }

    if (path_count <= 2)
    {
        return path_count;
    }

    size_t write_index = 1;

    int8_t prev_dx = static_cast<int8_t>(path[1].x) - static_cast<int8_t>(path[0].x);
    int8_t prev_dy = static_cast<int8_t>(path[1].y) - static_cast<int8_t>(path[0].y);

    for (size_t i = 2; i < path_count; i++)
    {
        const int8_t dx = static_cast<int8_t>(path[i].x) - static_cast<int8_t>(path[i - 1].x);
        const int8_t dy = static_cast<int8_t>(path[i].y) - static_cast<int8_t>(path[i - 1].y);

        if (dx != prev_dx || dy != prev_dy)
        {
            path[write_index++] = path[i - 1];
            prev_dx = dx;
            prev_dy = dy;
        }
    }

    path[write_index++] = path[path_count - 1];

    return write_index;
}

// ============================================================
// Convert to PurePursuit path
// ============================================================

inline size_t convertGridPathToPurePursuitPath(
    const GridPos* grid_path,
    size_t grid_path_count,
    PurePursuit::Point* pp_path,
    size_t pp_path_max,
    const Config& config
)
{
    if (grid_path == nullptr || pp_path == nullptr || pp_path_max == 0)
    {
        return 0;
    }

    size_t count = grid_path_count;

    if (count > pp_path_max)
    {
        count = pp_path_max;
    }

    for (size_t i = 0; i < count; i++)
    {
        pp_path[i] = gridToWorldCenter(grid_path[i], config);
    }

    return count;
}

// ============================================================
// A* main: grid path output
// ============================================================

inline Result findPathGrid(
    const uint8_t* map,
    GridPos start,
    GridPos goal,
    GridPos* out_path,
    size_t out_path_max,
    Work& work,
    const Config& config
)
{
    Result result;

    if (map == nullptr || out_path == nullptr || out_path_max == 0)
    {
        return result;
    }

    if (!inBounds(start.x, start.y) || !inBounds(goal.x, goal.y))
    {
        return result;
    }

    if (!isWalkable(map, start.x, start.y, config))
    {
        result.start_blocked = true;
        return result;
    }

    if (!isWalkable(map, goal.x, goal.y, config))
    {
        result.goal_blocked = true;
        return result;
    }

    resetWork(work);

    const uint16_t start_index = toIndex(start.x, start.y);
    const uint16_t goal_index = toIndex(goal.x, goal.y);

    work.g_cost[start_index] = 0;
    setParent(work, start_index, PARENT_NONE);

    setState(work, start_index, STATE_OPEN);
    pushOpen(work, start_index);

    const uint8_t dir_count = config.allow_diagonal ? 8 : 4;

    while (work.open_count > 0)
    {
        const int16_t current_index_signed = popBestOpen(work, goal, config);

        if (current_index_signed < 0)
        {
            break;
        }

        const uint16_t current_index = static_cast<uint16_t>(current_index_signed);

        if (getState(work, current_index) == STATE_CLOSED)
        {
            continue;
        }

        setState(work, current_index, STATE_CLOSED);
        result.searched_count++;

        if (current_index == goal_index)
        {
            bool overflow = false;

            size_t count = reconstructPathGrid(
                work,
                start,
                goal,
                out_path,
                out_path_max,
                &overflow
            );

            if (config.simplify_path)
            {
                count = simplifyPathGrid(out_path, count);
            }

            result.found = true;
            result.path_count = count;
            result.path_overflow = overflow;

            return result;
        }

        const GridPos current = toGridPos(current_index);

        for (uint8_t dir = 0; dir < dir_count; dir++)
        {
            const int8_t dx = DIR8[dir][0];
            const int8_t dy = DIR8[dir][1];

            const int16_t nx_i = static_cast<int16_t>(current.x) + dx;
            const int16_t ny_i = static_cast<int16_t>(current.y) + dy;

            if (!inBounds(nx_i, ny_i))
            {
                continue;
            }

            const uint8_t nx = static_cast<uint8_t>(nx_i);
            const uint8_t ny = static_cast<uint8_t>(ny_i);

            if (!isWalkable(map, nx, ny, config))
            {
                continue;
            }

            if (!canMoveDiagonal(map, current.x, current.y, dx, dy, config))
            {
                continue;
            }

            const uint16_t next_index = toIndex(nx, ny);

            if (getState(work, next_index) == STATE_CLOSED)
            {
                continue;
            }

            const bool diagonal_move = (dx != 0 && dy != 0);
            uint16_t move_cost = diagonal_move ? 14 : 10;

            const uint8_t cell_value = getCell(map, nx, ny);
            move_cost = saturatingCostAdd(
                move_cost,
                cellTraversalExtraCost(cell_value, config));

            const uint16_t tentative_g = saturatingCostAdd(
                work.g_cost[current_index],
                move_cost);

            const uint8_t next_state = getState(work, next_index);

            if (next_state == STATE_NONE || tentative_g < work.g_cost[next_index])
            {
                work.g_cost[next_index] = tentative_g;
                setParent(work, next_index, oppositeDir(dir));

                if (next_state != STATE_OPEN)
                {
                    setState(work, next_index, STATE_OPEN);

                    if (!pushOpen(work, next_index))
                    {
                        result.path_overflow = true;
                        return result;
                    }
                }
            }
        }
    }

    result.found = false;
    return result;
}

// ============================================================
// A* main: PurePursuit path output
// ============================================================

inline Result findPathForPurePursuit(
    const uint8_t* map,
    GridPos start,
    GridPos goal,
    PurePursuit::Point* pp_path,
    size_t pp_path_max,
    GridPos* work_grid_path,
    size_t work_grid_path_max,
    Work& work,
    const Config& config
)
{
    Result result = findPathGrid(
        map,
        start,
        goal,
        work_grid_path,
        work_grid_path_max,
        work,
        config
    );

    if (!result.found)
    {
        return result;
    }

    const size_t pp_count = convertGridPathToPurePursuitPath(
        work_grid_path,
        result.path_count,
        pp_path,
        pp_path_max,
        config
    );

    if (pp_count < result.path_count)
    {
        result.path_overflow = true;
    }

    result.path_count = pp_count;

    return result;
}

inline Result findPathFromWorldForPurePursuit(
    const uint8_t* map,
    float start_x_mm,
    float start_y_mm,
    float goal_x_mm,
    float goal_y_mm,
    PurePursuit::Point* pp_path,
    size_t pp_path_max,
    GridPos* work_grid_path,
    size_t work_grid_path_max,
    Work& work,
    const Config& config
)
{
    GridPos start{};
    GridPos goal{};

    if (!worldToGridChecked(start_x_mm, start_y_mm, start, config) ||
        !worldToGridChecked(goal_x_mm, goal_y_mm, goal, config))
    {
        return Result{};
    }

    return findPathForPurePursuit(
        map,
        start,
        goal,
        pp_path,
        pp_path_max,
        work_grid_path,
        work_grid_path_max,
        work,
        config
    );
}

} // namespace AStar
