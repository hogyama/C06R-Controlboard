#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "algorithm/astar.h"
#include "algorithm/pure_pursuit.h"

namespace {

uint8_t map_data[AStar::MAP_BYTES];
AStar::Work work;
AStar::GridPos grid_path[128];

void initializeUnknown()
{
    memset(map_data, 0x77, sizeof(map_data));
}

void testPackedCellAccess()
{
    initializeUnknown();
    assert(AStar::MAP_BYTES == 2048U);
    assert(AStar::getCell(map_data, 0, 0) == AStar::CELL_UNKNOWN);
    assert(AStar::getCell(map_data, 1, 0) == AStar::CELL_UNKNOWN);

    AStar::setCell(map_data, 0, 0, 3);
    AStar::setCell(map_data, 1, 0, 14);
    assert(AStar::getCell(map_data, 0, 0) == 3);
    assert(AStar::getCell(map_data, 1, 0) == 14);

    AStar::setCell(map_data, 0, 0, AStar::CELL_BLOCKED);
    assert(AStar::getCell(map_data, 0, 0) == AStar::CELL_BLOCKED);
    assert(AStar::getCell(map_data, 1, 0) == 14);
}

void testEvidenceKernel()
{
    initializeUnknown();
    const AStar::GridPos center{10, 10};
    assert(AStar::applyEvidenceKernel(map_data, center, 5, 1, 14));
    assert(AStar::getCell(map_data, 10, 10) == 12);
    assert(AStar::getCell(map_data, 11, 10) == 9);
    assert(AStar::getCell(map_data, 11, 11) == 8);

    assert(AStar::adjustCell(map_data, 10, 10, 10, 14) == 14);
    assert(AStar::adjustCell(map_data, 10, 10, -20) == AStar::CELL_FREE);
}

void testPackedSearchMetadata()
{
    AStar::resetWork(work);
    AStar::setParent(work, 0, 3);
    AStar::setParent(work, 1, 6);
    AStar::setParent(work, 2, AStar::PARENT_NONE);
    assert(AStar::getParent(work, 0) == 3);
    assert(AStar::getParent(work, 1) == 6);
    assert(AStar::getParent(work, 2) == AStar::PARENT_NONE);

    AStar::setState(work, 0, AStar::STATE_OPEN);
    AStar::setState(work, 1, AStar::STATE_CLOSED);
    AStar::setState(work, 2, AStar::STATE_OPEN);
    assert(AStar::getState(work, 0) == AStar::STATE_OPEN);
    assert(AStar::getState(work, 1) == AStar::STATE_CLOSED);
    assert(AStar::getState(work, 2) == AStar::STATE_OPEN);
}

void testWeightedAStarAvoidsRisk()
{
    initializeUnknown();
    AStar::Config config{};
    config.allow_diagonal = false;
    config.simplify_path = false;

    AStar::setCell(map_data, 2, 1, 14);
    AStar::setCell(map_data, 3, 1, 14);
    AStar::setCell(map_data, 4, 1, 14);

    const AStar::Result result = AStar::findPathGrid(
        map_data,
        AStar::GridPos{1, 1},
        AStar::GridPos{5, 1},
        grid_path,
        128,
        work,
        config);

    assert(result.found);
    assert(!result.path_overflow);
    for (size_t i = 0; i < result.path_count; ++i) {
        const uint8_t value =
            AStar::getCell(map_data, grid_path[i].x, grid_path[i].y);
        assert(value < 14);
    }
}

void testBlockedCellAndPurePursuit()
{
    initializeUnknown();
    AStar::Config config{};
    AStar::setCell(map_data, 5, 5, AStar::CELL_BLOCKED);
    const AStar::Result blocked = AStar::findPathGrid(
        map_data,
        AStar::GridPos{1, 1},
        AStar::GridPos{5, 5},
        grid_path,
        128,
        work,
        config);
    assert(!blocked.found);
    assert(blocked.goal_blocked);

    const PurePursuit::Point path[] = {
        {0.0f, 0.0f},
        {1000.0f, 0.0f},
        {2000.0f, 0.0f}
    };
    PurePursuit::PathState state{};
    PurePursuit::Config pursuit_config{};
    const PurePursuit::Output output =
        PurePursuit::calculatePathFollowing(
            PurePursuit::Pose{0.0f, 0.0f, 0.0f},
            path,
            3,
            state,
            pursuit_config);
    assert(output.valid);
    assert(!output.reached_goal);
    assert(output.linear_velocity_mm_s > 0.0f);
    assert(fabsf(output.angular_velocity_rad_s) < 1.0e-5f);
}

void testBlockedFallbackKeepsMapAndFindsPath()
{
    initializeUnknown();
    AStar::Config normal_config{};
    normal_config.allow_diagonal = false;
    normal_config.simplify_path = false;

    for (uint8_t y = 0; y < AStar::MAP_H; ++y) {
        AStar::setCell(
            map_data, 3, y, AStar::CELL_BLOCKED);
    }

    const AStar::Result normal = AStar::findPathGrid(
        map_data,
        AStar::GridPos{1, 1},
        AStar::GridPos{5, 1},
        grid_path,
        128,
        work,
        normal_config);
    assert(!normal.found);

    AStar::Config relaxed_config = normal_config;
    relaxed_config.allow_blocked_as_high_cost = true;
    const AStar::Result relaxed = AStar::findPathGrid(
        map_data,
        AStar::GridPos{1, 1},
        AStar::GridPos{5, 1},
        grid_path,
        128,
        work,
        relaxed_config);
    assert(relaxed.found);
    assert(!relaxed.path_overflow);

    bool crossed_blocked = false;
    for (size_t i = 0; i < relaxed.path_count; ++i) {
        if (AStar::getCell(
                map_data,
                grid_path[i].x,
                grid_path[i].y) == AStar::CELL_BLOCKED) {
            crossed_blocked = true;
        }
    }
    assert(crossed_blocked);
    assert(AStar::getCell(map_data, 3, 1) ==
           AStar::CELL_BLOCKED);
}

} // namespace

int main()
{
    testPackedCellAccess();
    testEvidenceKernel();
    testPackedSearchMetadata();
    testWeightedAStarAvoidsRisk();
    testBlockedCellAndPurePursuit();
    testBlockedFallbackKeepsMapAndFindsPath();
    return 0;
}
