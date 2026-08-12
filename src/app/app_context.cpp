#include "app_context.h"
#include "platform/field_config.h"

static_assert(AStar::MAP_BYTES == MAP_SIZE,
              "RAM and Flash packed-map sizes must match");

uint8_t grid_map[AStar::MAP_BYTES];
uint32_t grid_map_update_count = 0;

// 各サービスの実体は一つだけ生成し、対応するtaskから操作する。
SrvCan can;
SrvGps gps;
SrvTwe twe;
SrvRasp rasp;

// FlashヘッダにもGPSゴール座標を保存し、別地点の地図混入を防ぐ。
SrvFlash flash(
    FieldConfig::GOAL_LATITUDE_E7,
    FieldConfig::GOAL_LONGITUDE_E7);
AStar::Work astar_work;
AStar::GridPos grid_path[AStar::CELL_COUNT];
PurePursuit::Point pp_path[128];
size_t pp_path_count = 0;

AStar::Config astar_config;
PurePursuit::Config pp_config;
PurePursuit::PathState pp_state;
