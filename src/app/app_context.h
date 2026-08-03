#pragma once
#include "service/Can/srv_can.h"
#include "service/Gps/srv_gps.h"
#include "service/Twe/srv_twe.h"
#include "service/Rasp/srv_rasp.h"
#include "algorithm/pure_pursuit.h"
#include "algorithm/Astar.h"
#include "service/Flash/srv_flash.h"

extern SrvCan can;
extern SrvGps gps;
extern SrvTwe twe;
extern SrvRasp rasp;
extern SrvFlash flash;

// algorithm

extern uint8_t grid_map[AStar::MAP_BYTES];
extern uint32_t grid_map_update_count;

extern AStar::Work astar_work;

// 経路出力用
extern AStar::GridPos grid_path[128];
extern PurePursuit::Point pp_path[128];
extern size_t pp_path_count;

extern AStar::Config astar_config;
extern PurePursuit::Config pp_config;
extern PurePursuit::PathState pp_state;
