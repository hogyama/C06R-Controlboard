#pragma once

#include <stdint.h>
#include "domain/fusion/fusion_types.h"

namespace Domain::Navigation {

struct Input {
    Domain::Fusion::Output fusion;
    int32_t goal_x_mm;
    int32_t goal_y_mm;
    uint32_t grid_map_update_count;
};

struct JogCommand {
    int16_t velocity_mm_s;
    int16_t omega_rad_s_x100;
    uint16_t duration_ms;
};

} // namespace Domain::Navigation
