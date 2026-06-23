#ifndef ACTIONS_H
#define ACTIONS_H

#include "control_types.h"

namespace controller {

struct action_io {
    control_input &input;
    balance_status &status;
    leg_runtime &leg;
    float max_linear_vel;
};

struct action_state {
    mode_id mode = mode_id::BOOT;
    uint8_t phase = 0;
    uint32_t timer = 0;
    uint32_t ready_timer = 0;
    uint32_t elapsed = 0;
    int8_t jump_linear_dir = 0;
    int8_t jump_turn_dir = 0;
    float jump_target_yaw = 0.0f;
    float jump_linear_cmd = 0.0f;
    float jump_yaw_cmd = 0.0f;
};

void actions_init(action_state &state);
mode_id actions_mode(const action_state &state);
balance_command actions_update(action_state &state, action_io &ctx, uint32_t tick_ms);

}

#endif
