#ifndef BALANCE_CORE_H
#define BALANCE_CORE_H

#include "control_types.h"

namespace controller {

void balance_core_init();
void balance_core_set_command(const balance_command &cmd);
bool balance_core_get_status(balance_status &out);
void balance_core_set_mode(mode_id mode);

float balance_core_max_linear_vel();
float balance_core_max_steer_vel();
float balance_core_wheel_radius();

void balance_core_io_task(void *arg);
void balance_core_control_task(void *arg);

}

#endif
