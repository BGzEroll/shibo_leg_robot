#ifndef CONTROL_CONTROL_H
#define CONTROL_CONTROL_H

#include "balance.h"

namespace control
{
    bool request_middle_calibration();
    bool middle_calibration_success();

    void init();
    void foc_task_entry(void *arg);
    void control_task_entry(void *arg);
    void service_task_entry(void *arg);
}

#endif
