#ifndef RGB_DEV_H
#define RGB_DEV_H

#include "battery.h"

namespace rgb_dev
{
    struct input_ports
    {
        port::latest_reader<battery::data> battery_status;
    };

    void init(const input_ports &inputs);
    void task_entry(void *arg);
}

#endif
