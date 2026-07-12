#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>
#include "ports/latest_value.h"

namespace battery
{
    struct data
    {
        uint32_t timestamp_ms = 0;
        float voltage = 0.0f;
        bool valid = false;
        bool low = false;
    };

    struct output_ports
    {
        port::latest_writer<data> status;
    };

    void init(const output_ports &outputs);
    void task_entry(void *arg);
}

#endif
