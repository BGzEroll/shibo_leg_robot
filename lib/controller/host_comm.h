#ifndef HOST_COMM_H
#define HOST_COMM_H

#include <Arduino.h>
#include "balance_core.h"
#include "ports/latest_value.h"

namespace host_comm
{
    struct remote_data
    {
        uint32_t timestamp_us = 0;
        uint16_t buttons = 0;
        float axes[6]{};
    };

    struct vision_measurement
    {
        int16_t dx = 0;
        int16_t dy = 0;
        uint32_t timestamp_ms = 0;
        uint32_t seq = 0;
        bool valid = false;
    };

    struct input_ports
    {
        port::latest_reader<balance_core::debug_snapshot> debug_status;
    };

    struct output_ports
    {
        port::latest_writer<remote_data> remote_control;
        port::latest_writer<vision_measurement> vision;
    };

    void init(const input_ports &inputs, const output_ports &outputs);
    void task_entry(void *arg);
}

#endif
