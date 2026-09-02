#ifndef IO_HOST_H
#define IO_HOST_H

#include <Arduino.h>
#include "control/input.h"

namespace io
{
    namespace host
    {
        struct vision_measurement
        {
            int16_t dx = 0;
            int16_t dy = 0;
            uint32_t timestamp_ms = 0;
            uint32_t sequence = 0;
            bool valid = false;
        };

        bool latest_input(control::remote_input &out);
        bool latest_vision(vision_measurement &out);

        void init();
        void task_entry(void *arg);
    }
}

#endif
