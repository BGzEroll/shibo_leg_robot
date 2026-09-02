#ifndef HW_BATTERY_H
#define HW_BATTERY_H

#include <Arduino.h>

namespace hw
{
    namespace battery
    {
        struct state
        {
            uint32_t timestamp_ms = 0;
            float voltage = 0.0f;
            bool valid = false;
            bool low = false;
        };

        bool latest(state &out);
        void init();
        void update();
    }
}

#endif
