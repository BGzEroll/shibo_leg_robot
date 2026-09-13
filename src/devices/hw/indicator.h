#ifndef HW_INDICATOR_H
#define HW_INDICATOR_H

#include <Arduino.h>

namespace hw
{
    namespace indicator
    {
        void init();
        void update(bool battery_low, uint32_t tick_ms);
    }
}

#endif
