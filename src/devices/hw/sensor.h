#ifndef HW_SENSOR_H
#define HW_SENSOR_H

#include "bus/i2c_bus.h"

namespace hw
{
    namespace sensor
    {
        extern i2c_bus left_bus;
        extern i2c_bus right_bus;

        void task_entry(void *arg);
    }
}

#endif
