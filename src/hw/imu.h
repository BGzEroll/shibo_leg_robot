#ifndef HW_IMU_H
#define HW_IMU_H

#include <Arduino.h>

namespace hw
{
    namespace imu
    {
        struct state
        {
            uint32_t timestamp_us = 0;
            float temperature = 0.0f;
            float acc[3]{};
            float gyro[3]{};
            float angle[3]{};
        };

        bool latest(state &out);
        void init();
        void sample();
    }
}

#endif
