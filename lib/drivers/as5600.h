#ifndef AS5600_H
#define AS5600_H

#include "bus/i2c_bus.h"

class as5600
{
    public:
        explicit as5600(i2c_bus &bus);

    public:
        struct sample
        {
            uint32_t timestamp_us = 0;
            uint16_t raw = 0;
            bool valid = false;
        };

        sample read();

    private:
        i2c_bus &bus;
};

#endif
