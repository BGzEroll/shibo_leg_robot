#ifndef STS3032_H
#define STS3032_H

#include <Arduino.h>

#define SERVO_LEFT          1       // 左舵机
#define SERVO_RIGHT         2       // 右舵机

namespace sts3032
{
    struct status_data
    {
        uint8_t id;
        int16_t position;
        int16_t load;
    };

    extern status_data status[2];

    void get_position_and_load();
    void set_torque_switch(uint8_t id, uint8_t type);
    void set(uint8_t id, int16_t position, int16_t speed, uint8_t acc);
    void move();
    void calibrate_middle();
    void init();
}

#endif
