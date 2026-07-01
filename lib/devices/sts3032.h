#ifndef STS3032_H
#define STS3032_H

#include <Arduino.h>

#define SERVO_LEFT          1       // 左舵机
#define SERVO_RIGHT         2       // 右舵机

// 舵机位置限制参数，注意，该舵机中位于 2048，顺时针旋转时计数值增加，负载正方向增加，反之亦然
#define LEG_HEIGHT_BASE     20      // 腿部默认高度基准值，值越小，机身越高， 实测范围 -10（最高），52（最低）
#define SERVO_LEFT_MIN      (2048 + 40)     // 左舵机最低位置
#define SERVO_RIGHT_MIN     (2048 - 40)     // 右舵机最低位置
#define SERVO_LEFT_MAX      (2048 + 450)    // 左舵机最高
#define SERVO_RIGHT_MAX     (2048 - 450)    // 右舵机最高

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
