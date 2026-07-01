#ifndef PTK7350_H
#define PTK7350_H

#include "pwm_servo.h"

#define CAMSERVO_MIN            0      // 摄像头朝上 180 度，朝下 0 度，按照自己的安装来调试
#define CAMSERVO_MAX            180
#define FRONTIERSERVO_MIN       0      // 前挡板在外 0 度，在内 180 度，按照自己的安装来调试
#define FRONTIERSERVO_MAX       180

namespace ptk7350
{
    extern pwm_servo cam_servo;
    extern pwm_servo frontier_servo;
}

#endif
