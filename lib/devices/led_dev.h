#ifndef LED_DEV_H
#define LED_DEV_H

#include "led.h"

namespace led_dev
{
    extern led board_led;

    void init();
    void task_entry(void *arg);
}

#endif
