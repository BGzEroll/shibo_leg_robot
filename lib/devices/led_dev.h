#ifndef LED_DEV_H
#define LED_DEV_H

#include "led.h"
#include "battery.h"

namespace led_dev
{
    extern led board_led;

    struct input_ports
    {
        port::latest_reader<battery::data> battery_status;
    };

    void init(const input_ports &inputs);
    void task_entry(void *arg);
}

#endif
