#ifndef BATTERY_H
#define BATTERY_H

#include <Arduino.h>

namespace battery
{
    struct data
    {
        uint32_t timestamp_ms = 0;
        float voltage = 0.0f;
        bool valid = false;
        bool low = false;
    };

    QueueHandle_t queue();
    void init();
    void task_entry(void *arg);
}

#endif
