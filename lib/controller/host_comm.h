#ifndef HOST_COMM_H
#define HOST_COMM_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace host_comm
{
    struct remote_data
    {
        uint32_t timestamp_us = 0;
        uint16_t buttons = 0;
        float axes[6]{};
    };

    struct vision_measurement
    {
        int16_t dx = 0;
        int16_t dy = 0;
        uint32_t timestamp_ms = 0;
        bool valid = false;
    };

    QueueHandle_t remote_queue();
    bool vision_latest(vision_measurement &out);
    void task_entry(void *arg);
}

#endif
