#ifndef HOST_COMM_H
#define HOST_COMM_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace host_comm {

struct remote_data {
    uint32_t timestamp_us = 0;
    uint16_t buttons = 0;
    float axes[6]{};
};

void init();
QueueHandle_t remote_queue();
void task(void *arg);

}

#endif
