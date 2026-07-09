#ifndef ESP_HTTP_SERVER_H
#define ESP_HTTP_SERVER_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace esp_http_server
{
    struct remote_input_data
    {
        uint32_t timestamp_us = 0;
        uint16_t buttons = 0;
        float axes[6]{};
    };

    QueueHandle_t remote_queue();
    void init();
    void task_entry(void *arg);
}

#endif
