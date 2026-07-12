#ifndef ESP_HTTP_SERVER_H
#define ESP_HTTP_SERVER_H

#include <Arduino.h>
#include "ports/latest_value.h"

namespace esp_http_server
{
    struct remote_input_data
    {
        uint32_t timestamp_us = 0;
        uint16_t buttons = 0;
        float axes[6]{};
    };

    struct calibration_request
    {
        uint32_t sequence = 0;
    };

    struct calibration_status
    {
        uint32_t sequence = 0;
        bool success = false;
    };

    struct input_ports
    {
        port::latest_reader<calibration_status> calibration;
    };

    struct output_ports
    {
        port::latest_writer<remote_input_data> remote_control;
        port::latest_writer<calibration_request> calibration;
    };

    void init(const input_ports &inputs, const output_ports &outputs);
    void task_entry(void *arg);
}

#endif
