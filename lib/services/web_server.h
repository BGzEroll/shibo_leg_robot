#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <Arduino.h>

namespace web_server
{
    struct input
    {
        uint32_t stream_id = 0;
        uint32_t sequence = 0;
        uint32_t timestamp_us = 0;
        uint16_t held_buttons = 0;
        uint16_t press_count[16]{};
        float axes[6]{};
        bool valid = false;
    };

    bool peek_input(input &out);
    bool init();
    void task_entry(void *arg);
}

#endif
