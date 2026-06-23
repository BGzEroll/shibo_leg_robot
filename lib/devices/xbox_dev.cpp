#include "xbox_dev.h"

#include "esp_timer.h"
#include <string.h>

namespace xbox_dev {

xbox gamepad("fc:0b:58:01:99:76");
static QueueHandle_t xbox_data_queue = nullptr;

QueueHandle_t queue()
{
    return xbox_data_queue;
}

void init()
{
    gamepad.init();

    xbox_data_queue = xQueueCreate(1, sizeof(data));
}

void task(void *arg)
{
    (void)arg;

    while(true)
    {
        gamepad.update();

        xbox_dev::data state;
        state.timestamp_us = (uint32_t)esp_timer_get_time();
        state.buttons = gamepad.buttons;
        memcpy(state.axes, gamepad.axes, sizeof(gamepad.axes));

        if(xbox_data_queue)
        {
            xQueueOverwrite(xbox_data_queue, &state);
        }

        delay(20);
    }
}

}
