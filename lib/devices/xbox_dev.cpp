#include "xbox_dev.h"

#include "esp_timer.h"
#include <string.h>

xbox xbox_dev::gamepad("fc:0b:58:01:99:76");
static QueueHandle_t xbox_data_queue = nullptr;

/**
 * @brief 获取 Xbox 输入数据队列
 *
 * @return 队列句柄
 */
QueueHandle_t xbox_dev::queue()
{
    return xbox_data_queue;
}

/**
 * @brief 初始化 Xbox 设备模块
 */
void xbox_dev::init()
{
    gamepad.init();

    xbox_data_queue = xQueueCreate(1, sizeof(xbox_dev::data));
}

/**
 * @brief Xbox 输入采样任务入口
 *
 * @param arg RTOS 任务参数
 */
void xbox_dev::task_entry(void *arg)
{

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
