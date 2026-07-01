#include <Arduino.h>

#include "start.h"

/**
 * @brief Arduino 启动入口
 */
void setup()
{
    start_init_all();
    vTaskDelete(nullptr);    // 关闭 arduino 的 loop 任务
}

/**
 * @brief Arduino 空循环入口
 */
void loop()
{
    vTaskDelay(portMAX_DELAY);
}
