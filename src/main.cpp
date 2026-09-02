#include <Arduino.h>

#include "app.h"

/**
 * @brief Arduino 启动入口
 */
void setup()
{
    app::start();
    vTaskDelete(nullptr);
}

/**
 * @brief Arduino 空循环入口
 */
void loop()
{
    vTaskDelay(portMAX_DELAY);
}
