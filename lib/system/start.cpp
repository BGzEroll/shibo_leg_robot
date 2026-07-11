#include "start.h"

#include "robot_profile.h"
#include <Arduino.h>
#include "freertos/task.h"

/**
 * @brief 创建系统中的 RTOS 任务
 */
static void task_list()
{
    uint8_t count = 0;
    const application::task_descriptor *tasks = application::task_list(count);
    for(uint8_t i = 0; i < count; i++)
    {
        const application::task_descriptor &task = tasks[i];
        BaseType_t result = xTaskCreatePinnedToCore(
            task.entry,
            task.name,
            task.stack_size,
            nullptr,
            task.priority,
            nullptr,
            task.core_id);
        configASSERT(result == pdPASS);
    }
}

/**
 * @brief 执行系统启动初始化
 */
void start_init_all()
{
    delay(1000);

    application::init();

    task_list();
}
