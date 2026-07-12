#include "start.h"

#include <Arduino.h>
#include "freertos/task.h"
#include "battery.h"
#include "executors.h"
#include "host_comm.h"
#include "led_dev.h"
#include "rgb_dev.h"
#include "xbox_dev.h"
#include "esp_http_server.h"
#include "system_app.h"

/**
 * @brief 创建系统中的 RTOS 任务
 */
static void task_list()
{
    xTaskCreatePinnedToCore(battery::task_entry, "battery_task", 2048, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(led_dev::task_entry, "led_dev_task", 1024, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(rgb_dev::task_entry, "rgb_dev_task", 2048, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(xbox_dev::task_entry, "xbox_dev_task", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(system_executor::fast_io_task_entry, "fast_io_task", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(system_executor::control_task_entry, "control_task", 4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(host_comm::task_entry, "host_comm_task", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(esp_http_server::task_entry, "http_server_task", 4096, nullptr, 2, nullptr, 0);
}

/**
 * @brief 执行系统启动初始化
 */
void start_init_all()
{
    delay(1000);

    system_app::init();

    task_list();
}
