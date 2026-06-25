#include "start.h"

#include <Arduino.h>
#include "freertos/task.h"
#include "balance_core.h"
#include "controller.h"
#include "host_comm.h"
#include "led_dev.h"
#include "xbox_dev.h"
#include "esp_http_server.h"

/**
 * @brief 创建系统中的 RTOS 任务
 */
static void task_list()
{
    xTaskCreatePinnedToCore(led_dev::task_entry, "led_dev_task", 1024, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(xbox_dev::task_entry, "xbox_dev_task", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(balance_core::core_task_entry, "balance_io_task", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(balance_core::control_task_entry, "balance_ctl_task", 4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(host_comm::task_entry, "host_comm_task", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(esp_http_server::task_entry, "http_server_task", 4096, nullptr, 3, nullptr, 0);
}

/**
 * @brief 执行系统启动初始化
 */
void start_init_all()
{
    delay(1000);

    led_dev::init();
    xbox_dev::init();
    esp_http_server::init();
    controller::init();

    task_list();
}
