#include "start.h"

#include <Arduino.h>
#include "freertos/task.h"
#include "balance_core.h"
#include "controller.h"
#include "host_comm.h"
#include "led_dev.h"
#include "xbox_dev.h"

static void task_list()
{
    xTaskCreatePinnedToCore(led_dev::task, "led_dev_task", 1024, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(xbox_dev::task, "xbox_dev_task", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(balance_core::core_task_entry, "balance_io_task", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(balance_core::control_task_entry, "balance_ctl_task", 4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(host_comm::task, "host_comm_task", 4096, nullptr, 3, nullptr, 0);
}

void start_init_all()
{
    delay(1000);

    led_dev::init();
    xbox_dev::init();
    host_comm::init();
    balance_core::init();
    controller::init();

    task_list();
}
