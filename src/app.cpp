#include "app.h"

#include "control/control.h"
#include "freertos/task.h"
#include "hw/battery.h"
#include "hw/gamepad.h"
#include "hw/imu.h"
#include "hw/indicator.h"
#include "hw/motor.h"
#include "hw/servo.h"
#include "hw/wifi.h"
#include "io/host.h"
#include "io/web.h"

/* ---- 任务创建 ---- */

/**
 * @brief 创建应用的固定任务集合
 */
static void create_tasks()
{
    xTaskCreatePinnedToCore(
        control::foc_task_entry, "foc_task", 4096, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(
        control::control_task_entry, "control_task", 4096, nullptr, 5, nullptr, 0);
    xTaskCreatePinnedToCore(
        io::host::task_entry, "host_task", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(
        hw::gamepad::task_entry, "gamepad_task", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(
        control::service_task_entry, "service_task", 4096, nullptr, 2, nullptr, 0);
}

/* ---- app 公共 API ---- */

/**
 * @brief 初始化所有模块并启动应用任务
 */
void app::start()
{
    delay(1000);

    hw::battery::init();
    hw::indicator::init();
    hw::servo::init();
    hw::imu::init();
    hw::motor::init();
    control::init();
    hw::wifi::init();
    hw::gamepad::init();
    io::host::init();
    io::web::init();

    create_tasks();
}
