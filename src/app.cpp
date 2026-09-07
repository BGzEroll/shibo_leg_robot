#include "app.h"

#include "control/control.h"
#include "freertos/task.h"
#include "hw/battery.h"
#include "hw/gamepad.h"
#include "hw/imu.h"
#include "hw/indicator.h"
#include "hw/motor.h"
#include "hw/sensor.h"
#include "hw/servo.h"
#include "hw/wifi.h"
#include "io/host.h"
#include "io/web.h"
#include <atomic>

static std::atomic<bool> application_ready{false};

/* ---- 任务创建 ---- */

static TaskFunction_t deferred_tasks[] =
{
    hw::sensor::task_entry,
    control::foc_task_entry,
    control::control_task_entry,
    control::service_task_entry
};

/**
 * @brief 等待硬件初始化完成后进入实际任务入口
 *
 * @param arg 延迟任务配置
 */
static void deferred_task_entry(void *arg)
{
    TaskFunction_t task_entry = *static_cast<TaskFunction_t *>(arg);
    while(!application_ready.load(std::memory_order_acquire))
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    task_entry(nullptr);
    vTaskDelete(nullptr);
}

/**
 * @brief 在 BLE 建立连接前预先分配全部应用任务
 */
static void create_application_tasks()
{
    xTaskCreatePinnedToCore(
        deferred_task_entry, "sensor_task", 4096, &deferred_tasks[0], 5, nullptr, 1);
    xTaskCreatePinnedToCore(
        deferred_task_entry, "foc_task", 4096, &deferred_tasks[1], 5, nullptr, 1);
    xTaskCreatePinnedToCore(
        deferred_task_entry, "control_task", 4096, &deferred_tasks[2], 5, nullptr, 0);
    // xTaskCreatePinnedToCore(
    //     io::host::task_entry, "host_task", 4096, nullptr, 3, nullptr, 0);
    xTaskCreatePinnedToCore(
        deferred_task_entry, "service_task", 4096, &deferred_tasks[3], 2, nullptr, 0);
    xTaskCreatePinnedToCore(
        hw::gamepad::task_entry, "gamepad_task", 4096, nullptr, 3, nullptr, 0);
}

/* ---- app 公共 API ---- */

/**
 * @brief 查询硬件初始化和控制任务是否已经全部就绪
 *
 * @return 应用可以接收控制输入时返回 true
 */
bool app::ready()
{
    return application_ready.load(std::memory_order_acquire);
}

/**
 * @brief 初始化所有模块并启动应用任务
 */
void app::start()
{
    application_ready.store(false, std::memory_order_release);
    delay(1000);

    hw::gamepad::init();
    hw::wifi::init();
    io::web::init();
    io::host::init();

    // 先锁定控制任务所需内存，再让 BLE 在硬件校准期间建立连接。
    create_application_tasks();

    hw::battery::init();
    hw::indicator::init();
    hw::servo::init();
    hw::imu::init();
    hw::motor::init();
    control::init();

    application_ready.store(true, std::memory_order_release);
}
