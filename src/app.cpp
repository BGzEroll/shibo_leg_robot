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
static std::atomic<bool> control_tasks_released{false};
static std::atomic<app::startup_stage> current_startup_stage{
    app::startup_stage::STARTING};

/* ---- 任务创建 ---- */

/**
 * @brief 等待硬件初始化完成后进入实际任务入口
 *
 * @param task_entry 实际任务入口
 * @param arg 实际任务参数
 */
static void run_after_hardware_init(TaskFunction_t task_entry, void *arg)
{
    while(!control_tasks_released.load(std::memory_order_acquire))
    {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    task_entry(arg);
    vTaskDelete(nullptr);
}

/** @brief 等待硬件就绪后进入传感器任务 */
static void sensor_task_entry(void *arg)
{
    run_after_hardware_init(hw::sensor::task_entry, arg);
}

/** @brief 等待硬件就绪后进入 FOC 任务 */
static void foc_task_entry(void *arg)
{
    run_after_hardware_init(control::foc_task_entry, arg);
}

/** @brief 等待硬件就绪后进入控制任务 */
static void control_task_entry(void *arg)
{
    run_after_hardware_init(control::control_task_entry, arg);
}

/** @brief 等待硬件就绪后进入维护任务 */
static void service_task_entry(void *arg)
{
    run_after_hardware_init(control::service_task_entry, arg);
}

/**
 * @brief 在 BLE 建立连接前预先分配全部应用任务
 *
 * @return 全部任务创建成功时返回 true
 */
static bool create_application_tasks()
{
    bool success = true;
    success &= xTaskCreatePinnedToCore(
        sensor_task_entry, "sensor_task", 4096, nullptr, 5, nullptr, 1) == pdPASS;
    success &= xTaskCreatePinnedToCore(
        foc_task_entry, "foc_task", 4096, nullptr, 5, nullptr, 1) == pdPASS;
    success &= xTaskCreatePinnedToCore(
        control_task_entry, "control_task", 4096, nullptr, 5, nullptr, 0) == pdPASS;
    // xTaskCreatePinnedToCore(
    //     io::host::task_entry, "host_task", 4096, nullptr, 3, nullptr, 0);
    success &= xTaskCreatePinnedToCore(
        service_task_entry, "service_task", 4096, nullptr, 2, nullptr, 0) == pdPASS;
    success &= xTaskCreatePinnedToCore(
        hw::gamepad::task_entry, "gamepad_task", 4096, nullptr, 3, nullptr, 0) == pdPASS;
    return success;
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
 * @brief 查询当前启动阶段
 *
 * @return 当前启动阶段
 */
app::startup_stage app::stage()
{
    return current_startup_stage.load(std::memory_order_acquire);
}

/**
 * @brief 初始化所有模块并启动应用任务
 */
void app::start()
{
    application_ready.store(false, std::memory_order_release);
    control_tasks_released.store(false, std::memory_order_release);
    current_startup_stage.store(app::startup_stage::STARTING, std::memory_order_release);
    delay(1000);

    current_startup_stage.store(app::startup_stage::SERVICES, std::memory_order_release);
    hw::gamepad::init();
    hw::wifi::init();
    io::web::init();
    io::host::init();

    // 先锁定控制任务所需内存，再让 BLE 在硬件校准期间建立连接。
    current_startup_stage.store(app::startup_stage::TASKS, std::memory_order_release);
    if(!create_application_tasks())
    {
        current_startup_stage.store(
            app::startup_stage::TASK_CREATION_FAILED, std::memory_order_release);
        return;
    }

    current_startup_stage.store(app::startup_stage::BATTERY, std::memory_order_release);
    hw::battery::init();
    current_startup_stage.store(app::startup_stage::INDICATOR, std::memory_order_release);
    hw::indicator::init();
    current_startup_stage.store(app::startup_stage::SERVO, std::memory_order_release);
    hw::servo::init();
    current_startup_stage.store(app::startup_stage::IMU, std::memory_order_release);
    hw::imu::init();
    current_startup_stage.store(app::startup_stage::MOTOR, std::memory_order_release);
    hw::motor::init();
    current_startup_stage.store(app::startup_stage::CONTROL, std::memory_order_release);
    control::init();

    application_ready.store(true, std::memory_order_release);
    current_startup_stage.store(app::startup_stage::READY, std::memory_order_release);
    control_tasks_released.store(true, std::memory_order_release);
}
