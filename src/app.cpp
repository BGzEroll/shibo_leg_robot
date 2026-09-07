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

/**
 * @brief 创建可在硬件校准期间独立运行的手柄通信任务
 *
 * @return 任务创建成功时返回 true
 */
static bool create_gamepad_task()
{
    return xTaskCreatePinnedToCore(
        hw::gamepad::task_entry, "gamepad_task", 4096, nullptr, 3, nullptr, 0) == pdPASS;
}

/**
 * @brief 创建依赖全部硬件初始化完成的控制与维护任务
 *
 * @return 全部任务创建成功时返回 true
 */
static bool create_control_tasks()
{
    bool success = true;
    success &= xTaskCreatePinnedToCore(
        hw::sensor::task_entry, "sensor_task", 4096, nullptr, 5, nullptr, 1) == pdPASS;
    success &= xTaskCreatePinnedToCore(
        control::foc_task_entry, "foc_task", 4096, nullptr, 5, nullptr, 1) == pdPASS;
    success &= xTaskCreatePinnedToCore(
        control::control_task_entry, "control_task", 4096, nullptr, 5, nullptr, 0) == pdPASS;
    // xTaskCreatePinnedToCore(
    //     io::host::task_entry, "host_task", 4096, nullptr, 3, nullptr, 0);
    success &= xTaskCreatePinnedToCore(
        control::service_task_entry, "service_task", 4096, nullptr, 2, nullptr, 0) == pdPASS;
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
 * @brief 初始化所有模块并启动应用任务
 */
void app::start()
{
    application_ready.store(false, std::memory_order_release);
    delay(1000);

    // 手柄任务先运行，避免耗时的 FOC 校准阻塞 BLE 扫描和连接。
    hw::gamepad::init();
    hw::wifi::init();
    io::web::init();
    io::host::init();
    create_gamepad_task();

    hw::battery::init();
    hw::indicator::init();
    hw::servo::init();
    hw::imu::init();
    hw::motor::init();
    control::init();

    application_ready.store(create_control_tasks(), std::memory_order_release);
}
