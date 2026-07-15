#include "led_dev.h"

#include "battery.h"
#include "freertos/task.h"

led led_dev::board_led(13);

/**
 * @brief 查询当前是否处于低电状态
 *
 * @return 电池状态有效且低电时返回 true
 */
static bool battery_low()
{
    battery::data data;
    if(!battery::peek_data(data))
    {
        return false;
    }
    return data.valid && data.low;
}

/**
 * @brief 按当前电池状态和灯效相位更新 LED
 *
 * @param low 是否处于低电状态
 * @param phase_ms 当前灯效相位，单位毫秒
 */
static void update_led(bool low, uint32_t phase_ms)
{
    uint32_t on_duration_ms = low ? 100 : 50;
    if(phase_ms < on_duration_ms)
    {
        led_dev::board_led.on();
    }
    else
    {
        led_dev::board_led.off();
    }
}

/**
 * @brief 初始化板载 LED 设备
 */
void led_dev::init()
{
    led_dev::board_led.init();
}

/**
 * @brief 板载 LED 闪烁任务入口
 *
 * @param arg RTOS 任务参数
 */
void led_dev::task_entry(void *arg)
{
    static constexpr uint32_t TASK_PERIOD_MS = 50;
    static constexpr uint32_t NORMAL_PERIOD_MS = 1000;
    static constexpr uint32_t LOW_PERIOD_MS = 200;

    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t phase_ms = 0;
    bool last_low = false;
    while(true)
    {
        bool low = battery_low();
        if(low != last_low)
        {
            phase_ms = 0;
            last_low = low;
        }

        update_led(low, phase_ms);
        uint32_t period_ms = low ? LOW_PERIOD_MS : NORMAL_PERIOD_MS;
        phase_ms = (phase_ms + TASK_PERIOD_MS) % period_ms;
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}
