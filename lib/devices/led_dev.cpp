#include "led_dev.h"

led led_dev::board_led(13);

/**
 * @brief 执行一次板载 LED 闪烁周期
 */
static void blink()
{
    led_dev::board_led.on();
    delay(50);
    led_dev::board_led.off();
    delay(950);
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

    while(true)
    {
        blink();
    }
}
