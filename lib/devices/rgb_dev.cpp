#include "rgb_dev.h"

#include "FastLED.h"
#include "freertos/task.h"

/* ---- RGB 硬件配置与运行状态 ---- */

static constexpr uint8_t RGB_PIN = 21;
static constexpr uint8_t RGB_LED_COUNT = 2;
static constexpr uint8_t RGB_GLOBAL_BRIGHTNESS = 50;
static constexpr uint32_t TASK_PERIOD_MS = 50;
static constexpr uint32_t DOUBLE_FLASH_PERIOD_MS = 1000;

static CRGB rgb_leds[RGB_LED_COUNT];
static bool red_enabled = false;
static rgb_dev::input_ports module_inputs;

/* ---- 低电双闪内部流程 ---- */

/**
 * @brief 查询当前是否处于低电状态
 *
 * @return 电池状态有效且低电时返回 true
 */
static bool battery_low()
{
    battery::data data;
    if(!module_inputs.battery_status.read(data)){return false;}
    return data.valid && data.low;
}

/**
 * @brief 设置两颗 RGB 灯为红色或熄灭
 *
 * @param enabled 是否点亮红灯
 */
static void set_red(bool enabled)
{
    if(enabled == red_enabled){return;}

    red_enabled = enabled;
    fill_solid(
        rgb_leds,
        RGB_LED_COUNT,
        enabled ? CRGB::Red : CRGB::Black);
    FastLED.show();
}

/**
 * @brief 判断当前双闪相位是否需要点亮红灯
 *
 * @param phase_ms 当前灯效相位，单位毫秒
 *
 * @return 当前相位需要点亮时返回 true
 */
static bool red_phase_active(uint32_t phase_ms)
{
    return phase_ms < 100 ||
           (phase_ms >= 200 && phase_ms < 300);
}

/* ---- rgb_dev 公共 API ---- */

/**
 * @brief 初始化两颗 WS2812 RGB 灯及端口
 *
 * @param inputs RGB 输入端口
 */
void rgb_dev::init(const rgb_dev::input_ports &inputs)
{
    module_inputs = inputs;
    FastLED.addLeds<WS2812, RGB_PIN, GRB>(rgb_leds, RGB_LED_COUNT);
    FastLED.setBrightness(RGB_GLOBAL_BRIGHTNESS);
    fill_solid(rgb_leds, RGB_LED_COUNT, CRGB::Black);
    FastLED.show();
    red_enabled = false;
}

/**
 * @brief 低电红灯双闪任务入口
 *
 * @param arg RTOS 任务参数
 */
void rgb_dev::task_entry(void *arg)
{
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

        set_red(low && red_phase_active(phase_ms));
        if(low)
        {
            phase_ms = (phase_ms + TASK_PERIOD_MS) % DOUBLE_FLASH_PERIOD_MS;
        }
        else
        {
            phase_ms = 0;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}
