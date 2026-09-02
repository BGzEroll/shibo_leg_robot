#include "indicator.h"

#include "FastLED.h"
#include "led.h"

/* ---- 指示灯硬件配置与运行状态 ---- */

static constexpr uint8_t RGB_PIN = 21;
static constexpr uint8_t RGB_LED_COUNT = 2;
static constexpr uint8_t RGB_GLOBAL_BRIGHTNESS = 50;
static constexpr uint32_t UPDATE_PERIOD_MS = 50;
static constexpr uint32_t NORMAL_PERIOD_MS = 1000;
static constexpr uint32_t LOW_PERIOD_MS = 200;

static led board_led(13);
static CRGB rgb_leds[RGB_LED_COUNT];
static bool red_enabled = false;
static bool last_low = false;
static uint32_t update_timer_ms = 0;
static uint32_t phase_ms = 0;

/**
 * @brief 设置两颗 RGB 灯为红色或熄灭
 *
 * @param enabled 是否点亮红灯
 */
static void set_red(bool enabled)
{
    if(enabled == red_enabled){return;}

    red_enabled = enabled;
    fill_solid(rgb_leds, RGB_LED_COUNT, enabled ? CRGB::Red : CRGB::Black);
    FastLED.show();
}

/**
 * @brief 判断低电双闪当前相位是否需要点亮
 *
 * @param value_ms 当前低电灯效相位，单位毫秒
 *
 * @return 当前相位需要点亮时返回 true
 */
static bool low_flash_active(uint32_t value_ms)
{
    return value_ms < 100 || (value_ms >= 200 && value_ms < 300);
}

/**
 * @brief 更新板载 LED 和 RGB 状态
 *
 * @param battery_low 是否处于低电状态
 * @param tick_ms 距离上次更新的时间，单位毫秒
 */
void hw::indicator::update(bool battery_low, uint32_t tick_ms)
{
    update_timer_ms += tick_ms;
    if(update_timer_ms < UPDATE_PERIOD_MS){return;}
    update_timer_ms = 0;

    if(battery_low != last_low)
    {
        phase_ms = 0;
        last_low = battery_low;
    }

    uint32_t board_phase_ms = battery_low ? phase_ms % LOW_PERIOD_MS : phase_ms;
    if(board_phase_ms < (battery_low ? 100U : 50U))
    {
        board_led.on();
    }
    else
    {
        board_led.off();
    }

    set_red(battery_low && low_flash_active(phase_ms));
    phase_ms = (phase_ms + UPDATE_PERIOD_MS) % NORMAL_PERIOD_MS;
}

/**
 * @brief 初始化板载 LED 和 RGB 灯
 */
void hw::indicator::init()
{
    board_led.init();
    FastLED.addLeds<WS2812, RGB_PIN, GRB>(rgb_leds, RGB_LED_COUNT);
    FastLED.setBrightness(RGB_GLOBAL_BRIGHTNESS);
    fill_solid(rgb_leds, RGB_LED_COUNT, CRGB::Black);
    FastLED.show();
    red_enabled = false;
    last_low = false;
    update_timer_ms = 0;
    phase_ms = 0;
}
