#include "motor_pwm.h"

#include "driver/gpio.h"
#include "hal/mcpwm_ll.h"
#include "esp_rom_sys.h"

static constexpr uint32_t PWM_PERIOD = 3200; // 160 MHz / (2 * 25 kHz)。

/** @brief 保存 MCPWM0 的 A/B 输出组及固定引脚 */
motor_pwm::motor_pwm(mcpwm_operator_t output, gpio_num_t a, gpio_num_t b,
    gpio_num_t c, gpio_num_t enable)
    : output(output), pins{a, b, c}, enable(enable)
{
}

/** @brief 一次性初始化双电机共用的三相中心对齐 25 kHz 定时器 */
bool motor_pwm::init_timers()
{
    mcpwm_config_t config{};
    config.frequency = 50000;
    config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    config.duty_mode = MCPWM_DUTY_MODE_0;
    for(uint8_t i = 0; i < 3; i++)
    {
        if(mcpwm_init(MCPWM_UNIT_0, (mcpwm_timer_t)i, &config) != ESP_OK)
        {
            return false;
        }
        mcpwm_stop(MCPWM_UNIT_0, (mcpwm_timer_t)i);
    }
    MCPWM0.clk_cfg.clk_prescale = 0;
    mcpwm_sync_config_t sync{};
    sync.sync_sig = MCPWM_SELECT_TIMER0_SYNC;
    sync.count_direction = MCPWM_TIMER_DIRECTION_UP;
    for(uint8_t i = 0; i < 3; i++)
    {
        MCPWM0.timer[i].timer_cfg0.timer_prescale = 0;
        MCPWM0.timer[i].timer_cfg0.timer_period = PWM_PERIOD;
        MCPWM0.timer[i].timer_cfg0.timer_period_upmethod = 0;
        // 三相 A/B 比较值都在同步零点装载。
        MCPWM0.operators[i].gen_stmp_cfg.gen_a_upmethod = 1;
        MCPWM0.operators[i].gen_stmp_cfg.gen_b_upmethod = 1;
        if(mcpwm_sync_configure(MCPWM_UNIT_0, (mcpwm_timer_t)i, &sync) != ESP_OK ||
           mcpwm_start(MCPWM_UNIT_0, (mcpwm_timer_t)i) != ESP_OK)
        {
            return false;
        }
    }
    return mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0,
        MCPWM_SWSYNC_SOURCE_TEZ) == ESP_OK;
}

/** @brief 在连接 PWM 引脚前拉低本电机 EN */
bool motor_pwm::init()
{
    gpio_set_level(enable, 0);
    if(gpio_set_direction(enable, GPIO_MODE_OUTPUT) != ESP_OK){return false;}
    for(uint8_t i = 0; i < 3; i++)
    {
        if(mcpwm_gpio_init(MCPWM_UNIT_0,
            (mcpwm_io_signals_t)(2 * i + output), pins[i]) != ESP_OK)
        {
            return false;
        }
    }
    return true;
}

/** @brief 使能前等待比较值装载；关闭时立即拉低 EN */
void motor_pwm::set_enabled(bool value)
{
    if(value == enabled){return;}
    if(value){esp_rom_delay_us(50);}
    gpio_set_level(enable, value);
    enabled = value;
}

/** @brief 通过 IDF LL 写入 Q15 占空比，禁止三相更新中途装载 */
void motor_pwm::write(const foc_motor::duty &duty)
{
    MCPWM0.update_cfg.global_up_en = 0;
    for(uint8_t i = 0; i < 3; i++)
    {
        mcpwm_ll_operator_set_compare_value(&MCPWM0, i, output,
            ((uint32_t)duty.phase[i] * PWM_PERIOD + 16384) >> 15);
    }
    MCPWM0.update_cfg.global_up_en = 1;
}
