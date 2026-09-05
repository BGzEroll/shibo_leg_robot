#include "esp32_mcu.h"

#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) && defined(SOC_MCPWM_SUPPORTED) && !defined(SIMPLEFOC_ESP32_USELEDC)

/* ---- MCPWM 资源与参数 ---- */

static constexpr int32_t EMPTY_SLOT = -20;
static constexpr float MCPWM_FREQUENCY = 160e6f;
static constexpr int32_t PWM_RESOLUTION_DEFAULT = 4096;
static constexpr int32_t PWM_RESOLUTION_MIN = 3000;
static constexpr int32_t PWM_RESOLUTION_MAX = 8000;
static constexpr int32_t PWM_FREQUENCY = 25000;
static constexpr int32_t PWM_FREQUENCY_MAX = 50000;

struct motor_slot
{
    int32_t pin_a;
    mcpwm_dev_t *device;
    mcpwm_unit_t unit;
    mcpwm_operator_t pwm_operator;
    mcpwm_io_signals_t signal_a;
    mcpwm_io_signals_t signal_b;
    mcpwm_io_signals_t signal_c;
};

struct driver_params
{
    mcpwm_unit_t unit;
    mcpwm_operator_t pwm_operator;
};

// 保持原分配顺序：前两台电机共用 MCPWM0 的计时器，分别使用 A/B 输出。
static motor_slot motor_slots[4] =
{
    {EMPTY_SLOT, &MCPWM0, MCPWM_UNIT_0, MCPWM_OPR_A, MCPWM0A, MCPWM1A, MCPWM2A},
    {EMPTY_SLOT, &MCPWM0, MCPWM_UNIT_0, MCPWM_OPR_B, MCPWM0B, MCPWM1B, MCPWM2B},
    {EMPTY_SLOT, &MCPWM1, MCPWM_UNIT_1, MCPWM_OPR_A, MCPWM0A, MCPWM1A, MCPWM2A},
    {EMPTY_SLOT, &MCPWM1, MCPWM_UNIT_1, MCPWM_OPR_B, MCPWM0B, MCPWM1B, MCPWM2B}
};

/* ---- 计时器配置 ---- */

/**
 * @brief 配置中心对齐 PWM，并让三相计时器同步到 timer0 的零点
 *
 * @param pwm_frequency PWM 频率，单位 Hz
 * @param mcpwm_num MCPWM 寄存器组
 * @param mcpwm_unit MCPWM 单元
 */
static void configure_timer_frequency(int32_t pwm_frequency,
    mcpwm_dev_t *mcpwm_num, mcpwm_unit_t mcpwm_unit)
{
    // 初始化比较值为零；随后仍按原顺序重设分频、周期和同步关系。
    mcpwm_config_t pwm_config{};
    pwm_config.counter_mode = MCPWM_UP_DOWN_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    pwm_config.frequency = 2 * pwm_frequency;
    mcpwm_init(mcpwm_unit, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(mcpwm_unit, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(mcpwm_unit, MCPWM_TIMER_2, &pwm_config);
    _delay(100);

    mcpwm_stop(mcpwm_unit, MCPWM_TIMER_0);
    mcpwm_stop(mcpwm_unit, MCPWM_TIMER_1);
    mcpwm_stop(mcpwm_unit, MCPWM_TIMER_2);
    mcpwm_num->clk_cfg.clk_prescale = 0;

    // 保留原分频和分辨率计算精度，避免改变 PWM 频率。
    int32_t prescaler = ceil((double)MCPWM_FREQUENCY /
        (double)PWM_RESOLUTION_DEFAULT / 2.0f / (double)pwm_frequency) - 1;
    prescaler = _constrain(prescaler, 0, 128);
    int32_t resolution_corrected = (double)MCPWM_FREQUENCY / 2.0f /
        (double)pwm_frequency / (double)(prescaler + 1);
    if(resolution_corrected < PWM_RESOLUTION_MIN && prescaler > 0)
    {
        resolution_corrected = (double)MCPWM_FREQUENCY / 2.0f /
            (double)pwm_frequency / (double)(--prescaler + 1);
    }
    resolution_corrected = _constrain(
        resolution_corrected, PWM_RESOLUTION_MIN, PWM_RESOLUTION_MAX);

    mcpwm_num->timer[0].timer_cfg0.timer_prescale = prescaler;
    mcpwm_num->timer[1].timer_cfg0.timer_prescale = prescaler;
    mcpwm_num->timer[2].timer_cfg0.timer_prescale = prescaler;
    _delay(1);
    mcpwm_num->timer[0].timer_cfg0.timer_period = resolution_corrected;
    mcpwm_num->timer[1].timer_cfg0.timer_period = resolution_corrected;
    mcpwm_num->timer[2].timer_cfg0.timer_period = resolution_corrected;
    _delay(1);
    mcpwm_num->timer[0].timer_cfg0.timer_period_upmethod = 0;
    mcpwm_num->timer[1].timer_cfg0.timer_period_upmethod = 0;
    mcpwm_num->timer[2].timer_cfg0.timer_period_upmethod = 0;
    _delay(1);
    mcpwm_start(mcpwm_unit, MCPWM_TIMER_0);
    mcpwm_start(mcpwm_unit, MCPWM_TIMER_1);
    mcpwm_start(mcpwm_unit, MCPWM_TIMER_2);
    _delay(1);

    mcpwm_sync_config_t sync_config =
    {
        .sync_sig = MCPWM_SELECT_TIMER0_SYNC,
        .timer_val = 0,
        .count_direction = MCPWM_TIMER_DIRECTION_UP
    };
    mcpwm_sync_configure(mcpwm_unit, MCPWM_TIMER_0, &sync_config);
    mcpwm_sync_configure(mcpwm_unit, MCPWM_TIMER_1, &sync_config);
    mcpwm_sync_configure(mcpwm_unit, MCPWM_TIMER_2, &sync_config);
    mcpwm_set_timer_sync_output(mcpwm_unit, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);
}

/* ---- 3PWM 硬件接口 ---- */

/**
 * @brief 按原顺序分配三相 PWM 资源并连接引脚
 *
 * @param pwm_frequency PWM 频率，单位 Hz；0 或 NOT_SET 使用默认值
 * @param pin_a A 相引脚
 * @param pin_b B 相引脚
 * @param pin_c C 相引脚
 *
 * @return 驱动参数；资源耗尽返回 SIMPLEFOC_DRIVER_INIT_FAILED
 */
void *_configure3PWM(long pwm_frequency, int pin_a, int pin_b, int pin_c)
{
    if(!pwm_frequency || !_isset(pwm_frequency))
    {
        pwm_frequency = PWM_FREQUENCY;
    }
    else
    {
        pwm_frequency = _constrain(pwm_frequency, 0, PWM_FREQUENCY_MAX);
    }

    motor_slot *slot = nullptr;
    for(uint8_t i = 0; i < 4; i++)
    {
        if(motor_slots[i].pin_a == EMPTY_SLOT)
        {
            motor_slots[i].pin_a = pin_a;
            slot = &motor_slots[i];
            break;
        }
    }
    if(!slot){return SIMPLEFOC_DRIVER_INIT_FAILED;}

    mcpwm_gpio_init(slot->unit, slot->signal_a, pin_a);
    mcpwm_gpio_init(slot->unit, slot->signal_b, pin_b);
    mcpwm_gpio_init(slot->unit, slot->signal_c, pin_c);
    configure_timer_frequency(pwm_frequency, slot->device, slot->unit);

    return new driver_params{slot->unit, slot->pwm_operator};
}

/**
 * @brief 将三相归一化占空比写入 MCPWM
 *
 * @param dc_a A 相占空比，范围 0 至 1
 * @param dc_b B 相占空比，范围 0 至 1
 * @param dc_c C 相占空比，范围 0 至 1
 * @param params 资源分配时返回的驱动参数
 */
void _writeDutyCycle3PWM(float dc_a, float dc_b, float dc_c, void *params)
{
    const driver_params *driver = static_cast<const driver_params *>(params);
    mcpwm_set_duty(driver->unit, MCPWM_TIMER_0, driver->pwm_operator, dc_a * 100.0);
    mcpwm_set_duty(driver->unit, MCPWM_TIMER_1, driver->pwm_operator, dc_b * 100.0);
    mcpwm_set_duty(driver->unit, MCPWM_TIMER_2, driver->pwm_operator, dc_c * 100.0);
}

#endif
