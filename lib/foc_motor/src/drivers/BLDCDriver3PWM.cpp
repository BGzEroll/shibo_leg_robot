#include "BLDCDriver3PWM.h"

#include "hardware_api.h"
#include "../common/defaults.h"

/**
 * @brief 初始化三相引脚及默认供电参数
 *
 * @param phase_a A 相 PWM 引脚
 * @param phase_b B 相 PWM 引脚
 * @param phase_c C 相 PWM 引脚
 * @param enable_pin 公共使能引脚，NOT_SET 表示无独立使能
 */
BLDCDriver3PWM::BLDCDriver3PWM(int phase_a, int phase_b, int phase_c, int enable_pin)
    : pwmA(phase_a),
      pwmB(phase_b),
      pwmC(phase_c),
      enableA_pin(enable_pin)
{
    voltage_power_supply = DEF_POWER_SUPPLY;
    voltage_limit = NOT_SET;
    pwm_frequency = NOT_SET;
}

/** @brief 拉起公共使能并清零 PWM */
void BLDCDriver3PWM::enable()
{
    if(_isset(enableA_pin)){digitalWrite(enableA_pin, enable_active_high);}
    setPwm(0, 0, 0);
}

/** @brief 清零 PWM 并撤销公共使能 */
void BLDCDriver3PWM::disable()
{
    setPwm(0, 0, 0);
    if(_isset(enableA_pin)){digitalWrite(enableA_pin, !enable_active_high);}
}

/**
 * @brief 限制三相电压并转换为占空比
 *
 * @param ua A 相电压，单位 V
 * @param ub B 相电压，单位 V
 * @param uc C 相电压，单位 V
 */
void BLDCDriver3PWM::setPwm(float ua, float ub, float uc)
{
    ua = _constrain(ua, 0.0f, voltage_limit);
    ub = _constrain(ub, 0.0f, voltage_limit);
    uc = _constrain(uc, 0.0f, voltage_limit);
    dc_a = _constrain(ua / voltage_power_supply, 0.0f, 1.0f);
    dc_b = _constrain(ub / voltage_power_supply, 0.0f, 1.0f);
    dc_c = _constrain(uc / voltage_power_supply, 0.0f, 1.0f);
    _writeDutyCycle3PWM(dc_a, dc_b, dc_c, params);
}

/**
 * @brief 配置输出引脚、驱动限压和 MCPWM
 *
 * @return 硬件资源分配成功返回 1，否则返回 0
 */
int BLDCDriver3PWM::init()
{
    pinMode(pwmA, OUTPUT);
    pinMode(pwmB, OUTPUT);
    pinMode(pwmC, OUTPUT);
    if(_isset(enableA_pin)){pinMode(enableA_pin, OUTPUT);}
    if(!_isset(voltage_limit) || voltage_limit > voltage_power_supply)
    {
        voltage_limit = voltage_power_supply;
    }
    params = _configure3PWM(pwm_frequency, pwmA, pwmB, pwmC);
    initialized = params != SIMPLEFOC_DRIVER_INIT_FAILED;
    return initialized;
}
