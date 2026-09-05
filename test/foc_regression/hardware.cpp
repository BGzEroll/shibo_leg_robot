#include "drivers/hardware_api.h"

#include "soc/mcpwm_struct.h"
#include <cassert>
#include <cstdio>

uint32_t test_time_us = 1000;
uint32_t test_pwm_writes = 0;
bool test_enabled = false;

/** @brief 比较实际后端的资源分配、计时器配置和 PWM 写入调用 */
int main()
{
    const int32_t frequencies[4] = {0, -12345, 33000, 60000};
    for(uint8_t index = 0; index < 4; index++)
    {
        void *params = _configure3PWM(frequencies[index], 10 + index * 3,
            11 + index * 3, 12 + index * 3);
        assert(params != SIMPLEFOC_DRIVER_INIT_FAILED);
        const mcpwm_dev_t &device = index < 2 ? MCPWM0 : MCPWM1;
        std::printf("clock %u %u\n", device.clk_cfg.clk_prescale, test_time_us);
        for(uint8_t timer = 0; timer < 3; timer++)
        {
            const auto &config = device.timer[timer].timer_cfg0;
            std::printf("timer %u %u %u\n", config.timer_prescale,
                config.timer_period, config.timer_period_upmethod);
        }
        _writeDutyCycle3PWM(0, 0.5f, 1.0f, params);
    }
#ifdef FOC_TRIMMED
    // 原版资源耗尽会越界；裁剪版必须在访问资源前返回失败。
    assert(_configure3PWM(0, 1, 2, 3) == SIMPLEFOC_DRIVER_INIT_FAILED);
#endif
}
