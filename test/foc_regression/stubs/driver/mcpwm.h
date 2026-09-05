#ifndef TEST_MCPWM_H
#define TEST_MCPWM_H

#include <cassert>
#include <cstdio>
#include <cstdint>

enum mcpwm_unit_t { MCPWM_UNIT_0, MCPWM_UNIT_1 };
enum mcpwm_operator_t { MCPWM_OPR_A, MCPWM_OPR_B };
enum mcpwm_io_signals_t { MCPWM0A, MCPWM0B, MCPWM1A, MCPWM1B, MCPWM2A, MCPWM2B };
enum mcpwm_timer_t { MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_2 };
constexpr int MCPWM_UP_DOWN_COUNTER = 2;
constexpr int MCPWM_DUTY_MODE_0 = 0;
constexpr int MCPWM_SELECT_TIMER0_SYNC = 1;
constexpr int MCPWM_TIMER_DIRECTION_UP = 0;
constexpr int MCPWM_SWSYNC_SOURCE_TEZ = 0;
constexpr int MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE = 0;

struct mcpwm_config_t
{
    uint32_t frequency;
    float cmpr_a;
    float cmpr_b;
    int duty_mode;
    int counter_mode;
};

struct mcpwm_sync_config_t
{
    int sync_sig;
    uint32_t timer_val;
    int count_direction;
};

inline void mcpwm_init(mcpwm_unit_t unit, mcpwm_timer_t timer, const mcpwm_config_t *config)
{
#ifdef FOC_TRIMMED
    assert(config->cmpr_a == 0 && config->cmpr_b == 0);
#endif
    std::printf("init %d %d %u %d %d\n", unit, timer, config->frequency,
        config->duty_mode, config->counter_mode);
}
inline void mcpwm_stop(mcpwm_unit_t unit, mcpwm_timer_t timer)
{
    std::printf("stop %d %d\n", unit, timer);
}
inline void mcpwm_start(mcpwm_unit_t unit, mcpwm_timer_t timer)
{
    std::printf("start %d %d\n", unit, timer);
}
inline void mcpwm_sync_configure(mcpwm_unit_t unit, mcpwm_timer_t timer,
    const mcpwm_sync_config_t *config)
{
    std::printf("sync %d %d %d %u %d\n", unit, timer,
        config->sync_sig, config->timer_val, config->count_direction);
}
inline void mcpwm_set_timer_sync_output(mcpwm_unit_t unit, mcpwm_timer_t timer, int source)
{
    std::printf("sync_out %d %d %d\n", unit, timer, source);
}
inline void mcpwm_gpio_init(mcpwm_unit_t unit, mcpwm_io_signals_t signal, int pin)
{
    std::printf("gpio %d %d %d\n", unit, signal, pin);
}
inline void mcpwm_set_duty(mcpwm_unit_t unit, mcpwm_timer_t timer,
    mcpwm_operator_t pwm_operator, float duty)
{
    std::printf("duty %d %d %d %a\n", unit, timer, pwm_operator, duty);
}
inline void mcpwm_deadtime_enable(mcpwm_unit_t, mcpwm_timer_t, int, float, float) {}

#endif
