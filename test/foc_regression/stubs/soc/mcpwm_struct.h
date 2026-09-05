#ifndef TEST_MCPWM_STRUCT_H
#define TEST_MCPWM_STRUCT_H

#include <cstdint>

struct mcpwm_dev_t
{
    struct
    {
        uint32_t clk_prescale;
    } clk_cfg;
    struct
    {
        struct
        {
            uint32_t timer_prescale;
            uint32_t timer_period;
            uint32_t timer_period_upmethod;
        } timer_cfg0;
    } timer[3];
};

inline mcpwm_dev_t MCPWM0{};
inline mcpwm_dev_t MCPWM1{};

#endif
