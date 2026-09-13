#pragma once
#include <stdint.h>
struct fake_mcpwm
{
    struct {int clk_prescale;} clk_cfg;
    struct {struct {int timer_prescale, timer_period, timer_period_upmethod;} timer_cfg0;} timer[3];
    struct {struct {int gen_a_upmethod, gen_b_upmethod;} gen_stmp_cfg;} operators[3];
    struct {int global_up_en;} update_cfg;
    uint32_t compare[3][2]{};
};
extern fake_mcpwm MCPWM0;
inline void mcpwm_ll_operator_set_compare_value(fake_mcpwm *p, int phase, int axis, uint32_t value)
{
    p->compare[phase][axis] = value;
}
