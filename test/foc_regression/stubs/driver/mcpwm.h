#pragma once
#include "gpio.h"
enum mcpwm_operator_t {MCPWM_OPR_A, MCPWM_OPR_B};
enum mcpwm_io_signals_t {MCPWM0A, MCPWM0B, MCPWM1A, MCPWM1B, MCPWM2A, MCPWM2B};
enum mcpwm_timer_t {MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_2};
constexpr int MCPWM_UNIT_0 = 0, MCPWM_UP_DOWN_COUNTER = 2, MCPWM_DUTY_MODE_0 = 0;
constexpr int MCPWM_SELECT_TIMER0_SYNC = 1, MCPWM_TIMER_DIRECTION_UP = 0;
constexpr int MCPWM_SWSYNC_SOURCE_TEZ = 1;
struct mcpwm_config_t {int frequency, counter_mode, duty_mode;};
struct mcpwm_sync_config_t {int sync_sig, count_direction;};
inline int mcpwm_init(int, mcpwm_timer_t, const mcpwm_config_t *){return 0;}
inline int mcpwm_stop(int, mcpwm_timer_t){return 0;}
inline int mcpwm_start(int, mcpwm_timer_t){return 0;}
inline int mcpwm_sync_configure(int, mcpwm_timer_t, const mcpwm_sync_config_t *){return 0;}
inline int mcpwm_set_timer_sync_output(int, mcpwm_timer_t, int){return 0;}
inline int mcpwm_gpio_init(int, mcpwm_io_signals_t, gpio_num_t){return 0;}
