#pragma once
#include "esp_timer.h"
inline void esp_rom_delay_us(uint32_t us){fake_time_us += us;}
