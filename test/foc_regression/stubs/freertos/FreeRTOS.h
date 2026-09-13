#pragma once
#include <stdint.h>
using TickType_t = uint32_t;
constexpr uint32_t configTICK_RATE_HZ = 1000;
constexpr int pdTRUE = 1;
#define pdMS_TO_TICKS(ms) (ms)
