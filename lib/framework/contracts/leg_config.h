#ifndef LEG_CONFIG_H
#define LEG_CONFIG_H

#include <Arduino.h>

namespace leg_contract
{
    static constexpr float HEIGHT_BASE = 20.0f;
    static constexpr int16_t LEFT_MIN = 2048 + 40;
    static constexpr int16_t RIGHT_MIN = 2048 - 40;
    static constexpr int16_t LEFT_MAX = 2048 + 450;
    static constexpr int16_t RIGHT_MAX = 2048 - 450;
}

#endif
