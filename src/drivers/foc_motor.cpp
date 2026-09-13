#include "foc_motor.h"

#include <algorithm>

static constexpr int32_t Q15_ONE = 32768;
static constexpr int32_t OUTPUT_LIMIT = 18919;
static constexpr int32_t POLE_PAIRS = 7;
static constexpr uint32_t SPEED_FILTER_US = 3000;
static constexpr uint32_t MAX_PREDICT_US = 1000;

// 本机固定参数：星接线间 21.2 Ω，单相 10.6 Ω，Kt=Ke=0.0796，母线 8 V。
static constexpr float PHASE_RESISTANCE = 21.2f / 2.0f;
static constexpr float KT = 0.0796f;
static constexpr float KE = 0.0796f;
static constexpr float BUS_VOLTAGE = 8.0f;
static constexpr int32_t TORQUE_GAIN_Q16 = (int32_t)(
    PHASE_RESISTANCE / KT / BUS_VOLTAGE * 32768.0f / 1000000.0f * 65536.0f + 0.5f);
static constexpr int32_t BEMF_GAIN_Q14 = (int32_t)(
    KE / BUS_VOLTAGE * 32768.0f / 1000.0f * 16384.0f + 0.5f);

/** @brief STM32 main 的四分之一周期正弦查表与线性插值 */
static int32_t lookup_sin(uint16_t phase)
{
    static const uint16_t table[65] =
    {
        0, 804, 1608, 2411, 3212, 4011, 4808, 5602,
        6393, 7180, 7962, 8740, 9512, 10279, 11039, 11793,
        12540, 13279, 14010, 14733, 15447, 16151, 16846, 17531,
        18205, 18868, 19520, 20160, 20788, 21403, 22006, 22595,
        23170, 23732, 24279, 24812, 25330, 25833, 26320, 26791,
        27246, 27684, 28106, 28511, 28899, 29269, 29622, 29957,
        30274, 30572, 30853, 31114, 31357, 31581, 31786, 31972,
        32138, 32286, 32413, 32522, 32610, 32679, 32729, 32758,
        32768
    };

    const uint16_t index = phase >> 8;
    const int32_t fraction = phase & 0xFF;

    int32_t a, b;

    if(index < 64)
    {
        a = table[index];
        b = table[index + 1];
    }
    else if(index < 128)
    {
        a = table[128 - index];
        b = table[127 - index];
    }
    else if(index < 192)
    {
        a = -table[index - 128];
        b = -table[index - 127];
    }
    else
    {
        a = -table[256 - index];
        b = -table[255 - index];
    }

    return a + (((b - a) * fraction) >> 8);
}


/** @brief 每个新样本只更新一次跨圈计数和 3 ms 一阶速度滤波 */
void foc_motor::sample(uint16_t raw, uint32_t sample_us)
{
    if(sampled)
    {
        const uint32_t dt_us = sample_us - timestamp_us;
        if(!dt_us){return;}
        int32_t delta = (int32_t)raw - last_raw;
        if(delta > 2048){delta -= 4096;}
        else if(delta < -2048){delta += 4096;}
        full_count += delta;
        // 失联恢复时重新建立速度基准，避免跨越长间隔预测。
        speed_mrad_s = dt_us > 5000 ? 0 : (int32_t)(
            ((int64_t)speed_mrad_s * SPEED_FILTER_US + (int64_t)delta * 1533981) /
            (SPEED_FILTER_US + dt_us));
    }
    else
    {
        full_count = raw;
        sampled = true;
    }
    last_raw = raw;
    timestamp_us = sample_us;
}

/** @brief 保存独立的编码器方向及零电角 */
void foc_motor::align(int8_t sensor_direction, uint16_t mechanical_phase)
{
    direction = sensor_direction;
    zero_phase = (uint16_t)((int32_t)direction * POLE_PAIRS * mechanical_phase);
    speed_mrad_s = 0;
}

/** @brief 力矩单位为微牛米；预测角度并计算当前 PWM，不重复推进速度滤波 */
foc_motor::duty foc_motor::update(int32_t torque_uNm, uint32_t now_us) const
{
    const uint32_t age_us = std::min(now_us - timestamp_us, MAX_PREDICT_US);
    const int32_t advance = (int32_t)(((int64_t)speed_mrad_s * age_us * 175) >> 24);
    const uint16_t mechanical = (uint16_t)(((uint32_t)last_raw << 4) + advance);
    const uint16_t electrical = (uint16_t)(
        (int32_t)direction * POLE_PAIRS * mechanical - zero_phase);
    const int64_t uq = ((int64_t)torque_uNm * TORQUE_GAIN_Q16 >> 16) +
        ((int64_t)direction * speed_mrad_s * BEMF_GAIN_Q14 >> 14);
    return svpwm((int32_t)std::max(-(int64_t)OUTPUT_LIMIT,
        std::min((int64_t)OUTPUT_LIMIT, uq)), electrical);
}

/** @brief Q15 逆 Park、逆 Clarke 和零序注入，输出三相归一化占空比 */
foc_motor::duty foc_motor::svpwm(int32_t uq, uint16_t phase)
{
    uq = std::max(-OUTPUT_LIMIT, std::min(OUTPUT_LIMIT, uq));
    const int32_t alpha = -((lookup_sin(phase) * uq) >> 15);
    const int32_t beta = (lookup_sin((uint16_t)(phase + 0x4000)) * uq) >> 15;
    const int32_t phase_b = -(alpha >> 1) + ((28378 * beta) >> 15);
    const int32_t phase_c = -(alpha >> 1) - ((28378 * beta) >> 15);
    const int32_t offset = Q15_ONE / 2 -
        ((std::max(alpha, std::max(phase_b, phase_c)) +
          std::min(alpha, std::min(phase_b, phase_c))) >> 1);
    duty result{};
    const int32_t values[] = {alpha, phase_b, phase_c};
    for(uint8_t i = 0; i < 3; i++)
    {
        result.phase[i] = (uint16_t)std::max(0, std::min(Q15_ONE, values[i] + offset));
    }
    return result;
}
