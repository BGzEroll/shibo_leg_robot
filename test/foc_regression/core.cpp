#include "foc_motor.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <climits>
#include <cstdio>

/** @brief 用双精度三相参考覆盖全相位和正负限幅 */
static void check_svpwm()
{
    const double pi = std::acos(-1.0);
    double max_error = 0;
    for(int32_t uq : {INT_MIN, -18919, -8192, 0, 8192, 18919, INT_MAX})
    {
        for(uint32_t phase = 0; phase < 65536; phase++)
        {
            const auto duty = foc_motor::svpwm(uq, (uint16_t)phase);
            const double angle = phase * 2 * pi / 65536;
            const double voltage = std::max(-18919, std::min(18919, uq));
            const double alpha = -std::sin(angle) * voltage;
            const double beta = std::cos(angle) * voltage;
            double ref[] = {alpha, -0.5 * alpha + std::sqrt(3.0) * 0.5 * beta,
                -0.5 * alpha - std::sqrt(3.0) * 0.5 * beta};
            const double offset = 16384 -
                (*std::max_element(ref, ref + 3) + *std::min_element(ref, ref + 3)) / 2;
            for(uint8_t i = 0; i < 3; i++)
            {
                assert(duty.phase[i] <= 32768);
                max_error = std::max(max_error, std::abs(duty.phase[i] - ref[i] - offset));
                assert(std::abs(duty.phase[i] - ref[i] - offset) < 6);
                if(!uq){assert(duty.phase[i] == 16384);}
            }
        }
    }
    std::printf("SVPWM: 458752 vectors, max Q15 duty error %.3f\n", max_error);
}

/** @brief 检查跨圈、时戳回绕、重复样本、独立实例及物理参数尺度 */
int main()
{
    check_svpwm();
    foc_motor left;
    foc_motor right;
    left.sample(4090, UINT32_MAX - 499);
    left.sample(4, 500);
    assert(left.full_count == 4100 && left.speed_mrad_s > 0);
    const int32_t speed = left.speed_mrad_s;
    left.sample(4, 500);
    assert(left.full_count == 4100 && left.speed_mrad_s == speed);
    right.sample(4, 100);
    right.sample(4090, 1100);
    assert(right.full_count == -6 && right.speed_mrad_s < 0);
    assert(left.full_count == 4100 && left.speed_mrad_s == speed);
    left.align(1, 64);
    right.align(-1, 65440);
    for(int32_t torque : {INT_MIN, -10000, 0, 10000, INT_MAX})
    {
        const auto a = left.update(torque, 500);
        const auto b = right.update(torque, 1100);
        for(uint8_t i = 0; i < 3; i++){assert(a.phase[i] == b.phase[i]);}
    }
    const auto torque = left.update(10000, 500); // 0.01 N·m -> 1.33166 V。
    const double measured_uq = (torque.phase[1] - torque.phase[2]) * 8.0 /
        (32768 * std::sqrt(3.0));
    assert(std::abs(measured_uq - 10.6 / 0.0796 * 0.01) < 0.002);
    left.sample(14, 1500);
    const int32_t filtered_speed = left.speed_mrad_s;
    for(uint32_t i = 0; i < 1000; i++){left.update(10000, 1500 + i);}
    assert(left.speed_mrad_s == filtered_speed);
    const auto capped = left.update(0, 2500);
    const auto later = left.update(0, 4500);
    for(uint8_t i = 0; i < 3; i++){assert(capped.phase[i] == later.phase[i]);}
    left.sample(20, 20000);
    assert(left.speed_mrad_s == 0);
    std::puts("Encoder wrap, timestamp wrap, instance isolation, torque and prediction: PASS");
}
