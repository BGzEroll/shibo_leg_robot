#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <stdint.h>

/** @brief 根据实测时间差更新的一阶低通滤波器 */
class LowPassFilter
{
    public:
        LowPassFilter(float time_constant);
        ~LowPassFilter() = default;

    public:
        float operator()(float input);

    public:
        float Tf;

    protected:
        uint32_t timestamp_prev;
        float y_prev;
};

#endif
