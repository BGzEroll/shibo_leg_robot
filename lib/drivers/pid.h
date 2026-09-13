#ifndef PID_H
#define PID_H

#include <stdint.h>

/** @brief 带积分限幅和输出斜率限制的 PID 控制器 */
class pid
{
    public:
        pid(float p, float i, float d, float ramp, float limit);
        ~pid() = default;

    public:
        float operator()(float error);

    public:
        float p;
        float i;
        float d;
        float output_ramp;
        float limit;

    protected:
        float error_prev;
        float output_prev;
        float integral_prev;
        uint32_t timestamp_prev;
};

#endif
