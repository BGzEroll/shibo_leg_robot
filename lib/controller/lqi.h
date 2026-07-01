#ifndef LQI_H
#define LQI_H

#include <Arduino.h>

namespace lqi
{
    struct car_model
    {
        float r;
        float base_height;
        float leg_max_height;
        float leg_min_height;
    };

    struct speed_limit
    {
        float max_linear_vel;
        float max_steer_vel;
    };

    struct feedback_state
    {
        float pitch_angle;
        float pitch_rate;
        float avg_linear_pos;
        float avg_linear_vel;
        float yaw_angle;
        float yaw_rate;
    };

    struct reference_state
    {
        float linear_vel;
        float yaw_rate;
    };

    struct integral_state
    {
        float linear_vel_error;
        float yaw_rate_error;
    };

    struct integral_limit
    {
        float linear_vel_error;
        float yaw_rate_error;
    };

    extern car_model car;
    extern speed_limit limit;
    extern feedback_state state;
    extern reference_state ref;
    extern integral_state integral;
    extern integral_limit integral_clamp;
    extern float gain_poly[12][4];
    extern float feedback_gain[2][6];
}

#endif
