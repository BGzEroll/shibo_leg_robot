#include "lqi.h"

namespace controller {

static const float lqi_gain_poly[12][4] = {
    { -19.78318794f,  2.96741131f, -3.67412914f, -3.30769108f},
    {  45.57101193f, -13.86222792f, -0.81410746f, -0.10765136f},
    {  848.88274746f, -221.91446497f,  20.87630740f, -1.71800905f},
    { -0.00000000f,  0.00000000f, -0.00000000f, -0.05583265f},
    { -0.00000000f,  0.00000000f, -0.00000000f,  0.84459977f},
    {  0.00000000f, -0.00000000f,  0.00000000f,  0.33502155f},
    { -19.78318794f,  2.96741131f, -3.67412914f, -3.30769108f},
    {  45.57101193f, -13.86222792f, -0.81410746f, -0.10765136f},
    {  848.88274746f, -221.91446497f,  20.87630740f, -1.71800905f},
    {  0.00000000f, -0.00000000f,  0.00000000f,  0.05583265f},
    { -0.00000000f,  0.00000000f, -0.00000000f,  0.84459977f},
    { -0.00000000f,  0.00000000f, -0.00000000f, -0.33502155f}
};

lqi::lqi()
{
    car.r = 0.0526f / 2.0f;
    car.base_height = 0.03f;
    car.leg_max_height = 0.06f;
    car.leg_min_height = 0.02f;

    limit.max_linear_vel = 0.6f;
    limit.max_steer_vel = 2.0f;

    integral_clamp.linear_vel_error = 0.38f * 6.0f;
    integral_clamp.yaw_rate_error = 0.55f;

    memcpy(gain_poly, lqi_gain_poly, sizeof(gain_poly));
    memset(feedback_gain, 0, sizeof(feedback_gain));
    memset(&state, 0, sizeof(state));
    memset(&ref, 0, sizeof(ref));
    memset(&integral, 0, sizeof(integral));
}

}
