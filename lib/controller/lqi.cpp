#include "lqi.h"

namespace lqi
{
    void init();
}

static const float lqi_gain_poly[12][4] =
{
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

lqi::car_model lqi::car;
lqi::speed_limit lqi::limit;
lqi::feedback_state lqi::state;
lqi::reference_state lqi::ref;
lqi::integral_state lqi::integral;
lqi::integral_limit lqi::integral_clamp;
float lqi::gain_poly[12][4];
float lqi::feedback_gain[2][6];

/**
 * @brief 初始化 LQI 模型参数和运行状态
 */
void lqi::init()
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
