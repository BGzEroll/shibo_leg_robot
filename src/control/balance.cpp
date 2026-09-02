#include "balance.h"

#include "esp_timer.h"
#include "SimpleFOC.h"
#include <math.h>
#include <string.h>

/* ---- LQI 内部模型与运行状态 ---- */

struct lqi_car_model
{
    float r = 0.0f;
    float base_height = 0.0f;
    float leg_max_height = 0.0f;
    float leg_min_height = 0.0f;
};

struct lqi_speed_limit
{
    float max_linear_vel = 0.0f;
    float max_steer_vel = 0.0f;
};

struct lqi_feedback_state
{
    float pitch_angle = 0.0f;
    float pitch_rate = 0.0f;
    float avg_linear_pos = 0.0f;
    float avg_linear_vel = 0.0f;
    float yaw_angle = 0.0f;
    float yaw_rate = 0.0f;
};

struct lqi_reference_state
{
    float linear_vel = 0.0f;
    float yaw_rate = 0.0f;
};

struct lqi_integral_state
{
    float linear_vel_error = 0.0f;
    float yaw_rate_error = 0.0f;
};

struct lqi_integral_limit
{
    float linear_vel_error = 0.0f;
    float yaw_rate_error = 0.0f;
};

struct lqi_runtime
{
    lqi_car_model car;
    lqi_speed_limit limit;
    lqi_feedback_state state;
    lqi_reference_state reference;
    lqi_integral_state integral;
    lqi_integral_limit integral_clamp;
    float feedback_gain[2][6]{};
};

static const float LQI_GAIN_POLY[12][4] =
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

static lqi_runtime lqi_core;

/**
 * @brief 初始化 LQI 模型参数和运行状态
 */
static void init_lqi()
{
    lqi_core.car.r = 0.0526f / 2.0f;
    lqi_core.car.base_height = 0.03f;
    lqi_core.car.leg_max_height = 0.06f;
    lqi_core.car.leg_min_height = 0.02f;
    lqi_core.limit.max_linear_vel = 0.6f;
    lqi_core.limit.max_steer_vel = 2.0f;
    lqi_core.integral_clamp.linear_vel_error = 0.38f * 6.0f;
    lqi_core.integral_clamp.yaw_rate_error = 0.55f;
    memset(lqi_core.feedback_gain, 0, sizeof(lqi_core.feedback_gain));
    memset(&lqi_core.state, 0, sizeof(lqi_core.state));
    memset(&lqi_core.reference, 0, sizeof(lqi_core.reference));
    memset(&lqi_core.integral, 0, sizeof(lqi_core.integral));
}

struct balance_runtime
{
    control::balance_command command;
    control::status last_status;
    LowPassFilter velocity_filter{0.008f};
    float last_height = 0.0f;
    float lpf_linear_target = 0.0f;
    float lpf_yaw_target = 0.0f;
    float last_linear_target = 0.0f;
    float linear_release_timer = 0.0f;
    bool linear_release = false;
    bool linear_release_hold = false;
    bool first_state = true;
};

static balance_runtime balance_runtime_state;

/* ---- 平衡控制内部流程 ---- */

/**
 * @brief 清空线速度和转向参考以及积分状态
 */
static void reset_reference()
{
    balance_runtime_state.lpf_linear_target = 0.0f;
    balance_runtime_state.last_linear_target = 0.0f;
    balance_runtime_state.linear_release_timer = 0.0f;
    balance_runtime_state.linear_release = false;
    balance_runtime_state.linear_release_hold = false;
    balance_runtime_state.lpf_yaw_target = 0.0f;
    lqi_core.reference.linear_vel = 0.0f;
    lqi_core.reference.yaw_rate = 0.0f;
    lqi_core.integral.linear_vel_error = 0.0f;
    lqi_core.integral.yaw_rate_error = 0.0f;
}

/**
 * @brief 按误差积分并限制积分范围
 *
 * @param value 需要积分的值
 * @param error 当前误差
 * @param dt 时间步长，单位秒
 * @param limit 积分限幅
 */
static void integrate(float &value, float error, float dt, float limit)
{
    value += error * dt;
    value = constrain(value, -limit, limit);
}

/**
 * @brief 根据腿长更新 LQI 反馈增益
 *
 * @param height 当前平均腿长
 */
static void update_gain(float height)
{
    if(fabsf(height - balance_runtime_state.last_height) < 1.0e-4f){return;}
    balance_runtime_state.last_height = height;

    float height_squared = height * height;
    float height_cubed = height_squared * height;
    for(uint8_t i = 0; i < 6; i++)
    {
        lqi_core.feedback_gain[0][i] =
            LQI_GAIN_POLY[i][0] * height_cubed +
            LQI_GAIN_POLY[i][1] * height_squared +
            LQI_GAIN_POLY[i][2] * height +
            LQI_GAIN_POLY[i][3];
        lqi_core.feedback_gain[1][i] =
            LQI_GAIN_POLY[i + 6][0] * height_cubed +
            LQI_GAIN_POLY[i + 6][1] * height_squared +
            LQI_GAIN_POLY[i + 6][2] * height +
            LQI_GAIN_POLY[i + 6][3];
    }
}

/**
 * @brief 更新线速度参考和线速度积分
 *
 * @param dt 时间步长，单位秒
 */
static void update_linear_reference(float dt)
{
    const float tau = 0.024f;
    const float max_accel = 1.60f;
    const float release_duration = 0.30f;
    const float release_hold_speed = 0.03f;
    const float release_resume_speed = 0.02f;

    float target = balance_runtime_state.command.linear_vel;
    bool zero_command = target == 0.0f;
    bool had_command = balance_runtime_state.last_linear_target != 0.0f;
    balance_runtime_state.lpf_linear_target +=
        (target - balance_runtime_state.lpf_linear_target) *
        (1.0f - expf(-dt / tau));

    if(!zero_command)
    {
        balance_runtime_state.linear_release = false;
        balance_runtime_state.linear_release_timer = 0.0f;
        balance_runtime_state.linear_release_hold = false;
    }
    else if(had_command)
    {
        balance_runtime_state.linear_release = true;
        balance_runtime_state.linear_release_timer = 0.0f;
        balance_runtime_state.linear_release_hold = true;
        balance_runtime_state.lpf_linear_target = 0.0f;
    }

    float target_reference = balance_runtime_state.lpf_linear_target;
    bool release_done = false;
    if(balance_runtime_state.linear_release)
    {
        target_reference = 0.0f;
        balance_runtime_state.linear_release_timer += dt;
        if(balance_runtime_state.linear_release_hold &&
           fabsf(lqi_core.state.avg_linear_vel) <= release_hold_speed)
        {
            balance_runtime_state.linear_release_hold = false;
        }
        release_done = fabsf(lqi_core.state.avg_linear_vel) <= release_resume_speed ||
            balance_runtime_state.linear_release_timer >= release_duration;
    }

    if(fabsf(target_reference) > fabsf(lqi_core.reference.linear_vel))
    {
        float delta = target_reference - lqi_core.reference.linear_vel;
        float max_step = max_accel * dt;
        lqi_core.reference.linear_vel += constrain(delta, -max_step, max_step);
    }
    else
    {
        lqi_core.reference.linear_vel = target_reference;
    }

    float linear_error = lqi_core.reference.linear_vel - lqi_core.state.avg_linear_vel;
    if(balance_runtime_state.linear_release)
    {
        if(!balance_runtime_state.linear_release_hold)
        {
            float integral = lqi_core.integral.linear_vel_error;
            float candidate = integral + linear_error * dt;
            if(integral * candidate <= 0.0f)
            {
                lqi_core.integral.linear_vel_error = 0.0f;
            }
            else if(fabsf(candidate) < fabsf(integral))
            {
                lqi_core.integral.linear_vel_error = candidate;
            }
        }
    }
    else
    {
        integrate(lqi_core.integral.linear_vel_error, linear_error, dt,
            lqi_core.integral_clamp.linear_vel_error);
    }

    if(release_done)
    {
        balance_runtime_state.linear_release = false;
        balance_runtime_state.linear_release_timer = 0.0f;
        balance_runtime_state.linear_release_hold = false;
    }
    balance_runtime_state.last_linear_target = target;
}

/**
 * @brief 更新转向角速度参考和转向积分
 *
 * @param dt 时间步长，单位秒
 */
static void update_yaw_reference(float dt)
{
    if(!balance_runtime_state.command.steering)
    {
        balance_runtime_state.lpf_yaw_target = 0.0f;
        lqi_core.reference.yaw_rate = 0.0f;
        lqi_core.integral.yaw_rate_error = 0.0f;
        return;
    }

    const float tau = 0.009f;
    balance_runtime_state.lpf_yaw_target +=
        (balance_runtime_state.command.yaw_rate - balance_runtime_state.lpf_yaw_target) *
        (1.0f - expf(-dt / tau));
    lqi_core.reference.yaw_rate = balance_runtime_state.lpf_yaw_target;

    if(!balance_runtime_state.command.yaw_integral)
    {
        lqi_core.integral.yaw_rate_error = 0.0f;
        return;
    }

    integrate(lqi_core.integral.yaw_rate_error,
        lqi_core.reference.yaw_rate - lqi_core.state.yaw_rate,
        dt, lqi_core.integral_clamp.yaw_rate_error);
}

/**
 * @brief 将传感器快照写入 LQI 状态
 *
 * @param sensor 当前控制周期传感器快照
 */
static void update_state(const control::sensor_snapshot &sensor)
{
    if(sensor.imu_valid)
    {
        lqi_core.state.pitch_angle = sensor.pitch_angle;
        lqi_core.state.pitch_rate = sensor.pitch_rate;
        lqi_core.state.yaw_angle = sensor.yaw_angle;
        lqi_core.state.yaw_rate = sensor.yaw_rate;
    }
    if(sensor.encoder_valid)
    {
        lqi_core.state.avg_linear_pos = sensor.avg_linear_pos;
        lqi_core.state.avg_linear_vel =
            balance_runtime_state.velocity_filter(sensor.avg_linear_vel);
    }
    if(balance_runtime_state.first_state)
    {
        balance_runtime_state.first_state = false;
        reset_reference();
    }
}

/**
 * @brief 根据统一平衡命令计算左右电机输出
 *
 * @param command 统一平衡命令
 * @param status 当前状态输出
 */
static void solve_output(const control::balance_command &command, control::status &status)
{
    status.motor.timestamp_us = (uint32_t)esp_timer_get_time();
    if(command.mode == control::balance_mode::OFF)
    {
        status.output[0] = 0.0f;
        status.output[1] = 0.0f;
        status.motor.left = 0.0f;
        status.motor.right = 0.0f;
        status.motor.enabled = false;
        return;
    }

    status.motor.enabled = true;
    if(command.mode == control::balance_mode::DIRECT)
    {
        status.output[0] = command.direct_left;
        status.output[1] = command.direct_right;
        status.motor.left = command.direct_left;
        status.motor.right = command.direct_right;
        return;
    }

    if(command.mode != control::balance_mode::BALANCE &&
       command.mode != control::balance_mode::RECOVER)
    {
        status.output[0] = 0.0f;
        status.output[1] = 0.0f;
        status.motor.left = 0.0f;
        status.motor.right = 0.0f;
        return;
    }

    float feedback[6] =
    {
        lqi_core.state.pitch_angle,
        lqi_core.state.pitch_rate,
        lqi_core.state.avg_linear_vel - lqi_core.reference.linear_vel,
        lqi_core.state.yaw_rate - lqi_core.reference.yaw_rate,
        lqi_core.integral.linear_vel_error,
        lqi_core.integral.yaw_rate_error
    };

    if(command.mode == control::balance_mode::RECOVER)
    {
        feedback[2] = 0.0f;
        feedback[3] = 0.0f;
        feedback[4] = 0.0f;
        feedback[5] = 0.0f;
    }
    if(!command.linear_feedback){feedback[2] = 0.0f;}
    if(!command.yaw_feedback || !command.steering)
    {
        feedback[3] = 0.0f;
        feedback[5] = 0.0f;
    }
    else if(!command.yaw_integral)
    {
        feedback[5] = 0.0f;
    }

    memcpy(status.feedback_vector, feedback, sizeof(status.feedback_vector));
    float blend = command.mode == control::balance_mode::RECOVER ?
        command.recover_blend : 1.0f;
    for(uint8_t i = 0; i < 2; i++)
    {
        status.output[i] = 0.0f;
        for(uint8_t j = 0; j < 6; j++)
        {
            status.output[i] += lqi_core.feedback_gain[i][j] * feedback[j];
        }
        status.output[i] *= blend;
    }
    status.motor.left = status.output[0];
    status.motor.right = status.output[1];
}

/* ---- balance 公共 API ---- */

/**
 * @brief 获取平衡算法对输入层公开的限幅信息
 *
 * @return 平衡限幅信息
 */
control::info control::balance::get_info()
{
    control::info result;
    result.max_linear_vel = lqi_core.limit.max_linear_vel;
    result.max_steer_vel = lqi_core.limit.max_steer_vel;
    return result;
}

/**
 * @brief 执行一次平衡控制计算
 *
 * @param command 动作层生成的统一平衡命令
 * @param sensor 当前传感器快照
 * @param tick_ms 控制周期，单位毫秒
 *
 * @return 平衡状态和电机命令
 */
control::status control::balance::step(const control::balance_command &command,
    const control::sensor_snapshot &sensor, uint32_t tick_ms)
{
    balance_runtime_state.command = command;
    float dt = (float)tick_ms * 1.0e-3f;
    update_state(sensor);
    update_gain(sensor.avg_leg_height);

    if(command.reset_reference){reset_reference();}
    if(command.reset_yaw_integral){lqi_core.integral.yaw_rate_error = 0.0f;}

    bool balance_enabled = command.mode == control::balance_mode::BALANCE ||
        command.mode == control::balance_mode::RECOVER;
    if(balance_enabled)
    {
        update_linear_reference(dt);
        update_yaw_reference(dt);
    }

    control::status &status = balance_runtime_state.last_status;
    status.timestamp_us = sensor.timestamp_us;
    status.pitch_angle = lqi_core.state.pitch_angle;
    status.pitch_rate = lqi_core.state.pitch_rate;
    status.avg_linear_pos = lqi_core.state.avg_linear_pos;
    status.avg_linear_vel = lqi_core.state.avg_linear_vel;
    status.yaw_angle = lqi_core.state.yaw_angle;
    status.yaw_rate = lqi_core.state.yaw_rate;
    status.reference_linear_vel = lqi_core.reference.linear_vel;
    status.reference_yaw_rate = lqi_core.reference.yaw_rate;
    status.input[0] = command.linear_vel;
    status.input[1] = command.yaw_rate;
    status.roll_angle = sensor.roll_angle;
    status.leg_height[0] = sensor.leg_height[0];
    status.leg_height[1] = sensor.leg_height[1];
    status.avg_leg_height = sensor.avg_leg_height;
    solve_output(command, status);
    return status;
}

/**
 * @brief 初始化平衡算法运行状态
 */
void control::balance::init()
{
    lqi_core = lqi_runtime{};
    balance_runtime_state = balance_runtime{};
    init_lqi();
}
