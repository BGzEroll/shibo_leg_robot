#include "balance_core.h"

#include "esp_timer.h"

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
    lqi_reference_state ref;
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
    memset(&lqi_core.ref, 0, sizeof(lqi_core.ref));
    memset(&lqi_core.integral, 0, sizeof(lqi_core.integral));
}

/* ---- 平衡核心运行状态 ---- */

struct sensor_snapshot
{
    bool imu_valid = false;
    bool encoder_valid = false;
    uint32_t timestamp_us = 0;
    lqi_feedback_state feedback{};
    float roll_angle = 0.0f;
    float leg_height[2]{};
    float avg_leg_height = 0.0f;
    int16_t servo_position[2]{};
};

struct core_status_snapshot
{
    uint32_t timestamp_us = 0;
    float pitch_angle = 0.0f;
    float pitch_rate = 0.0f;
    float avg_linear_pos = 0.0f;
    float avg_linear_vel = 0.0f;
    float yaw_angle = 0.0f;
    float yaw_rate = 0.0f;
    float reference_linear_vel = 0.0f;
    float reference_yaw_rate = 0.0f;
    float input[2]{};
    float feedback_vector[6]{};
    float output[2]{};
    float roll_angle = 0.0f;
    float leg_height[2]{};
    float avg_leg_height = 0.0f;
};

static balance_core::input_ports module_inputs;
static balance_core::output_ports module_outputs;

struct core_runtime
{
    balance_core::motion_control motion;
    balance_core::direct_output_control direct_output;
    balance_core::recover_control recover;
    balance_core::feedback_override feedback;
    core_status_snapshot status;
    LowPassFilter vel_filter{0.008f};
    float last_height = 0.0f;
    float lpf_linear_target = 0.0f;
    float lpf_yaw_target = 0.0f;
    float last_linear_target = 0.0f;
    float linear_release_timer = 0.0f;
    bool linear_release = false;
    bool linear_release_hold = false;
    bool first_state = true;
    uint32_t servo_timer_ms = 0;
};

static core_runtime core;

/* ---- 平衡控制内部流程 ---- */

/**
 * @brief 将舵机位置计数转换为腿长估计值
 *
 * @param position 舵机位置计数值
 *
 * @return 计算结果
 */
static float servo_count_to_height(int16_t position)
{
    float d = fabsf((float)position - 2048.0f);
    return ((4.6289047954e-12f * d - 9.3936274976e-08f) * d +
            1.5357902969e-04f) * d + 4.2041568108e-02f;
}

/**
 * @brief 清空线速度和转向参考以及积分状态
 */
static void reset_reference()
{
    core.lpf_linear_target = 0.0f;
    core.last_linear_target = 0.0f;
    core.linear_release_timer = 0.0f;
    core.linear_release = false;
    core.linear_release_hold = false;
    core.lpf_yaw_target = 0.0f;
    lqi_core.ref.linear_vel = 0.0f;
    lqi_core.ref.yaw_rate = 0.0f;
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
    if(fabsf(height - core.last_height) < 1.0e-4f){return;}
    core.last_height = height;

    float h2 = height * height;
    float h3 = h2 * height;
    for(uint8_t i = 0; i < 6; i++)
    {
        lqi_core.feedback_gain[0][i] =
            LQI_GAIN_POLY[i][0] * h3 +
            LQI_GAIN_POLY[i][1] * h2 +
            LQI_GAIN_POLY[i][2] * height +
            LQI_GAIN_POLY[i][3];

        lqi_core.feedback_gain[1][i] =
            LQI_GAIN_POLY[i + 6][0] * h3 +
            LQI_GAIN_POLY[i + 6][1] * h2 +
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

    float target = core.motion.linear_vel;
    bool zero_cmd = target == 0.0f;
    bool had_cmd = core.last_linear_target != 0.0f;
    core.lpf_linear_target += (target - core.lpf_linear_target) * (1.0f - expf(-dt / tau));

    if(!zero_cmd)
    {
        core.linear_release = false;
        core.linear_release_timer = 0.0f;
        core.linear_release_hold = false;
    }
    else if(had_cmd)
    {
        core.linear_release = true;
        core.linear_release_timer = 0.0f;
        core.linear_release_hold = true;
        core.lpf_linear_target = 0.0f;
    }

    float target_ref = core.lpf_linear_target;
    bool release_done = false;
    if(core.linear_release)
    {
        target_ref = 0.0f;
        core.linear_release_timer += dt;
        if(core.linear_release_hold &&
           fabsf(lqi_core.state.avg_linear_vel) <= release_hold_speed)
        {
            core.linear_release_hold = false;
        }
        release_done =
            fabsf(lqi_core.state.avg_linear_vel) <= release_resume_speed ||
            core.linear_release_timer >= release_duration;
    }

    if(fabsf(target_ref) > fabsf(lqi_core.ref.linear_vel))
    {
        float delta = target_ref - lqi_core.ref.linear_vel;
        float max_step = max_accel * dt;
        lqi_core.ref.linear_vel += constrain(delta, -max_step, max_step);
    }
    else
    {
        lqi_core.ref.linear_vel = target_ref;
    }

    float linear_error = lqi_core.ref.linear_vel - lqi_core.state.avg_linear_vel;
    if(core.linear_release)
    {
        // 高速阶段冻结积分，进入低速阶段后只允许积分向零释放。
        if(!core.linear_release_hold)
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
        integrate(lqi_core.integral.linear_vel_error,
                  linear_error,
                  dt,
                  lqi_core.integral_clamp.linear_vel_error);
    }

    if(release_done)
    {
        core.linear_release = false;
        core.linear_release_timer = 0.0f;
        core.linear_release_hold = false;
    }

    core.last_linear_target = target;
}

/**
 * @brief 更新转向角速度参考和转向积分
 *
 * @param dt 时间步长，单位秒
 */
static void update_yaw_reference(float dt)
{
    if(!core.motion.enable_steering)
    {
        core.lpf_yaw_target = 0.0f;
        lqi_core.ref.yaw_rate = 0.0f;
        lqi_core.integral.yaw_rate_error = 0.0f;
        return;
    }

    const float tau = 0.009f;
    core.lpf_yaw_target += (core.motion.yaw_rate - core.lpf_yaw_target) * (1.0f - expf(-dt / tau));
    lqi_core.ref.yaw_rate = core.lpf_yaw_target;

    if(!core.feedback.enable_yaw_integral)
    {
        lqi_core.integral.yaw_rate_error = 0.0f;
        return;
    }

    integrate(lqi_core.integral.yaw_rate_error,
              lqi_core.ref.yaw_rate - lqi_core.state.yaw_rate,
              dt,
              lqi_core.integral_clamp.yaw_rate_error);
}

/**
 * @brief 读取传感器和编码器快照
 *
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 传感器快照
 */
static sensor_snapshot read_sensor(uint32_t tick_ms)
{
    sensor_snapshot sensor;
    sensor.timestamp_us = (uint32_t)esp_timer_get_time();
    actuator_port::leg_status leg_status;
    if(module_inputs.leg_status.read(leg_status))
    {
        sensor.servo_position[0] = leg_status.left_position;
        sensor.servo_position[1] = leg_status.right_position;
    }
    sensor.leg_height[0] = servo_count_to_height(sensor.servo_position[0]);
    sensor.leg_height[1] = servo_count_to_height(sensor.servo_position[1]);
    sensor.avg_leg_height = (sensor.leg_height[0] + sensor.leg_height[1]) * 0.5f;

    mpu6050_dev::data imu_data;
    if(module_inputs.imu.read(imu_data))
    {
        sensor.imu_valid = true;
        sensor.feedback.pitch_angle = imu_data.angle[1];
        sensor.feedback.pitch_rate = imu_data.gyro[1];
        sensor.feedback.yaw_angle = imu_data.angle[2];
        sensor.feedback.yaw_rate = imu_data.gyro[2];
        sensor.roll_angle = imu_data.angle[0];
    }

    motor::encoder_data encoder;
    if(module_inputs.encoder.read(encoder))
    {
        sensor.encoder_valid = true;
        sensor.feedback.avg_linear_pos =
            -(encoder.left_shaft_angle + encoder.right_shaft_angle) * lqi_core.car.r * 0.5f;
        sensor.feedback.avg_linear_vel =
            -(encoder.left_shaft_velocity + encoder.right_shaft_velocity) * lqi_core.car.r * 0.5f;
    }

    return sensor;
}

/**
 * @brief 将传感器快照写入 LQI 状态
 *
 * @param sensor 传感器快照
 */
static void update_state(const sensor_snapshot &sensor)
{
    if(sensor.imu_valid)
    {
        lqi_core.state.pitch_angle = sensor.feedback.pitch_angle;
        lqi_core.state.pitch_rate = sensor.feedback.pitch_rate;
        lqi_core.state.yaw_angle = sensor.feedback.yaw_angle;
        lqi_core.state.yaw_rate = sensor.feedback.yaw_rate;
    }
    if(sensor.encoder_valid)
    {
        lqi_core.state.avg_linear_pos = sensor.feedback.avg_linear_pos;
        lqi_core.state.avg_linear_vel = core.vel_filter(sensor.feedback.avg_linear_vel);
    }
    if(core.first_state)
    {
        core.first_state = false;
        reset_reference();
    }
}

/**
 * @brief 发布左右电机目标输出
 *
 * @param left 左侧目标值
 * @param right 右侧目标值
 */
static void publish_motor_target(float left, float right)
{
    motor::target_data target;
    target.timestamp_us = (uint32_t)esp_timer_get_time();
    target.left_torque = left;
    target.right_torque = right;
    target.enabled = core.motion.enable_motor;
    module_outputs.motor_target.publish(target);
}

/**
 * @brief 根据当前状态和命令计算电机输出
 */
static void solve_output()
{
    if(!core.motion.enable_motor)
    {
        core.status.output[0] = 0.0f;
        core.status.output[1] = 0.0f;
        publish_motor_target(0.0f, 0.0f);
        return;
    }
    if(core.direct_output.enable)
    {
        core.status.output[0] = core.direct_output.left;
        core.status.output[1] = core.direct_output.right;
        publish_motor_target(core.direct_output.left, core.direct_output.right);
        return;
    }
    if(!core.motion.enable_balance)
    {
        core.status.output[0] = 0.0f;
        core.status.output[1] = 0.0f;
        publish_motor_target(0.0f, 0.0f);
        return;
    }

    float x[6] =
    {
        lqi_core.state.pitch_angle,
        lqi_core.state.pitch_rate,
        lqi_core.state.avg_linear_vel - lqi_core.ref.linear_vel,
        lqi_core.state.yaw_rate - lqi_core.ref.yaw_rate,
        lqi_core.integral.linear_vel_error,
        lqi_core.integral.yaw_rate_error
    };

    if(core.recover.enable)
    {
        x[2] = x[3] = x[4] = x[5] = 0.0f;
    }
    if(!core.feedback.enable_linear_feedback){x[2] = 0.0f;}
    if(!core.feedback.enable_yaw_feedback || !core.motion.enable_steering)
    {
        x[3] = 0.0f;
        x[5] = 0.0f;
    }
    else if(!core.feedback.enable_yaw_integral)
    {
        x[5] = 0.0f;
    }

    memcpy(core.status.feedback_vector, x, sizeof(core.status.feedback_vector));
    for(uint8_t i = 0; i < 2; i++)
    {
        core.status.output[i] = 0.0f;
        for(uint8_t j = 0; j < 6; j++)
        {
            core.status.output[i] += lqi_core.feedback_gain[i][j] * x[j];
        }
        core.status.output[i] *= core.recover.enable ? core.recover.output_blend : 1.0f;
    }

    publish_motor_target(core.status.output[0], core.status.output[1]);
}

/**
 * @brief 执行一次平衡控制周期
 *
 * @param tick_ms 本次更新周期，单位毫秒
 */
static void run_control_step(uint32_t tick_ms)
{
    balance_core::command command;
    if(module_inputs.control.read(command))
    {
        core.motion = command.motion;
        core.direct_output = command.direct_output;
        core.recover = command.recover;
        core.feedback = command.feedback;
    }

    float dt = (float)tick_ms * 1.0e-3f;
    sensor_snapshot sensor = read_sensor(tick_ms);

    update_state(sensor);
    update_gain(sensor.avg_leg_height);

    if(core.motion.reset_reference){reset_reference();}
    if(core.motion.reset_yaw_integral){lqi_core.integral.yaw_rate_error = 0.0f;}

    if(core.motion.enable_balance)
    {
        update_linear_reference(dt);
        update_yaw_reference(dt);
    }

    core.status.timestamp_us = sensor.timestamp_us;
    core.status.pitch_angle = lqi_core.state.pitch_angle;
    core.status.pitch_rate = lqi_core.state.pitch_rate;
    core.status.avg_linear_pos = lqi_core.state.avg_linear_pos;
    core.status.avg_linear_vel = lqi_core.state.avg_linear_vel;
    core.status.yaw_angle = lqi_core.state.yaw_angle;
    core.status.yaw_rate = lqi_core.state.yaw_rate;
    core.status.reference_linear_vel = lqi_core.ref.linear_vel;
    core.status.reference_yaw_rate = lqi_core.ref.yaw_rate;
    core.status.input[0] = core.motion.linear_vel;
    core.status.input[1] = core.motion.yaw_rate;
    core.status.roll_angle = sensor.roll_angle;
    core.status.leg_height[0] = sensor.leg_height[0];
    core.status.leg_height[1] = sensor.leg_height[1];
    core.status.avg_leg_height = sensor.avg_leg_height;

    solve_output();

    balance_core::motion_status motion;
    motion.timestamp_us = core.status.timestamp_us;
    motion.pitch_angle = core.status.pitch_angle;
    motion.pitch_rate = core.status.pitch_rate;
    motion.avg_linear_vel = core.status.avg_linear_vel;
    motion.yaw_angle = core.status.yaw_angle;
    motion.yaw_rate = core.status.yaw_rate;
    motion.roll_angle = core.status.roll_angle;
    motion.avg_leg_height = core.status.avg_leg_height;
    module_outputs.motion.publish(motion);

    balance_core::debug_snapshot debug;
    debug.timestamp_us = core.status.timestamp_us;
    debug.pitch_angle = core.status.pitch_angle;
    debug.pitch_rate = core.status.pitch_rate;
    debug.avg_linear_pos = core.status.avg_linear_pos;
    debug.avg_linear_vel = core.status.avg_linear_vel;
    debug.yaw_angle = core.status.yaw_angle;
    debug.yaw_rate = core.status.yaw_rate;
    debug.reference_linear_vel = core.status.reference_linear_vel;
    debug.reference_yaw_rate = core.status.reference_yaw_rate;
    memcpy(debug.input, core.status.input, sizeof(debug.input));
    memcpy(debug.feedback_vector, core.status.feedback_vector, sizeof(debug.feedback_vector));
    memcpy(debug.output, core.status.output, sizeof(debug.output));
    debug.roll_angle = core.status.roll_angle;
    memcpy(debug.leg_height, core.status.leg_height, sizeof(debug.leg_height));
    module_outputs.debug.publish(debug);
}

/* ---- balance_core 公共 API ---- */

/**
 * @brief 获取平衡核心对外公开的信息快照
 *
 * @return 平衡核心信息快照
 */
balance_core::info balance_core::get_info()
{
    balance_core::info info;
    info.max_linear_vel = lqi_core.limit.max_linear_vel;
    info.max_steer_vel = lqi_core.limit.max_steer_vel;
    return info;
}

/**
 * @brief 执行一次平衡控制周期
 *
 * @param tick_ms 本次更新周期，单位毫秒
 */
void balance_core::step(uint32_t tick_ms)
{
    run_control_step(tick_ms);
}

/**
 * @brief 初始化平衡控制模块及端口
 *
 * @param inputs 平衡控制输入端口
 * @param outputs 平衡控制输出端口
 */
void balance_core::init(const balance_core::input_ports &inputs, const balance_core::output_ports &outputs)
{
    module_inputs = inputs;
    module_outputs = outputs;
    init_lqi();
}
