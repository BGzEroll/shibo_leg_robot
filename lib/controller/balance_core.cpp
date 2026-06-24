#include "balance_core.h"

#include "controller.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "lqi.h"
#include "motor.h"
#include "mpu6050_dev.h"
#include "sts3032.h"

namespace balance_core
{
	void init();
}

namespace lqi
{
	void init();
}

struct sensor_snapshot
{
    bool imu_valid = false;
    bool encoder_valid = false;
    uint32_t timestamp_us = 0;
    lqi::feedback_state feedback{};
    float roll_angle = 0.0f;
    float leg_height[2]{};
    float avg_leg_height = 0.0f;
    int16_t servo_position[2]{};
};

static QueueHandle_t status_queue = nullptr;

struct core_runtime
{
    balance_core::target_t target;
    balance_core::command_t command;
    balance_core::status_snapshot status;
    LowPassFilter vel_filter{0.008f};
    float last_height = 0.0f;
    float lpf_linear_target = 0.0f;
    float lpf_yaw_target = 0.0f;
    float last_linear_target = 0.0f;
    float linear_release_timer = 0.0f;
    bool linear_release = false;
    bool first_state = true;
    uint32_t servo_timer_ms = 0;
};

static core_runtime core;

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
    core.lpf_yaw_target = 0.0f;
    lqi::ref.linear_vel = 0.0f;
    lqi::ref.yaw_rate = 0.0f;
    lqi::integral.linear_vel_error = 0.0f;
    lqi::integral.yaw_rate_error = 0.0f;
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
        lqi::feedback_gain[0][i] =
            lqi::gain_poly[i][0] * h3 +
            lqi::gain_poly[i][1] * h2 +
            lqi::gain_poly[i][2] * height +
            lqi::gain_poly[i][3];

        lqi::feedback_gain[1][i] =
            lqi::gain_poly[i + 6][0] * h3 +
            lqi::gain_poly[i + 6][1] * h2 +
            lqi::gain_poly[i + 6][2] * height +
            lqi::gain_poly[i + 6][3];
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
    const float max_decel = 3.10f;
    const float max_release_decel = 7.20f;
    const float release_duration = 0.45f;
    const float release_stop_speed = 0.035f;
    const float dead_zone = lqi::limit.max_linear_vel * 0.05f;

    float target = core.target.linear_vel;
    bool zero_cmd = fabsf(target) < dead_zone;
    bool had_cmd = fabsf(core.last_linear_target) >= dead_zone;
    core.lpf_linear_target += (target - core.lpf_linear_target) * (1.0f - expf(-dt / tau));

    if(!zero_cmd)
    {
        core.linear_release = false;
        core.linear_release_timer = 0.0f;
    }
    else if(had_cmd)
    {
        core.linear_release = true;
        core.linear_release_timer = 0.0f;
        core.lpf_linear_target = 0.0f;
    }

    float target_ref = fabsf(core.lpf_linear_target) < dead_zone ? 0.0f : core.lpf_linear_target;
    if(core.linear_release)
    {
        target_ref = 0.0f;
        core.linear_release_timer += dt;
        if(fabsf(lqi::state.avg_linear_vel) < release_stop_speed ||
           core.linear_release_timer >= release_duration)
        {
            core.linear_release = false;
            core.linear_release_timer = 0.0f;
        }
    }

    float delta = target_ref - lqi::ref.linear_vel;
    float rate = fabsf(target_ref) > fabsf(lqi::ref.linear_vel) ? max_accel : max_decel;
    if(core.linear_release){rate = max_release_decel;}

    float max_step = rate * dt;
    lqi::ref.linear_vel += constrain(delta, -max_step, max_step);
    if(fabsf(target_ref) < dead_zone && fabsf(lqi::ref.linear_vel) < max_step)
    {
        lqi::ref.linear_vel = 0.0f;
    }

    if(!core.linear_release)
    {
        integrate(lqi::integral.linear_vel_error,
                  lqi::ref.linear_vel - lqi::state.avg_linear_vel,
                  dt,
                  lqi::integral_clamp.linear_vel_error);
    }

    core.last_linear_target = zero_cmd ? 0.0f : target;
}

/**
 * @brief 更新转向角速度参考和转向积分
 *
 * @param dt 时间步长，单位秒
 */
static void update_yaw_reference(float dt)
{
    if(!core.command.enable_steering)
    {
        core.lpf_yaw_target = 0.0f;
        lqi::ref.yaw_rate = 0.0f;
        lqi::integral.yaw_rate_error = 0.0f;
        return;
    }

    const float tau = 0.009f;
    core.lpf_yaw_target += (core.target.yaw_rate - core.lpf_yaw_target) * (1.0f - expf(-dt / tau));
    lqi::ref.yaw_rate = core.lpf_yaw_target;

    if(core.command.suppress_yaw_integral)
    {
        lqi::integral.yaw_rate_error = 0.0f;
        return;
    }

    integrate(lqi::integral.yaw_rate_error,
              lqi::ref.yaw_rate - lqi::state.yaw_rate,
              dt,
              lqi::integral_clamp.yaw_rate_error);
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
    if((core.servo_timer_ms += tick_ms) >= 20)
    {
        core.servo_timer_ms = 0;
        sts3032::get_position_and_load();
    }

    sensor_snapshot sensor;
    sensor.timestamp_us = (uint32_t)esp_timer_get_time();
    sensor.servo_position[0] = sts3032::status[0].position;
    sensor.servo_position[1] = sts3032::status[1].position;
    sensor.leg_height[0] = servo_count_to_height(sensor.servo_position[0]);
    sensor.leg_height[1] = servo_count_to_height(sensor.servo_position[1]);
    sensor.avg_leg_height = (sensor.leg_height[0] + sensor.leg_height[1]) * 0.5f;

    mpu6050_dev::data imu_data;
    if(mpu6050_dev::queue() && xQueuePeek(mpu6050_dev::queue(), &imu_data, 0) == pdTRUE)
    {
        sensor.imu_valid = true;
        sensor.feedback.pitch_angle = imu_data.angle[1];
        sensor.feedback.pitch_rate = imu_data.gyro[1];
        sensor.feedback.yaw_angle = imu_data.angle[2];
        sensor.feedback.yaw_rate = imu_data.gyro[2];
        sensor.roll_angle = imu_data.angle[0];
    }

    motor::encoder_data encoder;
    if(motor::encoder_queue() && xQueuePeek(motor::encoder_queue(), &encoder, 0) == pdTRUE)
    {
        sensor.encoder_valid = true;
        sensor.feedback.avg_linear_pos =
            -(encoder.left_shaft_angle + encoder.right_shaft_angle) * lqi::car.r * 0.5f;
        sensor.feedback.avg_linear_vel =
            -(encoder.left_shaft_velocity + encoder.right_shaft_velocity) * lqi::car.r * 0.5f;
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
        lqi::state.pitch_angle = sensor.feedback.pitch_angle;
        lqi::state.pitch_rate = sensor.feedback.pitch_rate;
        lqi::state.yaw_angle = sensor.feedback.yaw_angle;
        lqi::state.yaw_rate = sensor.feedback.yaw_rate;
    }
    if(sensor.encoder_valid)
    {
        lqi::state.avg_linear_pos = sensor.feedback.avg_linear_pos;
        lqi::state.avg_linear_vel = core.vel_filter(sensor.feedback.avg_linear_vel);
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
    if(motor::target_queue())
    {
        xQueueOverwrite(motor::target_queue(), &target);
    }
}

/**
 * @brief 根据当前状态和命令计算电机输出
 */
static void solve_output()
{
    if(!core.command.enable_motor)
    {
        core.status.output[0] = 0.0f;
        core.status.output[1] = 0.0f;
        publish_motor_target(0.0f, 0.0f);
        return;
    }
    if(core.command.direct_output)
    {
        core.status.output[0] = core.target.direct_left;
        core.status.output[1] = core.target.direct_right;
        publish_motor_target(core.target.direct_left, core.target.direct_right);
        return;
    }
    if(!core.command.enable_balance)
    {
        core.status.output[0] = 0.0f;
        core.status.output[1] = 0.0f;
        publish_motor_target(0.0f, 0.0f);
        return;
    }

    float x[6] = {
        lqi::state.pitch_angle,
        lqi::state.pitch_rate,
        lqi::state.avg_linear_vel - lqi::ref.linear_vel,
        lqi::state.yaw_rate - lqi::ref.yaw_rate,
        lqi::integral.linear_vel_error,
        lqi::integral.yaw_rate_error
    };

    if(core.command.recover_active)
    {
        x[2] = x[3] = x[4] = x[5] = 0.0f;
    }
    if(core.command.suppress_linear_feedback){x[2] = 0.0f;}
    if(core.command.suppress_yaw_feedback || !core.command.enable_steering)
    {
        x[3] = 0.0f;
        x[5] = 0.0f;
    }
    else if(core.command.suppress_yaw_integral)
    {
        x[5] = 0.0f;
    }

    memcpy(core.status.feedback_vector, x, sizeof(core.status.feedback_vector));
    for(uint8_t i = 0; i < 2; i++)
    {
        core.status.output[i] = 0.0f;
        for(uint8_t j = 0; j < 6; j++)
        {
            core.status.output[i] += lqi::feedback_gain[i][j] * x[j];
        }
        core.status.output[i] *= core.command.output_blend;
    }

    publish_motor_target(core.status.output[0], core.status.output[1]);
}

/**
 * @brief 执行一次平衡控制周期
 *
 * @param tick_ms 本次更新周期，单位毫秒
 */
static void control_step(uint32_t tick_ms)
{
    float dt = (float)tick_ms * 1.0e-3f;
    sensor_snapshot sensor = read_sensor(tick_ms);

    update_state(sensor);
    update_gain(sensor.avg_leg_height);

    if(core.command.reset_reference){reset_reference();}
    if(core.command.reset_yaw_integral){lqi::integral.yaw_rate_error = 0.0f;}

    if(core.command.enable_balance)
    {
        update_linear_reference(dt);
        update_yaw_reference(dt);
    }
    else
    {
        reset_reference();
    }

    core.status.timestamp_us = sensor.timestamp_us;
    core.status.pitch_angle = lqi::state.pitch_angle;
    core.status.pitch_rate = lqi::state.pitch_rate;
    core.status.avg_linear_pos = lqi::state.avg_linear_pos;
    core.status.avg_linear_vel = lqi::state.avg_linear_vel;
    core.status.yaw_angle = lqi::state.yaw_angle;
    core.status.yaw_rate = lqi::state.yaw_rate;
    core.status.reference_linear_vel = lqi::ref.linear_vel;
    core.status.reference_yaw_rate = lqi::ref.yaw_rate;
    core.status.input[0] = core.target.linear_vel;
    core.status.input[1] = core.target.yaw_rate;
    core.status.roll_angle = sensor.roll_angle;
    core.status.leg_height[0] = sensor.leg_height[0];
    core.status.leg_height[1] = sensor.leg_height[1];
    core.status.avg_leg_height = sensor.avg_leg_height;

    solve_output();

    if(status_queue)
    {
        xQueueOverwrite(status_queue, &core.status);
    }
}

/**
 * @brief 初始化平衡核心及其底层设备
 */
void balance_core::init()
{
    lqi::init();

    status_queue = xQueueCreate(1, sizeof(balance_core::status_snapshot));

    sts3032::init();
    mpu6050_dev::init();
    motor::init();
}

/**
 * @brief 设置平衡核心目标量
 *
 * @param target 目标值
 */
void balance_core::set_target(const balance_core::target_t &target)
{
    core.target = target;
}

/**
 * @brief 设置平衡核心控制命令
 *
 * @param command 控制命令
 */
void balance_core::set_command(const balance_core::command_t &command)
{
    core.command = command;
}

/**
 * @brief 读取平衡核心最新状态快照
 *
 * @param out 输出状态快照
 *
 * @return 条件是否成立
 */
bool balance_core::get_status(balance_core::status_snapshot &out)
{
    return status_queue && xQueuePeek(status_queue, &out, 0) == pdTRUE;
}

/**
 * @brief 获取平衡核心对上层公开的信息快照
 *
 * @return 平衡核心信息快照
 */
balance_core::info_t balance_core::get_info()
{
    balance_core::info_t info;
    info.max_linear_vel = lqi::limit.max_linear_vel;
    info.max_steer_vel = lqi::limit.max_steer_vel;
    info.wheel_radius = lqi::car.r;
    return info;
}

/**
 * @brief 平衡核心高频 IO 任务入口
 *
 * @param arg RTOS 任务参数
 */
void balance_core::core_task_entry(void *arg)
{
    (void)arg;
    uint32_t last_encoder_us = (uint32_t)esp_timer_get_time();
    uint32_t last_imu_us = last_encoder_us;

    while(true)
    {
        motor::left.loopFOC();
        motor::right.loopFOC();

        motor::target_data target;
        if(motor::target_queue() && xQueuePeek(motor::target_queue(), &target, 0) == pdTRUE)
        {
            motor::left.move(target.left_torque);
            motor::right.move(target.right_torque);
        }

        uint32_t now_us = (uint32_t)esp_timer_get_time();
        if((uint32_t)(now_us - last_encoder_us) >= 1000)
        {
            last_encoder_us = now_us;
            motor::encoder_data encoder;
            encoder.timestamp_us = now_us;
            encoder.left_shaft_angle = motor::left.shaft_angle;
            encoder.left_shaft_velocity = motor::left.shaft_velocity;
            encoder.right_shaft_angle = motor::right.shaft_angle;
            encoder.right_shaft_velocity = motor::right.shaft_velocity;
            if(motor::encoder_queue())
            {
                xQueueOverwrite(motor::encoder_queue(), &encoder);
            }
        }

        if((uint32_t)(now_us - last_imu_us) >= 5000)
        {
            last_imu_us = now_us;
            mpu6050_dev::imu.update();

            mpu6050_dev::data imu;
            imu.timestamp_us = now_us;
            imu.temperature = mpu6050_dev::imu.temperature;
            for(uint8_t i = 0; i < 3; i++)
            {
                imu.acc[i] = mpu6050_dev::imu.acc[i];
                imu.gyro[i] = mpu6050_dev::imu.gyro[i];
                imu.angle[i] = mpu6050_dev::imu.angle[i];
            }
            if(mpu6050_dev::queue())
            {
                xQueueOverwrite(mpu6050_dev::queue(), &imu);
            }
        }

        taskYIELD();
    }
}

/**
 * @brief 平衡核心控制任务入口
 *
 * @param arg RTOS 任务参数
 */
void balance_core::control_task_entry(void *arg)
{
    (void)arg;
    TickType_t last_wake_time = xTaskGetTickCount();

    while(true)
    {
        controller::update(1);
        control_step(1);
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(1));
    }
}
