#include "balance_core.h"

#include "controller.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "motor.h"
#include "mpu6050_dev.h"
#include "sts3032.h"

static QueueHandle_t command_queue = nullptr;
static QueueHandle_t status_queue = nullptr;

struct core_runtime {
    controller::lqi plant;
    balance_core::balance_command command;
    balance_core::balance_status status;
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

float balance_core::servo_count_to_height(int16_t position)
{
    float d = fabsf((float)position - 2048.0f);
    return ((4.6289047954e-12f * d - 9.3936274976e-08f) * d +
            1.5357902969e-04f) * d + 4.2041568108e-02f;
}

static void reset_linear_reference()
{
    core.lpf_linear_target = 0.0f;
    core.last_linear_target = 0.0f;
    core.linear_release_timer = 0.0f;
    core.linear_release = false;
}

static void reset_reference()
{
    reset_linear_reference();
    core.lpf_yaw_target = 0.0f;
    core.plant.ref.linear_vel = 0.0f;
    core.plant.ref.yaw_rate = 0.0f;
    core.plant.integral.linear_vel_error = 0.0f;
    core.plant.integral.yaw_rate_error = 0.0f;
}

static void integrate(float &value, float error, float dt, float limit)
{
    value += error * dt;
    value = constrain(value, -limit, limit);
}

static void update_gain(float height)
{
    if(fabsf(height - core.last_height) < 1.0e-4f){return;}
    core.last_height = height;

    float h2 = height * height;
    float h3 = h2 * height;
    for(uint8_t i = 0; i < 6; i++)
    {
        core.plant.feedback_gain[0][i] =
            core.plant.gain_poly[i][0] * h3 +
            core.plant.gain_poly[i][1] * h2 +
            core.plant.gain_poly[i][2] * height +
            core.plant.gain_poly[i][3];

        core.plant.feedback_gain[1][i] =
            core.plant.gain_poly[i + 6][0] * h3 +
            core.plant.gain_poly[i + 6][1] * h2 +
            core.plant.gain_poly[i + 6][2] * height +
            core.plant.gain_poly[i + 6][3];
    }
}

static void update_linear_reference(float dt)
{
    const float tau = 0.024f;
    const float max_accel = 1.60f;
    const float max_decel = 3.10f;
    const float max_release_decel = 7.20f;
    const float release_duration = 0.45f;
    const float release_stop_speed = 0.035f;
    const float dead_zone = core.plant.limit.max_linear_vel * 0.05f;

    float target = core.command.target_linear_vel;
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
        if(fabsf(core.plant.state.avg_linear_vel) < release_stop_speed ||
           core.linear_release_timer >= release_duration)
        {
            core.linear_release = false;
            core.linear_release_timer = 0.0f;
        }
    }

    float delta = target_ref - core.plant.ref.linear_vel;
    float rate = fabsf(target_ref) > fabsf(core.plant.ref.linear_vel) ? max_accel : max_decel;
    if(core.linear_release){rate = max_release_decel;}

    float max_step = rate * dt;
    core.plant.ref.linear_vel += constrain(delta, -max_step, max_step);
    if(fabsf(target_ref) < dead_zone && fabsf(core.plant.ref.linear_vel) < max_step)
    {
        core.plant.ref.linear_vel = 0.0f;
    }

    if(!core.linear_release)
    {
        integrate(core.plant.integral.linear_vel_error,
                  core.plant.ref.linear_vel - core.plant.state.avg_linear_vel,
                  dt,
                  core.plant.integral_clamp.linear_vel_error);
    }

    core.last_linear_target = zero_cmd ? 0.0f : target;
}

static void update_yaw_reference(float dt)
{
    if(!core.command.enable_steering)
    {
        core.lpf_yaw_target = 0.0f;
        core.plant.ref.yaw_rate = 0.0f;
        core.plant.integral.yaw_rate_error = 0.0f;
        return;
    }

    const float tau = 0.009f;
    core.lpf_yaw_target += (core.command.target_yaw_rate - core.lpf_yaw_target) * (1.0f - expf(-dt / tau));
    core.plant.ref.yaw_rate = core.lpf_yaw_target;

    if(core.command.suppress_yaw_integral)
    {
        core.plant.integral.yaw_rate_error = 0.0f;
        return;
    }

    integrate(core.plant.integral.yaw_rate_error,
              core.plant.ref.yaw_rate - core.plant.state.yaw_rate,
              dt,
              core.plant.integral_clamp.yaw_rate_error);
}

static balance_core::sensor_snapshot read_sensor(uint32_t tick_ms)
{
    if((core.servo_timer_ms += tick_ms) >= 20)
    {
        core.servo_timer_ms = 0;
        sts3032::get_position_and_load();
    }

    balance_core::sensor_snapshot sensor;
    sensor.timestamp_us = (uint32_t)esp_timer_get_time();
    sensor.servo_position[0] = sts3032::status[0].position;
    sensor.servo_position[1] = sts3032::status[1].position;
    sensor.leg_height[0] = balance_core::servo_count_to_height(sensor.servo_position[0]);
    sensor.leg_height[1] = balance_core::servo_count_to_height(sensor.servo_position[1]);
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
            -(encoder.left_shaft_angle + encoder.right_shaft_angle) * core.plant.car.r * 0.5f;
        sensor.feedback.avg_linear_vel =
            -(encoder.left_shaft_velocity + encoder.right_shaft_velocity) * core.plant.car.r * 0.5f;
    }

    return sensor;
}

static void update_state(const balance_core::sensor_snapshot &sensor)
{
    if(sensor.imu_valid)
    {
        core.plant.state.pitch_angle = sensor.feedback.pitch_angle;
        core.plant.state.pitch_rate = sensor.feedback.pitch_rate;
        core.plant.state.yaw_angle = sensor.feedback.yaw_angle;
        core.plant.state.yaw_rate = sensor.feedback.yaw_rate;
    }
    if(sensor.encoder_valid)
    {
        core.plant.state.avg_linear_pos = sensor.feedback.avg_linear_pos;
        core.plant.state.avg_linear_vel = core.vel_filter(sensor.feedback.avg_linear_vel);
    }
    if(core.first_state)
    {
        core.first_state = false;
        reset_reference();
    }
}

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

static void solve_output()
{
    if(core.command.manual_output)
    {
        core.status.output[0] = core.command.manual_left;
        core.status.output[1] = core.command.manual_right;
        publish_motor_target(core.command.manual_left, core.command.manual_right);
        return;
    }
    if(!core.command.enable_motor)
    {
        core.status.output[0] = 0.0f;
        core.status.output[1] = 0.0f;
        publish_motor_target(0.0f, 0.0f);
        return;
    }
    if(!core.command.enable_balance){return;}

    float x[6] = {
        core.plant.state.pitch_angle,
        core.plant.state.pitch_rate,
        core.plant.state.avg_linear_vel - core.plant.ref.linear_vel,
        core.plant.state.yaw_rate - core.plant.ref.yaw_rate,
        core.plant.integral.linear_vel_error,
        core.plant.integral.yaw_rate_error
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
            core.status.output[i] += core.plant.feedback_gain[i][j] * x[j];
        }
        core.status.output[i] *= core.command.output_blend;
    }

    publish_motor_target(core.status.output[0], core.status.output[1]);
}

static void control_step(uint32_t tick_ms)
{
    if(command_queue)
    {
        xQueuePeek(command_queue, &core.command, 0);
    }

    float dt = (float)tick_ms * 1.0e-3f;
    balance_core::sensor_snapshot sensor = read_sensor(tick_ms);

    update_state(sensor);
    update_gain(sensor.avg_leg_height);

    if(core.command.reset_reference){reset_reference();}
    if(core.command.reset_yaw_integral){core.plant.integral.yaw_rate_error = 0.0f;}

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
    core.status.feedback = core.plant.state;
    core.status.reference = core.plant.ref;
    core.status.input[0] = core.command.target_linear_vel;
    core.status.input[1] = core.command.target_yaw_rate;
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

void balance_core::init()
{
    command_queue = xQueueCreate(1, sizeof(balance_core::balance_command));
    status_queue = xQueueCreate(1, sizeof(balance_core::balance_status));
    xQueueOverwrite(command_queue, &core.command);

    sts3032::init();
    mpu6050_dev::init();
    motor::init();
}

void balance_core::set_command(const balance_core::balance_command &cmd)
{
    core.command = cmd;
    if(command_queue)
    {
        xQueueOverwrite(command_queue, &core.command);
    }
}

bool balance_core::get_status(balance_core::balance_status &out)
{
    return status_queue && xQueuePeek(status_queue, &out, 0) == pdTRUE;
}

void balance_core::set_mode(controller::mode_id mode)
{
    core.status.mode = mode;
}

float balance_core::max_linear_vel()
{
    return core.plant.limit.max_linear_vel;
}

float balance_core::max_steer_vel()
{
    return core.plant.limit.max_steer_vel;
}

float balance_core::wheel_radius()
{
    return core.plant.car.r;
}

void balance_core::io_task(void *arg)
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

void balance_core::control_task(void *arg)
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
