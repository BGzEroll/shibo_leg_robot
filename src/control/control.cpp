#include "control.h"

#include "action.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "hw/battery.h"
#include "hw/gamepad.h"
#include "hw/imu.h"
#include "hw/indicator.h"
#include "hw/motor.h"
#include "hw/servo.h"
#include "hw/wifi.h"
#include "input.h"
#include "io/host.h"
#include "io/web.h"
#include <math.h>

/* ---- control 运行状态 ---- */

static constexpr uint32_t CONTROL_PERIOD_MS = 2;
static constexpr uint32_t SERVO_PERIOD_MS = 20;
static constexpr uint32_t BATTERY_PERIOD_MS = 100;
static constexpr uint32_t SERVICE_PERIOD_MS = 5;
static constexpr uint32_t INDICATOR_PERIOD_MS = 50;
static constexpr uint32_t NETWORK_PERIOD_MS = 50;
static constexpr uint32_t ENCODER_TIMEOUT_US = 5000;
static constexpr uint32_t IMU_TIMEOUT_US = 15000;
static constexpr uint32_t MOTOR_COMMAND_TIMEOUT_US = 20000;
static constexpr float WHEEL_RADIUS = 0.0526f / 2.0f;

static control::action::state action_state;
static control::action::leg_runtime leg_state;
static control::control_input input_state;
static control::status balance_status;
static control::info balance_info;
static portMUX_TYPE request_lock = portMUX_INITIALIZER_UNLOCKED;
static bool middle_calibration_requested = false;
static bool middle_calibration_finished = false;
static float camera_angle = 90.0f;
static float camera_speed = 0.0f;
static int16_t last_camera_angle = -1;

/* ---- control 内部流程 ---- */

/**
 * @brief 将舵机位置计数转换为腿长估计值
 *
 * @param position 舵机位置计数值
 *
 * @return 腿长估计值
 */
static float servo_count_to_height(int16_t position)
{
    float distance = fabsf((float)position - 2048.0f);
    return ((4.6289047954e-12f * distance - 9.3936274976e-08f) * distance +
            1.5357902969e-04f) * distance + 4.2041568108e-02f;
}

/**
 * @brief 消费异步中位校准请求
 *
 * @return 有待处理请求时返回 true
 */
static bool consume_middle_calibration_request()
{
    control::action::mode current = control::action::current_mode(action_state);
    if(current != control::action::mode::BALANCE &&
       current != control::action::mode::SIT){return false;}

    bool requested = false;
    portENTER_CRITICAL(&request_lock);
    requested = middle_calibration_requested;
    middle_calibration_requested = false;
    portEXIT_CRITICAL(&request_lock);
    return requested;
}

/**
 * @brief 更新摄像头舵机控制
 *
 * @param tick_ms 本次更新周期，单位毫秒
 */
static void update_camera(uint32_t tick_ms)
{
    float target_speed = (float)input_state.camera_direction * 120.0f;
    float dt = (float)tick_ms * 1.0e-3f;
    camera_speed += (target_speed - camera_speed) * (1.0f - expf(-dt / 0.05f));
    camera_angle = constrain(camera_angle + camera_speed * dt,
        (float)hw::servo::CAMERA_MIN, (float)hw::servo::CAMERA_MAX);
    if((int16_t)camera_angle == last_camera_angle){return;}

    last_camera_angle = (int16_t)camera_angle;
    hw::servo::camera.set_angle((uint16_t)camera_angle);
}

/**
 * @brief 组装控制周期的传感器快照
 *
 * @param tick_ms 本次控制周期，单位毫秒
 *
 * @return 当前传感器状态
 */
static control::sensor_snapshot read_sensor(uint32_t tick_ms)
{
    static uint32_t servo_timer_ms = 0;
    servo_timer_ms += tick_ms;
    if(servo_timer_ms >= SERVO_PERIOD_MS)
    {
        servo_timer_ms = 0;
        hw::servo::get_position_and_load();
    }

    control::sensor_snapshot sensor;
    sensor.timestamp_us = (uint32_t)esp_timer_get_time();
    sensor.servo_position[0] = hw::servo::leg_status[0].position;
    sensor.servo_position[1] = hw::servo::leg_status[1].position;
    sensor.leg_height[0] = servo_count_to_height(sensor.servo_position[0]);
    sensor.leg_height[1] = servo_count_to_height(sensor.servo_position[1]);
    sensor.avg_leg_height = (sensor.leg_height[0] + sensor.leg_height[1]) * 0.5f;

    hw::imu::state imu_state;
    if(hw::imu::latest(imu_state) &&
       (uint32_t)(sensor.timestamp_us - imu_state.timestamp_us) <= IMU_TIMEOUT_US)
    {
        sensor.imu_valid = true;
        sensor.pitch_angle = imu_state.angle[1];
        sensor.pitch_rate = imu_state.gyro[1];
        sensor.yaw_angle = imu_state.angle[2];
        sensor.yaw_rate = imu_state.gyro[2];
        sensor.roll_angle = imu_state.angle[0];
    }

    hw::motor::encoder_state encoder;
    if(hw::motor::latest_encoder(encoder) &&
       (uint32_t)(sensor.timestamp_us - encoder.timestamp_us) <= ENCODER_TIMEOUT_US)
    {
        sensor.encoder_valid = true;
        sensor.avg_linear_pos =
            -(encoder.left_shaft_angle + encoder.right_shaft_angle) * WHEEL_RADIUS * 0.5f;
        sensor.avg_linear_vel =
            -(encoder.left_shaft_velocity + encoder.right_shaft_velocity) * WHEEL_RADIUS * 0.5f;
    }
    return sensor;
}

/**
 * @brief 读取最新电池状态
 *
 * @return 最新电池状态；没有快照时返回无效状态
 */
static hw::battery::state read_battery()
{
    hw::battery::state state;
    if(!hw::battery::latest(state)){return hw::battery::state{};}
    return state;
}

/**
 * @brief 读取最新视觉测量
 *
 * @param valid 是否存在有效视觉测量
 * @param dx 水平偏差输出
 * @param dy 垂直偏差输出
 * @param sequence 视觉序号输出
 */
static void read_vision(bool &valid, int16_t &dx, int16_t &dy, uint32_t &sequence)
{
    io::host::vision_measurement vision;
    valid = io::host::latest_vision(vision);
    dx = vision.dx;
    dy = vision.dy;
    sequence = vision.sequence;
}

/**
 * @brief 创建并启动应用控制链中的 FOC 任务
 *
 * @param arg RTOS 任务参数
 */
void control::foc_task_entry(void *arg)
{
    control::motor_command command;
    bool command_valid = false;
    bool motors_enabled = false;
    uint32_t last_encoder_sample_us = 0;

    while(true)
    {
        control::motor_command next_command;
        if(hw::motor::latest_command(next_command))
        {
            command = next_command;
            command_valid = true;
        }

        uint32_t encoder_sample_us = 0;
        bool encoder_updated =
            hw::motor::apply_latest_encoder_sample(encoder_sample_us);
        if(encoder_updated){last_encoder_sample_us = encoder_sample_us;}

        uint32_t now_us = (uint32_t)esp_timer_get_time();
        bool encoder_fresh = last_encoder_sample_us != 0 &&
            (uint32_t)(now_us - last_encoder_sample_us) <= ENCODER_TIMEOUT_US;
        bool command_fresh = command_valid &&
            (uint32_t)(now_us - command.timestamp_us) <= MOTOR_COMMAND_TIMEOUT_US;
        bool should_enable = command_valid && command.enabled && command_fresh &&
            encoder_fresh;

        if(should_enable)
        {
            if(!motors_enabled)
            {
                hw::motor::left.enable();
                hw::motor::right.enable();
                motors_enabled = true;
            }
        }
        else if(motors_enabled)
        {
            hw::motor::left.disable();
            hw::motor::right.disable();
            motors_enabled = false;
        }

        if(encoder_updated)
        {
            hw::motor::left.loopFOC();
            hw::motor::right.loopFOC();
            if(should_enable)
            {
                hw::motor::left.move(command.left);
                hw::motor::right.move(command.right);
            }
            else
            {
                hw::motor::left.move();
                hw::motor::right.move();
            }

            hw::motor::encoder_state encoder;
            encoder.timestamp_us = encoder_sample_us;
            encoder.left_shaft_angle = hw::motor::left.shaft_angle;
            encoder.left_shaft_velocity = hw::motor::left.shaft_velocity;
            encoder.right_shaft_angle = hw::motor::right.shaft_angle;
            encoder.right_shaft_velocity = hw::motor::right.shaft_velocity;
            hw::motor::publish_encoder(encoder);
        }
        taskYIELD();
    }
}

/**
 * @brief 执行唯一的固定周期控制编排
 *
 * @param arg RTOS 任务参数
 */
void control::control_task_entry(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    while(true)
    {
        control::sensor_snapshot sensor = read_sensor(CONTROL_PERIOD_MS);
        hw::battery::state battery = read_battery();
        bool vision_valid = false;
        int16_t vision_dx = 0;
        int16_t vision_dy = 0;
        uint32_t vision_sequence = 0;

        read_vision(vision_valid, vision_dx, vision_dy, vision_sequence);
        control::action_request external_action = consume_middle_calibration_request() ?
            control::action_request::MIDDLE_CALIBRATION :
            control::action_request::NONE;
        control::input_router::update(
            external_action,
            control::action::current_mode(action_state),
            balance_info.max_linear_vel,
            balance_info.max_steer_vel,
            input_state);
        update_camera(CONTROL_PERIOD_MS);

        control::action::context context{
            input_state,
            balance_status,
            leg_state,
            balance_info.max_linear_vel,
            balance_info.max_steer_vel,
            battery.valid,
            battery.low,
            sensor.servo_position[0],
            sensor.servo_position[1],
            vision_valid,
            vision_dx,
            vision_dy,
            vision_sequence
        };
        control::balance_command command =
            control::action::step(action_state, context, CONTROL_PERIOD_MS);
        if(action_state.middle_calibration_success)
        {
            portENTER_CRITICAL(&request_lock);
            middle_calibration_finished = true;
            portEXIT_CRITICAL(&request_lock);
        }
        balance_status = control::balance::step(command, sensor, CONTROL_PERIOD_MS);
        hw::motor::publish_command(balance_status.motor);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

/**
 * @brief 执行非实时硬件和服务维护
 *
 * 电池、指示灯、WiFi 和网页会话按各自周期分频。传感器由独立任务采样，
 * 控制任务只读取最近快照。
 *
 * @param arg RTOS 任务参数
 */
void control::service_task_entry(void *arg)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    uint32_t battery_timer_ms = BATTERY_PERIOD_MS;
    uint32_t indicator_timer_ms = INDICATOR_PERIOD_MS;
    uint32_t network_timer_ms = NETWORK_PERIOD_MS;

    while(true)
    {
        battery_timer_ms += SERVICE_PERIOD_MS;
        indicator_timer_ms += SERVICE_PERIOD_MS;
        network_timer_ms += SERVICE_PERIOD_MS;

        if(battery_timer_ms >= BATTERY_PERIOD_MS)
        {
            battery_timer_ms = 0;
            hw::battery::update();
        }
        if(indicator_timer_ms >= INDICATOR_PERIOD_MS)
        {
            indicator_timer_ms = 0;
            hw::battery::state battery = read_battery();
            hw::indicator::update(battery.valid && battery.low, INDICATOR_PERIOD_MS);
        }
        if(network_timer_ms >= NETWORK_PERIOD_MS)
        {
            network_timer_ms = 0;
            hw::wifi::update();
            io::web::update();
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(SERVICE_PERIOD_MS));
    }
}

/* ---- control 公共 API ---- */

/**
 * @brief 请求执行舵机中位校准动作
 *
 * @return 请求已记录时返回 true
 */
bool control::request_middle_calibration()
{
    portENTER_CRITICAL(&request_lock);
    middle_calibration_requested = true;
    middle_calibration_finished = false;
    portEXIT_CRITICAL(&request_lock);
    return true;
}

/**
 * @brief 查询舵机中位校准是否已经成功
 *
 * @return 已完成时返回 true
 */
bool control::middle_calibration_success()
{
    bool finished = false;
    portENTER_CRITICAL(&request_lock);
    finished = middle_calibration_finished;
    portEXIT_CRITICAL(&request_lock);
    return finished;
}

/**
 * @brief 初始化控制状态、平衡算法和输入路由
 */
void control::init()
{
    balance::init();
    balance_info = balance::get_info();
    action::init(action_state);
    input_router::init();
    leg_state = control::action::leg_runtime{};
    input_state = control::control_input{};
    balance_status = control::status{};
    camera_angle = 90.0f;
    camera_speed = 0.0f;
    last_camera_angle = -1;
    portENTER_CRITICAL(&request_lock);
    middle_calibration_requested = false;
    middle_calibration_finished = false;
    portEXIT_CRITICAL(&request_lock);
}
