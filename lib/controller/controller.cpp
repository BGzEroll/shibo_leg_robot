#include "controller.h"

#include "actions.h"

/* ---- 控制器运行状态 ---- */

static controller::action_state action;
static controller::leg_runtime leg;
static controller::control_input input;
static balance_core::motion_status status;
static balance_core::info balance_info;
static float cam_angle = 90.0f;
static float cam_speed = 0.0f;
static int16_t cam_last_angle = -1;
static bool middle_calibration_requested = false;
static bool middle_calibration_finished = false;
static portMUX_TYPE request_lock = portMUX_INITIALIZER_UNLOCKED;
static controller::input_ports module_inputs;
static controller::output_ports module_outputs;
static uint32_t calibration_sequence = 0;

/* ---- 控制器内部流程 ---- */

/**
 * @brief 将动作层语义请求转换为平衡核心当前控制命令
 *
 * @param request 动作层平衡请求
 */
static void apply_balance_request(const controller::balance_request &request)
{
    balance_core::command command;

    switch(request.mode)
    {
        case controller::balance_drive_mode::BALANCE:
            command.motion.enable_motor = true;
            command.motion.enable_balance = true;
            command.motion.enable_steering = request.enable_steering;
            command.motion.linear_vel = request.linear_vel;
            command.motion.yaw_rate = request.yaw_rate;
            break;

        case controller::balance_drive_mode::DIRECT_OUTPUT:
            command.motion.enable_motor = true;
            command.direct_output.enable = true;
            command.direct_output.left = request.direct_left;
            command.direct_output.right = request.direct_right;
            break;

        case controller::balance_drive_mode::RECOVER:
            command.motion.enable_motor = true;
            command.motion.enable_balance = true;
            command.motion.enable_steering = request.enable_steering;
            command.recover.enable = true;
            command.recover.output_blend = request.recover_blend;
            break;

        case controller::balance_drive_mode::STOP:
        default:
            break;
    }

    command.motion.reset_reference = request.reset_reference;
    command.motion.reset_yaw_integral = request.reset_yaw_integral;
    command.feedback.enable_linear_feedback = request.enable_linear_feedback;
    command.feedback.enable_yaw_feedback = request.enable_yaw_feedback;
    command.feedback.enable_yaw_integral = request.enable_yaw_integral;
    module_outputs.balance_command.publish(command);
}

/**
 * @brief 读取并登记新的中位校准请求
 */
static void update_middle_calibration_request()
{
    esp_http_server::calibration_request request;
    if(!module_inputs.calibration.read(request) || request.sequence == calibration_sequence){return;}

    calibration_sequence = request.sequence;
    portENTER_CRITICAL(&request_lock);
    middle_calibration_requested = true;
    middle_calibration_finished = false;
    portEXIT_CRITICAL(&request_lock);

    esp_http_server::calibration_status status;
    status.sequence = calibration_sequence;
    module_outputs.calibration.publish(status);
}

/**
 * @brief 在可校准模式下取出舵机中位校准请求
 *
 * @return 有待处理请求时返回 true
 */
static bool consume_middle_calibration_request()
{
    controller::mode_id mode = controller::actions_mode(action);
    if(mode != controller::mode_id::BALANCE && mode != controller::mode_id::SIT){return false;}

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
    float target_speed = (float)input.camera_direction * 120.0f;

    float dt = (float)tick_ms * 1.0e-3f;
    cam_speed += (target_speed - cam_speed) * (1.0f - expf(-dt / 0.05f));
    cam_angle = constrain(cam_angle + cam_speed * dt, 0.0f, 180.0f);

    if((int16_t)cam_angle != cam_last_angle)
    {
        cam_last_angle = (int16_t)cam_angle;
        if(module_inputs.actuators.set_camera_angle)
        {
            module_inputs.actuators.set_camera_angle((uint16_t)cam_angle);
        }
    }
}

/**
 * @brief 读取最新电池状态
 *
 * @return 最新电池状态，队列不可用时返回无效状态
 */
static battery::data read_battery()
{
    battery::data data;
    if(!module_inputs.battery_status.read(data)){return battery::data{};}
    return data;
}

/* ---- controller 公共 API ---- */

/**
 * @brief 标记舵机中位校准流程已经成功执行
 */
void controller::mark_middle_calibration_success()
{
    portENTER_CRITICAL(&request_lock);
    middle_calibration_finished = true;
    portEXIT_CRITICAL(&request_lock);

    esp_http_server::calibration_status status;
    status.sequence = calibration_sequence;
    status.success = true;
    module_outputs.calibration.publish(status);
}

/**
 * @brief 执行一次上层控制器更新
 *
 * @param tick_ms 本次更新周期，单位毫秒
 */
void controller::update(uint32_t tick_ms)
{
    module_inputs.motion_status.read(status);
    update_middle_calibration_request();
    controller::input_router::update(
        controller::actions_mode(action),
        balance_info.max_linear_vel,
        balance_info.max_steer_vel,
        input);
    input.middle_calibration_request = consume_middle_calibration_request();
    update_camera(tick_ms);

    battery::data battery_data = read_battery();
    host_comm::vision_measurement vision;
    bool vision_valid = module_inputs.vision.read(vision) && vision.valid &&
        (uint32_t)(millis() - vision.timestamp_ms) <= 350;
    actuator_port::leg_status leg_status;
    module_inputs.leg_status.read(leg_status);
    controller::action_io ctx{
        input,
        status,
        leg,
        balance_info.max_linear_vel,
        balance_info.max_steer_vel,
        battery_data.valid,
        battery_data.low,
        false,
        leg_status,
        vision,
        vision_valid
    };
    controller::balance_request request = controller::actions_update(action, ctx, tick_ms);

    apply_balance_request(request);
}

/**
 * @brief 初始化动作管理模块及端口
 *
 * @param inputs 动作管理输入端口
 * @param outputs 动作管理输出端口
 */
void controller::init(const controller::input_ports &inputs, const controller::output_ports &outputs)
{
    module_inputs = inputs;
    module_outputs = outputs;
    balance_info = balance_core::get_info();
    controller::actions_init(action, inputs.actuators);
    controller::input_router::init(inputs.control_sources);
    leg = controller::leg_runtime{};
}
