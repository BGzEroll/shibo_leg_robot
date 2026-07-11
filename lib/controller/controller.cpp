#include "controller.h"

#include "actions.h"
#include "actuator_port.h"
#include "battery.h"
#include "host_comm.h"
#include "input_router.h"
#include "motion_port.h"

namespace host_comm
{
    void init();
}

/* ---- 控制器运行状态 ---- */

static controller::action_state action;
static controller::leg_runtime leg;
static controller::control_input input;
static controller::motion_status status;
static controller::motion_info motion_limits;
static float cam_angle = 90.0f;
static float cam_speed = 0.0f;
static int16_t cam_last_angle = -1;
static bool middle_calibration_requested = false;
static bool middle_calibration_finished = false;
static portMUX_TYPE request_lock = portMUX_INITIALIZER_UNLOCKED;
static controller::actuator_port *robot_actuator = nullptr;
static controller::motion_port *robot_motion = nullptr;

/* ---- 控制器内部流程 ---- */

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
 * @param actuator 执行器意图输出
 */
static void update_camera(uint32_t tick_ms, controller::actuator_intent &actuator)
{
    float target_speed = (float)input.camera_direction * 120.0f;

    float dt = (float)tick_ms * 1.0e-3f;
    cam_speed += (target_speed - cam_speed) * (1.0f - expf(-dt / 0.05f));
    cam_angle = constrain(
        cam_angle + cam_speed * dt,
        (float)controller::robot_model::CAMERA_SERVO_MIN,
        (float)controller::robot_model::CAMERA_SERVO_MAX);

    if((int16_t)cam_angle != cam_last_angle)
    {
        cam_last_angle = (int16_t)cam_angle;
        actuator.accessory.camera_valid = true;
        actuator.accessory.camera_angle = (uint16_t)cam_angle;
    }
}

/**
 * @brief 读取动作层需要的执行器反馈
 *
 * @return 执行器反馈快照
 */
static controller::actuator_feedback read_actuator_feedback()
{
    if(!robot_actuator){return controller::actuator_feedback{};}
    return robot_actuator->feedback();
}

/**
 * @brief 读取最新电池状态
 *
 * @return 最新电池状态，队列不可用时返回无效状态
 */
static battery::data read_battery()
{
    battery::data data;
    if(!battery::queue() ||
       xQueuePeek(battery::queue(), &data, 0) != pdTRUE)
    {
        return battery::data{};
    }
    return data;
}

/* ---- controller 公共 API ---- */

/**
 * @brief 查询舵机中位校准流程是否已经成功执行
 *
 * @return 已成功执行时返回 true
 */
bool controller::middle_calibration_success()
{
    bool finished = false;
    portENTER_CRITICAL(&request_lock);
    finished = middle_calibration_finished;
    portEXIT_CRITICAL(&request_lock);
    return finished;
}

/**
 * @brief 标记舵机中位校准流程已经成功执行
 */
void controller::mark_middle_calibration_success()
{
    portENTER_CRITICAL(&request_lock);
    middle_calibration_finished = true;
    portEXIT_CRITICAL(&request_lock);
}

/**
 * @brief 请求执行舵机中位校准动作流程
 *
 * @return 请求写入成功时返回 true
 */
bool controller::request_middle_calibration()
{
    portENTER_CRITICAL(&request_lock);
    middle_calibration_requested = true;
    middle_calibration_finished = false;
    portEXIT_CRITICAL(&request_lock);
    return true;
}

/**
 * @brief 执行一次上层控制器更新
 *
 * @param tick_ms 本次更新周期，单位毫秒
 */
void controller::update(uint32_t tick_ms)
{
    if(robot_motion){robot_motion->latest_status(status);}
    controller::input_router::update(
        controller::actions_mode(action),
        motion_limits.max_linear_vel,
        motion_limits.max_steer_vel,
        input);
    input.middle_calibration_request = consume_middle_calibration_request();
    controller::actuator_intent actuator;
    controller::actuator_feedback actuator_feedback = read_actuator_feedback();
    update_camera(tick_ms, actuator);

    battery::data battery_data = read_battery();
    controller::action_io ctx{
        input,
        status,
        leg,
        actuator,
        actuator_feedback,
        motion_limits.max_linear_vel,
        motion_limits.max_steer_vel,
        battery_data.valid,
        battery_data.low,
        false
    };
    controller::balance_request request = controller::actions_update(action, ctx, tick_ms);

    if(robot_actuator){robot_actuator->apply(actuator);}
    if(robot_motion){robot_motion->apply(request);}
}

/**
 * @brief 配置控制器使用的执行器端口
 *
 * @param configured_actuator 执行器端口
 * @param configured_motion 运动控制端口
 */
void controller::configure(controller::actuator_port &configured_actuator,
    controller::motion_port &configured_motion)
{
    robot_actuator = &configured_actuator;
    robot_motion = &configured_motion;
}

/**
 * @brief 初始化控制器模块及其内部子模块
 */
void controller::init()
{
    host_comm::init();
    if(robot_motion)
    {
        robot_motion->init();
        motion_limits = robot_motion->info();
    }
    controller::actions_init(action);
    controller::input_router::init();
    leg = controller::leg_runtime{};
}
