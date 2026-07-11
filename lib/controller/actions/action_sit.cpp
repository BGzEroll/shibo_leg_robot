#include "action_sit.h"

#include "controller.h"

static constexpr int16_t SERVO_MIDDLE_COUNT = 2048;
static constexpr int16_t SIT_MIDDLE_READY_ERROR = 50;
static constexpr uint32_t MIDDLE_CALIBRATION_TORQUE_OFF_MS = 500;
static constexpr uint32_t MIDDLE_CALIBRATION_RUN_MS = 2000;
static constexpr uint32_t MIDDLE_CALIBRATION_SUCCESS_MS = 2500;

static controller::actions::action_runtime sit_runtime;

/* ---- 坐下与校准内部流程 ---- */

/**
 * @brief 判断左右腿舵机当前位置是否已经接近中位
 *
 * @param ctx 动作输入输出上下文
 *
 * @return 左右腿舵机都接近中位时返回 true
 */
static bool servo_middle_ready(const controller::action_io &ctx)
{
    int16_t left_error = abs(ctx.feedback.left_leg_position - SERVO_MIDDLE_COUNT);
    int16_t right_error = abs(ctx.feedback.right_leg_position - SERVO_MIDDLE_COUNT);
    return left_error <= SIT_MIDDLE_READY_ERROR && right_error <= SIT_MIDDLE_READY_ERROR;
}

/**
 * @brief 生成坐下直出电机请求
 *
 * @param cmd 平衡请求
 */
static void set_sit_direct_output(controller::balance_request &cmd)
{
    cmd.mode = controller::balance_drive_mode::DIRECT_OUTPUT;
    cmd.direct_left = -0.05f;
    cmd.direct_right = -0.05f;
}

/**
 * @brief 判断当前坐下阶段是否允许切换到中位校准
 *
 * @param phase 当前动作阶段
 *
 * @return 允许切换时返回 true
 */
static bool sit_phase_can_enter_middle_calibration(uint8_t phase)
{
    return phase == controller::actions::PREPARE ||
           phase == controller::actions::INIT_PREPARE ||
           phase == controller::actions::MOVING ||
           phase == controller::actions::DONE;
}

/**
 * @brief 更新坐下类流程状态机并生成动作结果
 *
 * @param runtime 坐下类流程运行状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 * @param calibration 是否在坐下完成后执行中位校准
 *
 * @return 生成的动作结果
 */
static controller::action_result update_sit_flow(controller::actions::action_runtime &runtime,
    controller::action_io &ctx, uint32_t tick_ms, bool calibration)
{
    controller::action_result result;
    switch(runtime.phase)
    {
        case controller::actions::PREPARE:
            result.balance.mode = controller::balance_drive_mode::BALANCE;
            result.balance.enable_steering = true;
            controller::actions::set_pose(
                ctx,
                controller::robot_model::SERVO_LEFT_MIN,
                controller::robot_model::SERVO_RIGHT_MIN,
                450,
                250);
            runtime.timer = 0;
            runtime.phase = controller::actions::INIT_PREPARE;
            break;

        case controller::actions::INIT_PREPARE:
            result.balance.mode = controller::balance_drive_mode::BALANCE;
            result.balance.enable_steering = true;
            if(servo_middle_ready(ctx))
            {
                runtime.timer = 0;
                controller::actions::set_torque(ctx, 2);
                set_sit_direct_output(result.balance);
                runtime.phase = controller::actions::MOVING;
            }
            break;

        case controller::actions::MOVING:
            runtime.timer += tick_ms;
            if(fabsf(ctx.status.pitch_angle) >= 0.25f)
            {
                runtime.timer = 0;
                result.balance.mode = controller::balance_drive_mode::STOP;
                runtime.phase = controller::actions::DONE;
                break;
            }
            set_sit_direct_output(result.balance);
            break;

        case controller::actions::DONE:
            if(calibration)
            {
                runtime.timer += tick_ms;
                if(runtime.timer >= MIDDLE_CALIBRATION_TORQUE_OFF_MS && runtime.ready_timer == 0)
                {
                    controller::actions::set_torque(ctx, 0);
                    runtime.ready_timer = 1;
                }
                if(runtime.timer >= MIDDLE_CALIBRATION_RUN_MS && runtime.elapsed == 0)
                {
                    ctx.actuator.calibrate_middle = true;
                    runtime.elapsed = 1;
                }
                if(runtime.timer >= MIDDLE_CALIBRATION_SUCCESS_MS && runtime.elapsed == 1)
                {
                    controller::mark_middle_calibration_success();
                    runtime.elapsed = 2;
                }
            }
            else if((runtime.timer += tick_ms) >= 10000 || ctx.input.disable_leg_torque)
            {
                controller::actions::set_torque(ctx, 0);
                runtime.timer = 10000;
            }
            if(ctx.input.exit_action && !ctx.sit_exit_locked)
            {
                controller::actions::set_pose(
                    ctx,
                    controller::robot_model::SERVO_LEFT_MIN,
                    controller::robot_model::SERVO_RIGHT_MIN,
                    450,
                    250);
                controller::actions::reset_leg(ctx.leg);
                runtime.phase = controller::actions::EXIT_PREPARE;
            }
            break;

        case controller::actions::EXIT_PREPARE:
            if((runtime.timer += tick_ms) >= 350)
            {
                runtime.timer = 0;
                runtime.elapsed = 0;
                runtime.ready_timer = 0;
                runtime.phase = controller::actions::EXIT_RECOVER;
            }
            break;

        case controller::actions::EXIT_RECOVER:
            result.balance = controller::actions::recover_command(runtime, ctx);
            if(controller::actions::recover_ready(runtime, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                result.request = controller::action_request::ACTION_DONE;
            }
            break;
    }

    return result;
}

// 管理 SIT 模式的坐下和起身恢复阶段。
class sit_action_impl : public controller::actions::action
{
    public:
        controller::mode_id mode() const override
        {
            return controller::mode_id::SIT;
        }

        void enter(controller::action_io &ctx, controller::mode_id previous,
            const controller::action_enter_params &params) override
        {
            sit_runtime = controller::actions::action_runtime{};
        }

        controller::action_result update(controller::action_io &ctx, uint32_t tick_ms) override
        {
            controller::action_result result = update_sit_flow(sit_runtime, ctx, tick_ms, false);
            if(ctx.input.middle_calibration_request &&
               sit_phase_can_enter_middle_calibration(sit_runtime.phase))
            {
                result.request = controller::action_request::MIDDLE_CALIBRATION;
            }
            return result;
        }

        void exit(controller::action_io &ctx, controller::mode_id next) override
        {
        }
};

// 管理 MIDDLE_CALIBRATION 模式，并复用坐下流程的姿态阶段。
class middle_calibration_action_impl : public controller::actions::action
{
    public:
        controller::mode_id mode() const override
        {
            return controller::mode_id::MIDDLE_CALIBRATION;
        }

        void enter(controller::action_io &ctx, controller::mode_id previous,
            const controller::action_enter_params &params) override
        {
            if(previous != controller::mode_id::SIT)
            {
                sit_runtime = controller::actions::action_runtime{};
                return;
            }

            sit_runtime.timer = 0;
            sit_runtime.ready_timer = 0;
            sit_runtime.elapsed = 0;
        }

        controller::action_result update(controller::action_io &ctx, uint32_t tick_ms) override
        {
            return update_sit_flow(sit_runtime, ctx, tick_ms, true);
        }

        void exit(controller::action_io &ctx, controller::mode_id next) override
        {
        }
};

static sit_action_impl sit_action_instance;
static middle_calibration_action_impl middle_calibration_action_instance;

/**
 * @brief 获取 SIT 动作对象
 *
 * @return SIT 动作对象
 */
controller::actions::action &controller::actions::sit_action()
{
    return sit_action_instance;
}

/**
 * @brief 获取 MIDDLE_CALIBRATION 动作对象
 *
 * @return MIDDLE_CALIBRATION 动作对象
 */
controller::actions::action &controller::actions::middle_calibration_action()
{
    return middle_calibration_action_instance;
}
