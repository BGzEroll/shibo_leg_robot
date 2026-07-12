#include "actions.h"

#include "actions/action_common.h"
#include "actions/action_jump.h"
#include "actions/action_kick.h"
#include "actions/action_sit.h"

/* ---- 基础动作对象 ---- */

// 管理 BOOT 模式的舵机初始化和恢复站立流程。
class boot_action_impl : public controller::actions::action
{
    public:
        controller::mode_id mode() const override
        {
            return controller::mode_id::BOOT;
        }

        void enter(controller::action_io &ctx, controller::mode_id previous,
            const controller::action_enter_params &params) override
        {
            runtime = controller::actions::action_runtime{};
        }

        controller::action_result update(controller::action_io &ctx, uint32_t tick_ms) override
        {
            controller::action_result result;
            switch(runtime.phase)
            {
                case controller::actions::PREPARE:
                    controller::actions::set_torque(0);
                    runtime.phase = controller::actions::WAIT_SIGNAL;
                    break;

                case controller::actions::WAIT_SIGNAL:
                    if(ctx.input.request == controller::action_request::BOOT_CONFIRM &&
                       ctx.battery_valid &&
                       !ctx.battery_low)
                    {
                        runtime.phase = controller::actions::INIT;
                    }
                    break;

                case controller::actions::INIT:
                    controller::actions::set_pose(leg_contract::LEFT_MIN, leg_contract::RIGHT_MIN, 450, 250);
                    controller::actions::reset_leg(ctx.leg);
                    runtime.phase = controller::actions::INIT_PREPARE;
                    break;

                case controller::actions::INIT_PREPARE:
                    if((runtime.timer += tick_ms) >= 350)
                    {
                        runtime.timer = 0;
                        runtime.elapsed = 0;
                        runtime.ready_timer = 0;
                        runtime.phase = controller::actions::INIT_RECOVER;
                    }
                    break;

                case controller::actions::INIT_RECOVER:
                    result.balance = controller::actions::recover_command(runtime, ctx);
                    if(controller::actions::recover_ready(runtime, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
                    {
                        result.request = controller::action_request::ACTION_DONE;
                    }
                    break;
            }
            return result;
        }

        void exit(controller::action_io &ctx, controller::mode_id next) override
        {
        }

    private:
        controller::actions::action_runtime runtime;
};

// 管理 BALANCE 模式的常规平衡、腿部调节和语义请求上报。
class balance_action_impl : public controller::actions::action
{
    public:
        controller::mode_id mode() const override
        {
            return controller::mode_id::BALANCE;
        }

        void enter(controller::action_io &ctx, controller::mode_id previous,
            const controller::action_enter_params &params) override
        {
        }

        controller::action_result update(controller::action_io &ctx, uint32_t tick_ms) override
        {
            controller::action_result result;
            result.balance.mode = controller::balance_drive_mode::BALANCE;
            result.balance.enable_steering = true;
            result.balance.linear_vel = ctx.input.linear_cmd;
            result.balance.yaw_rate = ctx.input.yaw_cmd;
            controller::actions::run_leg_control(ctx);

            if(ctx.input.reset_leg)
            {
                controller::actions::reset_leg(ctx.leg);
            }

            if(ctx.input.middle_calibration_request)
            {
                result.request = controller::action_request::MIDDLE_CALIBRATION;
            }
            else
            {
                result.request = ctx.input.request;
            }
            return result;
        }

        void exit(controller::action_io &ctx, controller::mode_id next) override
        {
        }
};

// 管理 STOP 模式的空输出和重新 BOOT 请求上报。
class stop_action_impl : public controller::actions::action
{
    public:
        controller::mode_id mode() const override
        {
            return controller::mode_id::STOP;
        }

        void enter(controller::action_io &ctx, controller::mode_id previous,
            const controller::action_enter_params &params) override
        {
        }

        controller::action_result update(controller::action_io &ctx, uint32_t tick_ms) override
        {
            controller::action_result result;
            result.request = ctx.input.request;
            return result;
        }

        void exit(controller::action_io &ctx, controller::mode_id next) override
        {
        }
};

static boot_action_impl boot_action_instance;
static balance_action_impl balance_action_instance;
static stop_action_impl stop_action_instance;

/* ---- 动作调度内部流程 ---- */

/**
 * @brief 按模式获取静态动作对象
 *
 * @param mode 动作模式
 *
 * @return 动作对象
 */
static controller::actions::action &action_for_mode(controller::mode_id mode)
{
    switch(mode)
    {
        case controller::mode_id::BOOT:
            return boot_action_instance;

        case controller::mode_id::BALANCE:
            return balance_action_instance;

        case controller::mode_id::SIT:
            return controller::actions::sit_action();

        case controller::mode_id::JUMP:
            return controller::actions::jump_action();

        case controller::mode_id::KICK_PLACE:
            return controller::actions::kick_place_action();

        case controller::mode_id::KICK_RUN:
            return controller::actions::kick_run_action();

        case controller::mode_id::MIDDLE_CALIBRATION:
            return controller::actions::middle_calibration_action();

        case controller::mode_id::STOP:
        default:
            return stop_action_instance;
    }
}

/**
 * @brief 切换当前动作对象并执行生命周期回调
 *
 * @param state 动作调度状态
 * @param ctx 动作输入输出上下文
 * @param next_mode 目标模式
 * @param params 进入目标动作的参数
 */
static void switch_action(controller::action_state &state, controller::action_io &ctx,
    controller::mode_id next_mode, const controller::action_enter_params &params = controller::action_enter_params{})
{
    controller::mode_id previous = state.mode;
    controller::actions::action *previous_action = state.current;
    controller::actions::action &next_action = action_for_mode(next_mode);

    if(previous_action)
    {
        previous_action->exit(ctx, next_mode);
    }

    state.mode = next_mode;
    state.current = &next_action;
    next_action.enter(ctx, previous, params);
}

/**
 * @brief 根据电池状态更新坐下后的起身锁
 *
 * @param state 动作调度状态
 * @param ctx 动作输入输出上下文
 */
static void update_sit_exit_lock(controller::action_state &state, const controller::action_io &ctx)
{
    if(ctx.battery_valid && !ctx.battery_low)
    {
        state.sit_exit_locked = false;
        return;
    }

    bool sit_mode =
        state.mode == controller::mode_id::SIT ||
        state.mode == controller::mode_id::MIDDLE_CALIBRATION;
    if(sit_mode && ctx.battery_valid && ctx.battery_low)
    {
        state.sit_exit_locked = true;
    }
}

/**
 * @brief 将跳跃语义请求转换为动作进入参数
 *
 * @param request 动作语义请求
 *
 * @return 动作进入参数
 */
static controller::action_enter_params jump_params_for_request(controller::action_request request)
{
    controller::action_enter_params params;
    switch(request)
    {
        case controller::action_request::JUMP_FORWARD:
            params.jump = controller::jump_command::FORWARD;
            break;

        case controller::action_request::JUMP_BACKWARD:
            params.jump = controller::jump_command::BACKWARD;
            break;

        case controller::action_request::JUMP_LEFT:
            params.jump = controller::jump_command::TURN_LEFT;
            break;

        case controller::action_request::JUMP_RIGHT:
            params.jump = controller::jump_command::TURN_RIGHT;
            break;

        case controller::action_request::JUMP_IN_PLACE:
        default:
            params.jump = controller::jump_command::IN_PLACE;
            break;
    }
    return params;
}

/**
 * @brief 应用动作语义请求产生的模式切换
 *
 * @param state 动作调度状态
 * @param ctx 动作输入输出上下文
 * @param request 动作语义请求
 */
static void apply_transition(controller::action_state &state, controller::action_io &ctx,
    controller::action_request request)
{
    if(request == controller::action_request::NONE){return;}

    if(state.mode != controller::mode_id::STOP &&
       request == controller::action_request::STOP)
    {
        switch_action(state, ctx, controller::mode_id::STOP);
        return;
    }

    switch(state.mode)
    {
        case controller::mode_id::BOOT:
            if(request == controller::action_request::ACTION_DONE)
            {
                switch_action(state, ctx, controller::mode_id::BALANCE);
            }
            break;

        case controller::mode_id::BALANCE:
            if(request == controller::action_request::RESET_BALANCE)
            {
                switch_action(state, ctx, controller::mode_id::BALANCE);
            }
            else if(request == controller::action_request::SIT)
            {
                switch_action(state, ctx, controller::mode_id::SIT);
            }
            else if(request == controller::action_request::MIDDLE_CALIBRATION)
            {
                switch_action(state, ctx, controller::mode_id::MIDDLE_CALIBRATION);
            }
            else if(request == controller::action_request::JUMP_IN_PLACE ||
                    request == controller::action_request::JUMP_FORWARD ||
                    request == controller::action_request::JUMP_BACKWARD ||
                    request == controller::action_request::JUMP_LEFT ||
                    request == controller::action_request::JUMP_RIGHT)
            {
                switch_action(state, ctx, controller::mode_id::JUMP, jump_params_for_request(request));
            }
            else if(request == controller::action_request::KICK_PLACE)
            {
                switch_action(state, ctx, controller::mode_id::KICK_PLACE);
            }
            else if(request == controller::action_request::KICK_RUN)
            {
                switch_action(state, ctx, controller::mode_id::KICK_RUN);
            }
            break;

        case controller::mode_id::SIT:
            if(request == controller::action_request::MIDDLE_CALIBRATION)
            {
                switch_action(state, ctx, controller::mode_id::MIDDLE_CALIBRATION);
            }
            else if(request == controller::action_request::ACTION_DONE && !state.sit_exit_locked)
            {
                switch_action(state, ctx, controller::mode_id::BALANCE);
            }
            break;

        case controller::mode_id::MIDDLE_CALIBRATION:
            if(request == controller::action_request::ACTION_DONE && !state.sit_exit_locked)
            {
                switch_action(state, ctx, controller::mode_id::BALANCE);
            }
            break;

        case controller::mode_id::JUMP:
            if(request == controller::action_request::ACTION_DONE)
            {
                switch_action(state, ctx, controller::mode_id::BALANCE);
            }
            break;

        case controller::mode_id::KICK_PLACE:
            if(request == controller::action_request::KICK_RUN)
            {
                switch_action(state, ctx, controller::mode_id::KICK_RUN);
            }
            else if(request == controller::action_request::ACTION_DONE)
            {
                switch_action(state, ctx, controller::mode_id::BALANCE);
            }
            break;

        case controller::mode_id::KICK_RUN:
            if(request == controller::action_request::KICK_PLACE)
            {
                switch_action(state, ctx, controller::mode_id::KICK_PLACE);
            }
            else if(request == controller::action_request::ACTION_DONE)
            {
                switch_action(state, ctx, controller::mode_id::BALANCE);
            }
            break;

        case controller::mode_id::STOP:
            if(request == controller::action_request::BOOT &&
               !state.sit_exit_locked)
            {
                switch_action(state, ctx, controller::mode_id::BOOT);
            }
            break;

        default:
            break;
    }
}

/* ---- 动作调度 API ---- */

/**
 * @brief 初始化动作调度状态
 *
 * @param state 动作调度状态
 */
void controller::actions_init(controller::action_state &state, const actuator_port::services &actuators)
{
    controller::actions::bind_actuators(actuators);
    state.mode = controller::mode_id::BOOT;
    state.current = &boot_action_instance;
    state.sit_exit_locked = false;
}

/**
 * @brief 获取当前动作模式
 *
 * @param state 动作调度状态
 *
 * @return 当前动作模式
 */
controller::mode_id controller::actions_mode(const controller::action_state &state)
{
    return state.mode;
}

/**
 * @brief 按当前动作对象更新状态机并生成平衡请求
 *
 * @param state 动作调度状态
 * @param ctx 动作输入输出上下文
 * @param tick_ms 本次更新周期，单位毫秒
 *
 * @return 生成的平衡请求
 */
controller::balance_request controller::actions_update(controller::action_state &state, controller::action_io &ctx,
    uint32_t tick_ms)
{
    if(!state.current)
    {
        state.current = &action_for_mode(state.mode);
    }

    update_sit_exit_lock(state, ctx);
    ctx.sit_exit_locked = state.sit_exit_locked;

    if(state.mode != controller::mode_id::STOP &&
       ctx.input.request == controller::action_request::STOP)
    {
        switch_action(state, ctx, controller::mode_id::STOP);
        return controller::balance_request{};
    }

    controller::action_result result = state.current->update(ctx, tick_ms);
    apply_transition(state, ctx, result.request);
    return result.balance;
}
