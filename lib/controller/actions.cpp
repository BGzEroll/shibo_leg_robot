#include "actions.h"

#include "sts3032.h"

namespace controller {

enum phase : uint8_t {
    PREPARE = 0,
    WAIT_SIGNAL,
    INIT,
    INIT_PREPARE,
    INIT_RECOVER,
    MOVING,
    DONE,
    EXIT_PREPARE,
    EXIT_RECOVER,
    PUSH,
    FLY,
    LAND,
    RECOVER
};

static float wrap_pi(float angle)
{
    while(angle > PI){angle -= 2.0f * PI;}
    while(angle < -PI){angle += 2.0f * PI;}
    return angle;
}

static float angle_error(float target, float current)
{
    return wrap_pi(target - current);
}

static void set_pose(int16_t left, int16_t right, uint16_t speed, uint8_t accel)
{
    sts3032::set(SERVO_LEFT, left, speed, accel);
    sts3032::set(SERVO_RIGHT, right, speed, accel);
    sts3032::move();
}

static void set_torque(uint8_t type)
{
    sts3032::set_torque_switch(SERVO_LEFT, type);
    sts3032::set_torque_switch(SERVO_RIGHT, type);
}

static void reset_leg(leg_runtime &leg)
{
    leg.roll_adjust = 0.0f;
    leg.height_base = (float)LEG_HEIGHT_BASE;
    leg.reset_roll_pid();
}

static void run_leg_control(action_io &ctx)
{
    if((ctx.input.buttons & BTN_RIGHT) && !(ctx.input.buttons & ~BTN_RIGHT)){ctx.leg.roll_adjust += 0.025f;}
    if((ctx.input.buttons & BTN_LEFT) && !(ctx.input.buttons & ~BTN_LEFT)){ctx.leg.roll_adjust -= 0.025f;}
    if((ctx.input.buttons & BTN_UP) && !(ctx.input.buttons & ~BTN_UP)){ctx.leg.height_base -= 0.025f;}
    if((ctx.input.buttons & BTN_DOWN) && !(ctx.input.buttons & ~BTN_DOWN)){ctx.leg.height_base += 0.025f;}

    float roll_angle = ctx.leg.roll_lpf(ctx.status.roll_angle / (float)PI * 180.0f);
    float leg_add = ctx.leg.roll_pid(roll_angle - ctx.leg.roll_adjust);
    int16_t left = (int16_t)(2048.0f + 8.4f * (30.0f - ctx.leg.height_base) - leg_add);
    int16_t right = (int16_t)(2048.0f - 8.4f * (30.0f - ctx.leg.height_base) - leg_add);

    left = constrain(left, SERVO_LEFT_MIN, SERVO_LEFT_MAX);
    right = constrain(right, SERVO_RIGHT_MAX, SERVO_RIGHT_MIN);
    set_pose(left, right, 1000, 0);
}

static void begin_mode(action_state &state, mode_id mode, jump_command jump = jump_command::IN_PLACE)
{
    state.mode = mode;
    state.phase = PREPARE;
    state.timer = 0;
    state.ready_timer = 0;
    state.elapsed = 0;
    state.jump_linear_cmd = 0.0f;
    state.jump_yaw_cmd = 0.0f;

    if(mode != mode_id::JUMP){return;}

    state.jump_linear_dir = 0;
    state.jump_turn_dir = 0;
    if(jump == jump_command::FORWARD){state.jump_linear_dir = 1;}
    if(jump == jump_command::BACKWARD){state.jump_linear_dir = -1;}
    if(jump == jump_command::TURN_LEFT){state.jump_turn_dir = 1;}
    if(jump == jump_command::TURN_RIGHT){state.jump_turn_dir = -1;}
}

static bool recover_ready(action_state &state, const balance_status &status, uint32_t tick_ms,
    float pitch_limit, float rate_limit, uint32_t hold_ms, uint32_t timeout_ms)
{
    state.elapsed += tick_ms;
    if(fabsf(status.feedback.pitch_angle) < pitch_limit &&
       fabsf(status.feedback.pitch_rate) < rate_limit)
    {
        state.ready_timer += tick_ms;
    }
    else
    {
        state.ready_timer = 0;
    }

    return state.ready_timer >= hold_ms || state.elapsed >= timeout_ms;
}

static balance_command recover_command(action_state &state, action_io &ctx)
{
    balance_command cmd;
    cmd.enable_balance = true;
    cmd.enable_motor = true;
    cmd.recover_active = true;
    cmd.output_blend = constrain((float)state.elapsed * 1.0e-3f / 0.22f, 0.0f, 1.0f);
    run_leg_control(ctx);
    return cmd;
}

static balance_command update_boot(action_state &state, action_io &ctx, uint32_t tick_ms)
{
    balance_command cmd;
    switch(state.phase)
    {
        case PREPARE:
            set_torque(0);
            state.phase = WAIT_SIGNAL;
            break;

        case WAIT_SIGNAL:
            if(ctx.input.buttons & BTN_RB){state.phase = INIT;}
            break;

        case INIT:
            set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
            reset_leg(ctx.leg);
            cmd.reset_reference = true;
            state.phase = INIT_PREPARE;
            break;

        case INIT_PREPARE:
            if((state.timer += tick_ms) >= 350)
            {
                state.timer = 0;
                state.elapsed = 0;
                state.ready_timer = 0;
                cmd.reset_reference = true;
                state.phase = INIT_RECOVER;
            }
            break;

        case INIT_RECOVER:
            cmd = recover_command(state, ctx);
            if(recover_ready(state, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                cmd.reset_reference = true;
                begin_mode(state, mode_id::BALANCE);
            }
            break;
    }
    return cmd;
}

static balance_command update_balance(action_state &state, action_io &ctx)
{
    balance_command cmd;
    cmd.enable_balance = true;
    cmd.enable_motor = true;
    cmd.enable_steering = true;
    cmd.target_linear_vel = ctx.input.linear_cmd;
    cmd.target_yaw_rate = ctx.input.yaw_cmd;
    run_leg_control(ctx);

    if((ctx.input.pressed_buttons & BTN_LS) &&
       fabsf(ctx.input.linear_cmd) < ctx.max_linear_vel * 0.05f)
    {
        reset_leg(ctx.leg);
        cmd.reset_reference = true;
    }
    if(ctx.input.pressed_buttons & BTN_LB){begin_mode(state, mode_id::SIT);}
    if(ctx.input.pressed_buttons & BTN_RS){begin_mode(state, mode_id::JUMP, jump_command::IN_PLACE);}
    if(ctx.input.pressed_buttons & BTN_Y){begin_mode(state, mode_id::JUMP, jump_command::FORWARD);}
    if(ctx.input.pressed_buttons & BTN_A){begin_mode(state, mode_id::JUMP, jump_command::BACKWARD);}
    if(ctx.input.pressed_buttons & BTN_X){begin_mode(state, mode_id::JUMP, jump_command::TURN_LEFT);}
    if(ctx.input.pressed_buttons & BTN_B){begin_mode(state, mode_id::JUMP, jump_command::TURN_RIGHT);}
    return cmd;
}

static balance_command update_sit(action_state &state, action_io &ctx, uint32_t tick_ms)
{
    balance_command cmd;
    switch(state.phase)
    {
        case PREPARE:
            set_torque(2);
            state.phase = MOVING;
            break;

        case MOVING:
            cmd.enable_motor = true;
            cmd.manual_output = true;
            cmd.manual_left = -0.15f;
            cmd.manual_right = -0.15f;
            if(fabsf(ctx.status.feedback.pitch_angle) >= 0.25f || (state.timer += tick_ms) >= 5000)
            {
                state.timer = 0;
                state.phase = DONE;
            }
            break;

        case DONE:
            if(ctx.input.buttons & BTN_LS){set_torque(0);}
            if(ctx.input.buttons & BTN_RB)
            {
                set_pose(SERVO_LEFT_MIN, SERVO_RIGHT_MIN, 450, 250);
                reset_leg(ctx.leg);
                cmd.reset_reference = true;
                state.phase = EXIT_PREPARE;
            }
            break;

        case EXIT_PREPARE:
            if((state.timer += tick_ms) >= 350)
            {
                state.timer = 0;
                state.elapsed = 0;
                state.ready_timer = 0;
                cmd.reset_reference = true;
                state.phase = EXIT_RECOVER;
            }
            break;

        case EXIT_RECOVER:
            cmd = recover_command(state, ctx);
            if(recover_ready(state, ctx.status, tick_ms, 0.16f, 1.2f, 140, 2500))
            {
                cmd.reset_reference = true;
                begin_mode(state, mode_id::BALANCE);
            }
            break;
    }
    return cmd;
}

static balance_command update_jump_command(action_state &state, action_io &ctx)
{
    balance_command cmd;
    bool linear_jump = state.jump_linear_dir != 0;
    bool yaw_jump = state.jump_turn_dir != 0 || linear_jump;
    cmd.enable_balance = true;
    cmd.enable_motor = true;
    cmd.enable_steering = yaw_jump;
    cmd.suppress_yaw_integral = !yaw_jump;

    float push_vel = 0.0f;
    uint32_t push_ramp_ms = 80;
    if(state.jump_linear_dir > 0)
    {
        push_vel = min(ctx.max_linear_vel, 0.40f);
        push_ramp_ms = 160;
    }
    else if(state.jump_linear_dir < 0)
    {
        push_vel = min(ctx.max_linear_vel, 0.34f);
        push_ramp_ms = 240;
    }

    if(state.phase == PUSH)
    {
        float ramp = constrain((float)state.timer / (float)push_ramp_ms, 0.0f, 1.0f);
        state.jump_linear_cmd = (float)state.jump_linear_dir * push_vel * ramp;
    }
    else
    {
        state.jump_linear_cmd = 0.0f;
    }

    cmd.target_linear_vel = state.jump_linear_cmd;
    if(yaw_jump)
    {
        float err = angle_error(state.jump_target_yaw, ctx.status.feedback.yaw_angle);
        float ff = 0.0f;
        float kp = state.jump_turn_dir == 0 ? 3.0f : 1.0f;
        float max_rate = state.jump_turn_dir == 0 ? 1.8f : 0.6f;

        if(state.jump_turn_dir != 0)
        {
            if(state.phase == PUSH){ff = 1.2f; kp = 1.4f; max_rate = 1.8f;}
            if(state.phase == FLY){ff = 6.4f; kp = 2.0f; max_rate = 6.4f;}
            if(state.phase == LAND){kp = 0.35f; max_rate = 0.4f;}
            if(state.phase == RECOVER){kp = 0.8f; max_rate = 0.5f;}
        }

        state.jump_yaw_cmd = constrain((float)state.jump_turn_dir * ff + kp * err, -max_rate, max_rate);
        cmd.target_yaw_rate = state.jump_yaw_cmd;
    }

    if(state.jump_linear_dir == 0 || state.phase != PUSH){cmd.suppress_linear_feedback = true;}
    if(!yaw_jump){cmd.suppress_yaw_feedback = true;}
    return cmd;
}

static balance_command update_jump(action_state &state, action_io &ctx, uint32_t tick_ms)
{
    balance_command cmd = update_jump_command(state, ctx);

    switch(state.phase)
    {
        case PREPARE:
            state.jump_target_yaw = wrap_pi(ctx.status.feedback.yaw_angle +
                                            (float)state.jump_turn_dir * PI * 0.5f);
            set_pose(SERVO_LEFT_MIN + 60, SERVO_RIGHT_MIN - 60, 450, 250);
            cmd.reset_yaw_integral = true;
            state.phase = PUSH;
            state.timer = 0;
            break;

        case PUSH:
        {
            uint32_t wait_ms = state.jump_linear_dir > 0 ? 650 : (state.jump_linear_dir < 0 ? 700 : 200);
            if((state.timer += tick_ms) >= wait_ms)
            {
                set_pose(SERVO_LEFT_MAX + 20, SERVO_RIGHT_MAX - 20, 0, 0);
                state.timer = 0;
                state.phase = FLY;
            }
            break;
        }

        case FLY:
            if((state.timer += tick_ms) >= 130)
            {
                set_pose(SERVO_LEFT_MIN + 60, SERVO_RIGHT_MIN - 60, 0, 0);
                state.timer = 0;
                state.phase = LAND;
            }
            break;

        case LAND:
            if((state.timer += tick_ms) >= 260)
            {
                state.timer = 0;
                state.elapsed = 0;
                state.phase = RECOVER;
            }
            break;

        case RECOVER:
            state.elapsed += tick_ms;
            if(fabsf(ctx.status.feedback.pitch_angle) < 0.18f &&
               fabsf(ctx.status.feedback.pitch_rate) < 1.6f)
            {
                state.timer += tick_ms;
            }
            else
            {
                state.timer = 0;
            }

            if(state.timer >= 80 || state.elapsed >= 350)
            {
                cmd.reset_yaw_integral = true;
                begin_mode(state, mode_id::BALANCE);
            }
            break;
    }

    return cmd;
}

void actions_init(action_state &state)
{
    begin_mode(state, mode_id::BOOT);
}

mode_id actions_mode(const action_state &state)
{
    return state.mode;
}

balance_command actions_update(action_state &state, action_io &ctx, uint32_t tick_ms)
{
    if(state.mode != mode_id::STOP && (ctx.input.pressed_buttons & BTN_START))
    {
        begin_mode(state, mode_id::STOP);
        balance_command cmd;
        cmd.reset_reference = true;
        return cmd;
    }

    switch(state.mode)
    {
        case mode_id::BOOT:
            return update_boot(state, ctx, tick_ms);

        case mode_id::BALANCE:
            return update_balance(state, ctx);

        case mode_id::SIT:
            return update_sit(state, ctx, tick_ms);

        case mode_id::JUMP:
            return update_jump(state, ctx, tick_ms);

        case mode_id::STOP:
            if(ctx.input.buttons & BTN_RB)
            {
                begin_mode(state, mode_id::BOOT);
            }
            return balance_command{};

        default:
            return balance_command{};
    }
}

}
