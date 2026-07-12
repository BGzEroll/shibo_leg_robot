#ifndef ACTIONS_H
#define ACTIONS_H

#include <Arduino.h>
#include "SimpleFOC.h"
#include "ports/actuator_ports.h"
#include "contracts/leg_config.h"
#include "balance_core.h"
#include "control_input.h"
#include "host_comm.h"

namespace controller
{
    namespace actions
    {
        class action;
    }

    enum class mode_id : uint8_t
    {
        BOOT = 0,
        BALANCE,
        SIT,
        JUMP,
        STOP,
        KICK_PLACE,
        KICK_RUN,
        MIDDLE_CALIBRATION
    };

    enum class jump_command : uint8_t
    {
        IN_PLACE = 0,
        FORWARD,
        BACKWARD,
        TURN_LEFT,
        TURN_RIGHT
    };

    enum class balance_drive_mode : uint8_t
    {
        STOP = 0,
        BALANCE,
        DIRECT_OUTPUT,
        RECOVER
    };

    struct leg_runtime
    {
        void reset_roll_pid()
        {
            roll_pid = PIDController{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
        }

        float roll_adjust = 0.0f;
        float height_base = leg_contract::HEIGHT_BASE;
        PIDController roll_pid{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
        LowPassFilter roll_lpf{0.3f};
    };

    struct balance_request
    {
        balance_drive_mode mode = balance_drive_mode::STOP;
        bool enable_steering = false;
        bool reset_reference = false;
        bool reset_yaw_integral = false;
        float linear_vel = 0.0f;
        float yaw_rate = 0.0f;
        float direct_left = 0.0f;
        float direct_right = 0.0f;
        float recover_blend = 1.0f;
        bool enable_linear_feedback = true;
        bool enable_yaw_feedback = true;
        bool enable_yaw_integral = true;
    };

    struct action_io
    {
        control_input &input;
        balance_core::motion_status &status;
        leg_runtime &leg;
        float max_linear_vel;
        float max_steer_vel;
        bool battery_valid;
        bool battery_low;
        bool sit_exit_locked;
        actuator_port::leg_status leg_status;
        host_comm::vision_measurement vision;
        bool vision_valid;
    };

    struct action_enter_params
    {
        jump_command jump = jump_command::IN_PLACE;
    };

    struct action_result
    {
        balance_request balance;
        action_request request = action_request::NONE;
    };

    struct action_state
    {
        mode_id mode = mode_id::BOOT;
        actions::action *current = nullptr;
        bool sit_exit_locked = false;
    };

    namespace actions
    {
        enum phase : uint8_t
        {
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

        struct action_runtime
        {
            uint8_t phase = PREPARE;
            uint32_t timer = 0;
            uint32_t ready_timer = 0;
            uint32_t elapsed = 0;
        };

        class action
        {
            public:
                virtual ~action() = default;

            public:
                virtual mode_id mode() const = 0;
                virtual void enter(action_io &ctx, mode_id previous,
                    const action_enter_params &params) = 0;
                virtual action_result update(action_io &ctx, uint32_t tick_ms) = 0;
                virtual void exit(action_io &ctx, mode_id next) = 0;
        };
    }

    void actions_init(action_state &state, const actuator_port::services &actuators);
    mode_id actions_mode(const action_state &state);
    balance_request actions_update(action_state &state, action_io &ctx, uint32_t tick_ms);
}

#endif
