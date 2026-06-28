#ifndef ACTIONS_H
#define ACTIONS_H

#include <Arduino.h>
#include "SimpleFOC.h"
#include "balance_core.h"
#include "sts3032.h"

namespace controller
{
	enum class mode_id : uint8_t
	{
		BOOT = 0,
		BALANCE,
		SIT,
		JUMP,
		STOP,
		KICK_PLACE,
		KICK_RUN
	};

	enum class jump_command : uint8_t
	{
		IN_PLACE = 0,
		FORWARD,
		BACKWARD,
		TURN_LEFT,
		TURN_RIGHT
	};

	struct control_input
	{
		uint16_t raw_buttons = 0;
		uint16_t buttons = 0;
		uint16_t pressed_buttons = 0;
		float axes[6]{};
		float linear_cmd = 0.0f;
		float yaw_cmd = 0.0f;
	};

	struct leg_runtime
	{
		void reset_roll_pid()
		{
			roll_pid = PIDController{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
		}

		float roll_adjust = 0.0f;
		float height_base = (float)LEG_HEIGHT_BASE;
		PIDController roll_pid{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
		LowPassFilter roll_lpf{0.3f};
	};

	struct balance_request
	{
		balance_core::target_t target;
		balance_core::command_t command;
	};

	struct action_io
	{
		control_input &input;
		balance_core::status_snapshot &status;
		leg_runtime &leg;
		float max_linear_vel;
	};

	struct jump_runtime
	{
		int8_t linear_dir = 0;
		int8_t turn_dir = 0;
		float target_yaw = 0.0f;
		float linear_cmd = 0.0f;
		float yaw_cmd = 0.0f;
	};

	struct kick_runtime
	{
		float cam_angle = 90.0f;
		float target_yaw = 0.0f;
		int16_t last_dy = 0;
		uint16_t frontier_angle = 181;
		uint32_t last_dy_time = 0;
		uint32_t kick_timer = 0;
		uint32_t post_timer = 0;
		bool chased = false;
		bool aligned = false;
		bool kicking = false;
		bool post_kick = false;
	};

	struct action_state
	{
		mode_id mode = mode_id::BOOT;
		uint8_t phase = 0;
		uint32_t timer = 0;
		uint32_t ready_timer = 0;
		uint32_t elapsed = 0;
		jump_runtime jump;
		kick_runtime kick;
	};

	void actions_init(action_state &state);
	mode_id actions_mode(const action_state &state);
	balance_request actions_update(action_state &state, action_io &ctx, uint32_t tick_ms);
}

#endif
