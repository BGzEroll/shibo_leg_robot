#ifndef ACTIONS_H
#define ACTIONS_H

#include <Arduino.h>
#include "SimpleFOC.h"
#include "balance_core.h"
#include "sts3032.h"

namespace controller {

	enum class mode_id : uint8_t {
		BOOT = 0,
		BALANCE,
		SIT,
		JUMP,
		STOP
	};

	enum class jump_command : uint8_t {
		IN_PLACE = 0,
		FORWARD,
		BACKWARD,
		TURN_LEFT,
		TURN_RIGHT
	};

	struct control_input {
		uint16_t raw_buttons = 0;
		uint16_t buttons = 0;
		uint16_t pressed_buttons = 0;
		float axes[6]{};
		float linear_cmd = 0.0f;
		float yaw_cmd = 0.0f;
	};

	struct leg_runtime {
		void reset_roll_pid()
		{
			roll_pid = PIDController{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
		}

		float roll_adjust = 0.0f;
		float height_base = (float)LEG_HEIGHT_BASE;
		PIDController roll_pid{8.0f, 30.0f, 0.0f, 100000.0f, 450.0f};
		LowPassFilter roll_lpf{0.3f};
	};

	struct action_io {
		control_input &input;
		balance_core::balance_status &status;
		leg_runtime &leg;
		float max_linear_vel;
	};

	struct action_state {
		mode_id mode = mode_id::BOOT;
		uint8_t phase = 0;
		uint32_t timer = 0;
		uint32_t ready_timer = 0;
		uint32_t elapsed = 0;
		int8_t jump_linear_dir = 0;
		int8_t jump_turn_dir = 0;
		float jump_target_yaw = 0.0f;
		float jump_linear_cmd = 0.0f;
		float jump_yaw_cmd = 0.0f;
	};

	void actions_init(action_state &state);
	mode_id actions_mode(const action_state &state);
	balance_core::balance_command actions_update(action_state &state, action_io &ctx, uint32_t tick_ms);

}

#endif
