#ifndef BALANCE_CORE_H
#define BALANCE_CORE_H

#include <Arduino.h>

namespace balance_core
{
	struct target_t
	{
		float linear_vel = 0.0f;
		float yaw_rate = 0.0f;
		float direct_left = 0.0f;
		float direct_right = 0.0f;
	};

	struct command_t
	{
		bool enable_motor = false;
		bool enable_balance = false;
		bool enable_steering = false;

		bool reset_reference = false;
		bool reset_yaw_integral = false;

		bool direct_output = false;
		bool suppress_linear_feedback = false;
		bool suppress_yaw_feedback = false;
		bool suppress_yaw_integral = false;
		bool recover_active = false;
		float output_blend = 1.0f;
	};

	struct status_snapshot
	{
		uint32_t timestamp_us = 0;
		float pitch_angle = 0.0f;
		float pitch_rate = 0.0f;
		float avg_linear_pos = 0.0f;
		float avg_linear_vel = 0.0f;
		float yaw_angle = 0.0f;
		float yaw_rate = 0.0f;
		float reference_linear_vel = 0.0f;
		float reference_yaw_rate = 0.0f;
		float input[2]{};
		float feedback_vector[6]{};
		float output[2]{};
		float roll_angle = 0.0f;
		float leg_height[2]{};
		float avg_leg_height = 0.0f;
	};

	void init();
	void set_target(const target_t &target);
	void set_command(const command_t &command);
	bool get_status(status_snapshot &out);

	float max_linear_vel();
	float max_steer_vel();
	float wheel_radius();

	void core_task_entry(void *arg);
	void control_task_entry(void *arg);
}

#endif
