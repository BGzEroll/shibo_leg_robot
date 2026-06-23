#ifndef BALANCE_CORE_H
#define BALANCE_CORE_H

#include <Arduino.h>
#include "lqi.h"

namespace controller {
	enum class mode_id : uint8_t;
}

namespace balance_core {

	struct balance_command {
		bool enable_motor = false;
		bool enable_balance = false;
		bool enable_steering = false;

		bool reset_reference = false;
		bool reset_yaw_integral = false;

		float target_linear_vel = 0.0f;
		float target_yaw_rate = 0.0f;

		bool manual_output = false;
		float manual_left = 0.0f;
		float manual_right = 0.0f;

		bool suppress_linear_feedback = false;
		bool suppress_yaw_feedback = false;
		bool suppress_yaw_integral = false;
		bool recover_active = false;
		float output_blend = 1.0f;
	};

	struct sensor_snapshot {
		bool imu_valid = false;
		bool encoder_valid = false;
		uint32_t timestamp_us = 0;
		controller::lqi::feedback_state feedback{};
		float roll_angle = 0.0f;
		float leg_height[2]{};
		float avg_leg_height = 0.0f;
		int16_t servo_position[2]{};
	};

	struct balance_status {
		uint32_t timestamp_us = 0;
		controller::mode_id mode{};
		controller::lqi::feedback_state feedback{};
		controller::lqi::reference_state reference{};
		float input[2]{};
		float feedback_vector[6]{};
		float output[2]{};
		float roll_angle = 0.0f;
		float leg_height[2]{};
		float avg_leg_height = 0.0f;
	};

	float servo_count_to_height(int16_t position);

	void init();
	void set_command(const balance_command &cmd);
	bool get_status(balance_status &out);
	void set_mode(controller::mode_id mode);

	float max_linear_vel();
	float max_steer_vel();
	float wheel_radius();

	void io_task(void *arg);
	void control_task(void *arg);

}

#endif
