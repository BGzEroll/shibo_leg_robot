#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include "SimpleFOC.h"
#include "bus/i2c_bus.h"

namespace motor
{
	struct encoder_data
	{
		uint32_t timestamp_us;
		float left_shaft_angle;
		float right_shaft_angle;
		float left_shaft_velocity;
		float right_shaft_velocity;
	};

	struct target_data
	{
		uint32_t timestamp_us;
		float left_torque;
		float right_torque;
	};

	extern BLDCMotor left;
	extern BLDCMotor right;

	QueueHandle_t encoder_queue();
	QueueHandle_t target_queue();
	void init();
}

#endif
