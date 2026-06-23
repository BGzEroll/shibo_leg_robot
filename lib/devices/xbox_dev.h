#ifndef XBOX_DEV_H
#define XBOX_DEV_H

#include "xbox.h"

namespace xbox_dev
{
	struct data
	{
		uint32_t timestamp_us;
		uint16_t buttons;
		float axes[6];
	};

	extern xbox gamepad;

	QueueHandle_t queue();
	void init();
	void task_entry(void *arg);
}

#endif
