#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

namespace controller
{
#ifdef DEBUG_MODE
	struct debug_snapshot_t
	{
		uint8_t mode = 0;
		uint8_t phase = 0;
		bool kick_mode = false;
		float kick_cam_angle = 0.0f;
		float kick_cam_error = 0.0f;
		float kick_cam_rate = 0.0f;
		float kick_yaw_rate = 0.0f;
	};

#endif

	void update(uint32_t tick_ms);
#ifdef DEBUG_MODE
	bool debug_snapshot(debug_snapshot_t &out);
#endif
	void init();
}

#endif
