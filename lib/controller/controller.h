#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

namespace controller
{
	bool request_middle_calibration();
	void update(uint32_t tick_ms);
	void init();
}

#endif
