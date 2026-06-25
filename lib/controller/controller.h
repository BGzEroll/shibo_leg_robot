#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

namespace controller
{
	void update(uint32_t tick_ms);
	void init();
}

#endif
