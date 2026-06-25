#ifndef WIFI_DEV_H
#define WIFI_DEV_H

#include <Arduino.h>
#include <IPAddress.h>

namespace wifi_dev
{
	void init();
	void task_entry(void *arg);
	bool station_connected();
	IPAddress station_ip();
}

#endif
