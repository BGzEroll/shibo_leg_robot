#ifndef WIFI_DEV_H
#define WIFI_DEV_H

#include <Arduino.h>
#include <IPAddress.h>

namespace wifi_dev
{
	bool station_connected();
	bool config_portal_active();
	bool connect_and_save(const String &ssid, const String &password, IPAddress &ip);
	IPAddress station_ip();
	void update();
	void init();
}

#endif
