#ifndef HW_WIFI_H
#define HW_WIFI_H

#include <Arduino.h>
#include <IPAddress.h>

namespace hw
{
    namespace wifi
    {
        bool station_connected();
        bool config_portal_active();
        bool connect_and_save(const String &ssid, const String &password, IPAddress &ip);
        IPAddress station_ip();

        void set_low_latency_mode(bool enabled);
        void init();
        void update();
    }
}

#endif
