#ifndef WIFI_DEV_H
#define WIFI_DEV_H

#include <Arduino.h>
#include <IPAddress.h>

namespace wifi_dev
{
    struct network
    {
        char ssid[33];
        int8_t rssi;
        bool secure;
    };

    bool station_connected();
    bool config_portal_active();
    bool connect_and_save(const String &ssid, const String &password, IPAddress &ip);
    bool scan_networks(network *items, uint8_t max_count, uint8_t &count);
    bool connect_and_save_station(const char *ssid, const char *password, IPAddress &ip);
    IPAddress station_ip();
    void shutdown();
    void update();
    void init();
}

#endif
