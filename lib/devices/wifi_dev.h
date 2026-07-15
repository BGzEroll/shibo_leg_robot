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

    /**
     * @brief 切换网页遥控使用的低延迟 WiFi 模式
     *
     * @param enabled 是否关闭 WiFi 休眠以降低遥控延迟
     */
    void set_low_latency_mode(bool enabled);
    void update();
    void init();
}

#endif
