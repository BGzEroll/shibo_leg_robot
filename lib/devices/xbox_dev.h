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

    struct ble_device
    {
        String address;
        String name;
        int8_t rssi;
        bool xbox;
        bool connectable;
    };

    QueueHandle_t queue();
    bool connected();
    String target_address();
    bool scan_ble(ble_device *devices, uint8_t max_count, uint8_t &count, uint32_t duration_ms);
    bool set_target_address(const String &address);
    void init();
    void task_entry(void *arg);
}

#endif
