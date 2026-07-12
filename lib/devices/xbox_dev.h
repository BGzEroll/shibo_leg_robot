#ifndef XBOX_DEV_H
#define XBOX_DEV_H

#include <Arduino.h>
#include "ports/latest_value.h"

namespace xbox_dev
{
    struct data
    {
        uint32_t timestamp_us;
        uint16_t buttons;
        float axes[6];
        bool connected = false;
    };

    struct ble_device
    {
        String address;
        String name;
        int8_t rssi;
        bool xbox;
        bool connectable;
    };

    struct output_ports
    {
        port::latest_writer<data> control;
    };

    bool connected();
    String target_address();
    bool scan_ble(ble_device *devices, uint8_t max_count, uint8_t &count, uint32_t duration_ms);
    bool set_target_address(const String &address);
    void init(const output_ports &outputs);
    void task_entry(void *arg);
}

#endif
