#ifndef HW_GAMEPAD_H
#define HW_GAMEPAD_H

#include <Arduino.h>
#include "control/input.h"

namespace hw
{
    namespace gamepad
    {
        struct ble_device
        {
            String address;
            String name;
            int8_t rssi = 0;
            bool xbox = false;
            bool connectable = false;
        };

        bool latest_input(control::remote_input &out);
        bool connected();
        String target_address();
        bool scan_ble(ble_device *devices, uint8_t max_count, uint8_t &count, uint32_t duration_ms);
        bool set_target_address(const String &address);

        void init();
        void task_entry(void *arg);
    }
}

#endif
