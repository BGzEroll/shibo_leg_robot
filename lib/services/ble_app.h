#ifndef BLE_APP_H
#define BLE_APP_H

#include <Arduino.h>

namespace ble_app
{
    void init();
    void task_entry(void *arg);
}

#endif
