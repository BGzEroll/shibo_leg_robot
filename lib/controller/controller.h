#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>

namespace controller
{
    class actuator_port;
    class motion_port;

    bool middle_calibration_success();
    void mark_middle_calibration_success();
    bool request_middle_calibration();
    void update(uint32_t tick_ms);
    void configure(actuator_port &configured_actuator, motion_port &configured_motion);
    void init();
}

#endif
