#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include "balance_core.h"
#include "battery.h"
#include "esp_http_server.h"
#include "host_comm.h"
#include "input_router.h"
#include "ports/latest_value.h"

namespace controller
{
    struct input_ports
    {
        port::latest_reader<battery::data> battery_status;
        port::latest_reader<balance_core::motion_status> motion_status;
        port::latest_reader<host_comm::vision_measurement> vision;
        port::latest_reader<actuator_port::leg_status> leg_status;
        port::latest_reader<esp_http_server::calibration_request> calibration;
        input_router::input_ports control_sources;
        actuator_port::services actuators;
    };

    struct output_ports
    {
        port::latest_writer<balance_core::command> balance_command;
        port::latest_writer<esp_http_server::calibration_status> calibration;
    };

    void mark_middle_calibration_success();
    void update(uint32_t tick_ms);
    void init(const input_ports &inputs, const output_ports &outputs);
}

#endif
