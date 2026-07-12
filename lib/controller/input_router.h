#ifndef INPUT_ROUTER_H
#define INPUT_ROUTER_H

#include <Arduino.h>
#include "esp_http_server.h"
#include "host_comm.h"
#include "ports/latest_value.h"
#include "xbox_dev.h"

namespace controller
{
    enum class mode_id : uint8_t;
    struct control_input;

    namespace input_router
    {
        struct input_ports
        {
            port::latest_reader<xbox_dev::data> xbox;
            port::latest_reader<esp_http_server::remote_input_data> web;
            port::latest_reader<host_comm::remote_data> host;
        };

        void update(mode_id mode, float max_linear_vel, float max_steer_vel, control_input &out);
        void init(const input_ports &inputs);
    }
}

#endif
