#ifndef INPUT_ROUTER_H
#define INPUT_ROUTER_H

#include <Arduino.h>

namespace controller
{
    enum class mode_id : uint8_t;
    struct control_input;
    class input_provider;

    namespace input_router
    {
        void update(mode_id mode, float max_linear_vel, float max_steer_vel, control_input &out);
        void configure(input_provider *const *providers, uint8_t count);
        void init();
    }
}

#endif
