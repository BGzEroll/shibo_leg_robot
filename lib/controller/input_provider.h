#ifndef INPUT_PROVIDER_H
#define INPUT_PROVIDER_H

#include "control_input.h"

namespace controller
{
    struct input_sample
    {
        uint32_t timestamp_us = 0;
        uint16_t buttons = 0;
        float axes[6]{};
    };

    class input_provider
    {
        public:
            virtual ~input_provider() = default;

        public:
            virtual input_source source() const = 0;
            virtual bool claims_control(uint32_t now_us) = 0;
            virtual bool latest(input_sample &out) = 0;
    };
}

#endif
