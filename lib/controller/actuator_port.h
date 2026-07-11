#ifndef ACTUATOR_PORT_H
#define ACTUATOR_PORT_H

#include "actuator_intent.h"

namespace controller
{
    class actuator_port
    {
        public:
            virtual ~actuator_port() = default;

        public:
            virtual actuator_feedback feedback() const = 0;
            virtual void apply(const actuator_intent &intent) = 0;
    };
}

#endif
