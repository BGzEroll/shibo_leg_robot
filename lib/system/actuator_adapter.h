#ifndef ACTUATOR_ADAPTER_H
#define ACTUATOR_ADAPTER_H

#include "ports/actuator_ports.h"
#include "ports/latest_value.h"

namespace actuator_adapter
{
    actuator_port::services services();
    void sample_leg_status(uint32_t now_us);
    void init(port::latest_writer<actuator_port::leg_status> status_output);
}

#endif
