#ifndef IO_WEB_H
#define IO_WEB_H

#include <Arduino.h>
#include "control/input.h"

namespace io
{
    namespace web
    {
        bool latest_input(control::remote_input &out);
        bool init();
        void update();
    }
}

#endif
