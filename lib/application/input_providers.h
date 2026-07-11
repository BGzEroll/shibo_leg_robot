#ifndef INPUT_PROVIDERS_H
#define INPUT_PROVIDERS_H

#include "input_provider.h"

namespace application
{
    controller::input_provider &xbox_input();
    controller::input_provider &web_input();
    controller::input_provider &host_input();
}

#endif
