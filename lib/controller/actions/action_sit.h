#ifndef ACTION_SIT_H
#define ACTION_SIT_H

#include "action_common.h"

namespace controller
{
    namespace actions
    {
        balance_request update_sit(action_state &state, action_io &ctx, uint32_t tick_ms);
        balance_request update_middle_calibration(action_state &state, action_io &ctx, uint32_t tick_ms);
    }
}

#endif
