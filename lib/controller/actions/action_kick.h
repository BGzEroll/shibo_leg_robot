#ifndef ACTION_KICK_H
#define ACTION_KICK_H

#include "../actions.h"

namespace controller
{
    namespace actions
    {
        balance_request kick_base_command(action_io &ctx);
        void begin_kick_exit(action_state &state);
        balance_request update_kick_place(action_state &state, action_io &ctx, uint32_t tick_ms);
        balance_request update_kick_run(action_state &state, action_io &ctx, uint32_t tick_ms);
    }
}

#endif
