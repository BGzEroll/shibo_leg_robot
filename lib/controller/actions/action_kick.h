#ifndef ACTION_KICK_H
#define ACTION_KICK_H

#include "../actions.h"

namespace controller
{
	namespace actions
	{
		balance_request update_kick_place(action_state &state, action_io &ctx, uint32_t tick_ms);
		balance_request update_kick_run(action_state &state, action_io &ctx, uint32_t tick_ms);
	}
}

#endif
