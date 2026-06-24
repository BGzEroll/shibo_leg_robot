#ifndef ACTION_JUMP_H
#define ACTION_JUMP_H

#include "action_common.h"

namespace controller
{
	namespace actions
	{
		balance_request update_jump(action_state &state, action_io &ctx, uint32_t tick_ms);
	}
}

#endif
