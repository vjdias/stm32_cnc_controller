#include "Services/Safety/safety_service.h"
#include "Services/Log/log_service.h"
#include "Protocol/frame_defs.h"
#include <stdio.h>

LOG_SVC_DEFINE(LOG_SVC_SAFETY, "safety");

static volatile safety_state_t g_state = SAFETY_NORMAL;

void safety_service_init(void) {
	g_state = SAFETY_NORMAL;

	LOGT_THIS(LOG_STATE_START, PROTO_OK, "init", "normal");
}
void safety_estop_assert(void) {
	g_state = SAFETY_ESTOP;

	LOGT_THIS(LOG_STATE_ESTOP_ASSERT, PROTO_OK, "estop", "assert");
}
void safety_estop_release(void) {
	if (g_state == SAFETY_ESTOP)
		g_state = SAFETY_RECOVERY_WAIT;

	LOGT_THIS(LOG_STATE_ESTOP_RELEASE, PROTO_OK, "estop", "release");
}
int safety_is_safe(void) {
	return g_state == SAFETY_NORMAL;
}
