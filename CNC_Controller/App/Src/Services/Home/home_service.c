#include "Services/Home/home_service.h"
#include "Services/Log/log_service.h"
#include "Protocol/frame_defs.h"
#include <stdio.h>

LOG_SVC_DEFINE(LOG_SVC_HOME, "home");

static home_status_t g_home;

void home_service_init(void) {
	g_home.axis_done_mask = 0;
	g_home.error_flags = 0;

	LOGT_THIS(LOG_STATE_START, PROTO_OK, "init", "ok");
}
const home_status_t* home_status_get(void) {
	return &g_home;
}

void home_on_move_home(const uint8_t *frame, uint32_t len) {
	(void) frame;
	(void) len; /* implementar FSM de homing */

	LOGT_THIS(LOG_STATE_RECEIVED, PROTO_OK, "move_home", "not_implemented");
}
