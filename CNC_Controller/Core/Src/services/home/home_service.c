#include "services/home/home_service.h"

static home_status_t g_home;

void home_service_init(void) {
	g_home.axis_done_mask = 0;
	g_home.error_flags = 0;
}
const home_status_t* home_status_get(void) {
	return &g_home;
}

void home_on_move_home(const uint8_t *frame, uint32_t len) {
	(void) frame;
	(void) len; /* implementar FSM de homing */
}
