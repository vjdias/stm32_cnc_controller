#include "Services/Probe/probe_service.h"

static probe_status_t g_probe;

void probe_service_init(void) {
	g_probe.axis_done_mask = 0;
	g_probe.error_flags = 0;
	g_probe.latched_pos_x = g_probe.latched_pos_y = g_probe.latched_pos_z = 0;
}
const probe_status_t* probe_status_get(void) {
	return &g_probe;
}

void probe_on_move_probe_level(const uint8_t *frame, uint32_t len) {
	(void) frame;
	(void) len; /* implementar FSM de probe */
}
