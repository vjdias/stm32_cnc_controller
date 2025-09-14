#include "services/motion/motion_service.h"

static motion_status_t g_status;

void motion_service_init(void) {
	g_status.state = MOTION_IDLE;
	g_status.queue_depth = 0;
	g_status.pctX = g_status.pctY = g_status.pctZ = 0;
	g_status.pidErrX = g_status.pidErrY = g_status.pidErrZ = 0;
}

const motion_status_t* motion_status_get(void) {
	return &g_status;
}

void motion_on_tim6_tick(void) { /* DDA feed aqui futuramente */
}
void motion_on_tim7_tick(void) { /* Atualização de status/PID aqui futuramente */
}

void motion_on_move_queue_add(const uint8_t *frame, uint32_t len) {
	(void) frame;
	(void) len;
	if (g_status.state == MOTION_IDLE)
		g_status.state = MOTION_QUEUED;
	g_status.queue_depth++;
}
void motion_on_move_queue_status(const uint8_t *frame, uint32_t len) {
	(void) frame;
	(void) len;
}
void motion_on_start_move(const uint8_t *frame, uint32_t len) {
	(void) frame;
	(void) len;
	if (g_status.queue_depth)
		g_status.state = MOTION_RUNNING;
}
void motion_on_move_end(const uint8_t *frame, uint32_t len) {
	(void) frame;
	(void) len;
	g_status.state = MOTION_STOPPING;
}
