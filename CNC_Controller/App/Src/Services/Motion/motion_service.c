#include "Services/Motion/motion_service.h"
#include "Services/Log/log_service.h"
#include "Protocol/frame_defs.h"
#include <stdio.h>

LOG_SVC_DEFINE(LOG_SVC_MOTION, "motion");

static motion_status_t g_status;

void motion_service_init(void) {
	g_status.state = MOTION_IDLE;
	g_status.queue_depth = 0;
	g_status.pctX = g_status.pctY = g_status.pctZ = 0;
	g_status.pidErrX = g_status.pidErrY = g_status.pidErrZ = 0;

	LOGT_THIS(LOG_STATE_START, PROTO_OK, "init", "ok");
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

	LOGA_THIS(LOG_STATE_RECEIVED, PROTO_OK, "queue_add", "queue_depth=%u", (unsigned)g_status.queue_depth);
}
void motion_on_move_queue_status(const uint8_t *frame, uint32_t len) {
	(void) frame;
	(void) len;

	LOGA_THIS(LOG_STATE_RECEIVED, PROTO_OK, "queue_status", "queue_depth=%u,state=%u", (unsigned)g_status.queue_depth, (unsigned)g_status.state);
}
void motion_on_start_move(const uint8_t *frame, uint32_t len) {
	(void) frame;
	(void) len;
	if (g_status.queue_depth)
		g_status.state = MOTION_RUNNING;

	LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "start_move", "%s", (g_status.state == MOTION_RUNNING ? "running" : "ignored"));
}
void motion_on_move_end(const uint8_t *frame, uint32_t len) {
	(void) frame;
	(void) len;
	g_status.state = MOTION_STOPPING;

	LOGT_THIS(LOG_STATE_APPLIED, PROTO_OK, "move_end", "stopping");
}
