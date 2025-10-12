#include "Services/Motion/motion_service.h"
#include "Services/Log/log_service.h"
#include "Protocol/frame_defs.h"
#include "Protocol/Requests/move_queue_add_request.h"
#include "Protocol/Requests/move_queue_status_request.h"
#include "Protocol/Requests/start_move_request.h"
#include "Protocol/Requests/move_end_request.h"
#include "Protocol/Responses/move_queue_add_ack_response.h"
#include "Protocol/Responses/move_queue_status_response.h"
#include "Protocol/Responses/start_move_response.h"
#include "Protocol/Responses/move_end_response.h"
#include "app.h"
#include <string.h>
#include <stdio.h>

LOG_SVC_DEFINE(LOG_SVC_MOTION, "motion");

// Estado público resumido
static motion_status_t g_status;

// Fila simples de movimentos (ring buffer minimalista)
#ifndef MOTION_QUEUE_CAPACITY
#define MOTION_QUEUE_CAPACITY 8u
#endif

typedef struct {
    uint8_t  frameId;
    uint8_t  dirMask;
    uint16_t vx, vy, vz;   // taxas relativas (placeholder)
    uint32_t sx, sy, sz;   // passos/intervalos por eixo
} motion_slot_t;

static motion_slot_t g_queue[MOTION_QUEUE_CAPACITY];
static uint8_t g_q_head = 0u, g_q_tail = 0u, g_q_count = 0u;

// Movimento ativo (para cálculo de progresso)
static motion_slot_t g_active = {0};
static uint32_t g_rem_x = 0, g_rem_y = 0, g_rem_z = 0; // restantes
static uint8_t g_active_valid = 0u;

static int q_push(const motion_slot_t *s)
{
    if (!s) return PROTO_ERR_ARG;
    if (g_q_count >= MOTION_QUEUE_CAPACITY) return PROTO_ERR_RANGE;
    g_queue[g_q_tail] = *s;
    g_q_tail = (uint8_t)((g_q_tail + 1u) % MOTION_QUEUE_CAPACITY);
    g_q_count++;
    g_status.queue_depth = g_q_count;
    return PROTO_OK;
}

static int q_pop(motion_slot_t *out)
{
    if (g_q_count == 0u) return PROTO_ERR_RANGE;
    if (out) *out = g_queue[g_q_head];
    g_q_head = (uint8_t)((g_q_head + 1u) % MOTION_QUEUE_CAPACITY);
    g_q_count--;
    g_status.queue_depth = g_q_count;
    return PROTO_OK;
}

static void active_load_from_slot(const motion_slot_t *s)
{
    if (!s) { g_active_valid = 0u; return; }
    g_active = *s;
    g_rem_x = s->sx;
    g_rem_y = s->sy;
    g_rem_z = s->sz;
    g_active_valid = 1u;
    // Reinicia progresso
    g_status.pctX = g_status.pctY = g_status.pctZ = 0u;
}

static void active_clear(void)
{
    memset(&g_active, 0, sizeof g_active);
    g_rem_x = g_rem_y = g_rem_z = 0u;
    g_active_valid = 0u;
}

static void motion_push_queue_add_ack(uint8_t frame_id, uint8_t status)
{
    uint8_t raw[6];
    move_queue_add_ack_resp_t resp = { frame_id, status };
    if (move_queue_add_ack_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    } else {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "resp", "queue_add_ack_encode_fail");
    }
}

static void motion_push_queue_status(uint8_t frame_id)
{
    uint8_t raw[12];
    move_queue_status_resp_t resp;
    resp.frameId = frame_id;
    resp.status = (uint8_t)g_status.state;
    resp.pidErrX = (uint8_t)g_status.pidErrX;
    resp.pidErrY = (uint8_t)g_status.pidErrY;
    resp.pidErrZ = (uint8_t)g_status.pidErrZ;
    resp.pctX = g_status.pctX;
    resp.pctY = g_status.pctY;
    resp.pctZ = g_status.pctZ;
    if (move_queue_status_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    } else {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "resp", "queue_status_encode_fail");
    }
}

static void motion_push_start_move(uint8_t frame_id)
{
    uint8_t raw[4];
    start_move_resp_t resp = { frame_id };
    if (start_move_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    } else {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "resp", "start_move_encode_fail");
    }
}

static void motion_push_move_end(uint8_t frame_id)
{
    uint8_t raw[4];
    move_end_resp_t resp = { frame_id };
    if (move_end_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    } else {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "resp", "move_end_encode_fail");
    }
}

void motion_service_init(void) {
	g_status.state = MOTION_IDLE;
	g_status.queue_depth = 0;
	g_status.pctX = g_status.pctY = g_status.pctZ = 0;
	g_status.pidErrX = g_status.pidErrY = g_status.pidErrZ = 0;

    g_q_head = g_q_tail = g_q_count = 0u;
    active_clear();

	LOGT_THIS(LOG_STATE_START, PROTO_OK, "init", "ok");
}

const motion_status_t* motion_status_get(void) {
	return &g_status;
}

void motion_on_tim6_tick(void) {
    // Alimenta DDA simplificado: consome 1 unidade por eixo se running
    if (g_status.state != MOTION_RUNNING || !g_active_valid) return;

    if (g_rem_x) g_rem_x--;
    if (g_rem_y) g_rem_y--;
    if (g_rem_z) g_rem_z--;

    // Atualiza progresso em % (0..100) com saturação
    if (g_active.sx) {
        uint32_t done = g_active.sx - g_rem_x;
        uint32_t pct = (done >= g_active.sx) ? 100u : (uint32_t)((done * 100u) / (g_active.sx));
        g_status.pctX = (uint8_t)((pct > 100u) ? 100u : pct);
    } else {
        g_status.pctX = 100u;
    }
    if (g_active.sy) {
        uint32_t done = g_active.sy - g_rem_y;
        uint32_t pct = (done >= g_active.sy) ? 100u : (uint32_t)((done * 100u) / (g_active.sy));
        g_status.pctY = (uint8_t)((pct > 100u) ? 100u : pct);
    } else {
        g_status.pctY = 100u;
    }
    if (g_active.sz) {
        uint32_t done = g_active.sz - g_rem_z;
        uint32_t pct = (done >= g_active.sz) ? 100u : (uint32_t)((done * 100u) / (g_active.sz));
        g_status.pctZ = (uint8_t)((pct > 100u) ? 100u : pct);
    } else {
        g_status.pctZ = 100u;
    }

    // Checa término do slot atual
    if (g_rem_x == 0u && g_rem_y == 0u && g_rem_z == 0u) {
        active_clear();
        if (g_q_count > 0u) {
            motion_slot_t next;
            (void)q_pop(&next);
            active_load_from_slot(&next);
            g_status.state = MOTION_RUNNING;
        } else {
            g_status.state = MOTION_DONE;
        }
    }
}
void motion_on_tim7_tick(void) {
    // Atualização periódica de PID (placeholder: zera erros)
    g_status.pidErrX = 0;
    g_status.pidErrY = 0;
    g_status.pidErrZ = 0;
}

void motion_on_move_queue_add(const uint8_t *frame, uint32_t len) {
    move_queue_add_req_t req;
    if (!frame) return;
    proto_result_t st = move_queue_add_req_decoder(frame, len, &req);
    if (st != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, st, "queue_add", "decode_fail");
        return;
    }

    motion_slot_t slot;
    slot.frameId = req.frameId;
    slot.dirMask = req.dirMask;
    slot.vx = req.vx; slot.vy = req.vy; slot.vz = req.vz;
    slot.sx = req.sx; slot.sy = req.sy; slot.sz = req.sz;

    uint8_t ack_status = PROTO_OK;
    if (q_push(&slot) != PROTO_OK) {
        ack_status = (uint8_t)PROTO_ERR_RANGE; // fila cheia
    } else {
        if (g_status.state == MOTION_IDLE || g_status.state == MOTION_DONE)
            g_status.state = MOTION_QUEUED;
        // Se não há ativo e ainda não estamos rodando, preparamos o próximo
        if (!g_active_valid && g_status.state == MOTION_QUEUED && g_q_count == 1u) {
            // Pré-carrega o slot, start_move irá apenas mudar o estado para RUNNING
            motion_slot_t tmp;
            (void)q_pop(&tmp);
            active_load_from_slot(&tmp);
            // Decremento do count já refletido por q_pop; como pré-carregamos, consideramos um slot "em processamento"
        }
    }

    motion_push_queue_add_ack(req.frameId, ack_status);

    LOGA_THIS(LOG_STATE_RECEIVED, ack_status, "queue_add",
              "queue_depth=%u,cap=%u,frame=%u",
              (unsigned)g_q_count, (unsigned)MOTION_QUEUE_CAPACITY, (unsigned)req.frameId);
}
void motion_on_move_queue_status(const uint8_t *frame, uint32_t len) {
    move_queue_status_req_t req;
    if (!frame) return;
    proto_result_t st = move_queue_status_req_decoder(frame, len, &req);
    if (st != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, st, "queue_status", "decode_fail");
        return;
    }
    motion_push_queue_status(req.frameId);

    LOGA_THIS(LOG_STATE_RECEIVED, PROTO_OK, "queue_status",
              "depth=%u,state=%u,pct=%u,%u,%u",
              (unsigned)g_q_count, (unsigned)g_status.state,
              (unsigned)g_status.pctX, (unsigned)g_status.pctY, (unsigned)g_status.pctZ);
}
void motion_on_start_move(const uint8_t *frame, uint32_t len) {
    start_move_req_t req;
    if (!frame) return;
    proto_result_t st = start_move_req_decoder(frame, len, &req);
    if (st != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, st, "start_move", "decode_fail");
        return;
    }

    if (g_active_valid) {
        g_status.state = MOTION_RUNNING;
    } else if (g_q_count > 0u) {
        motion_slot_t next;
        if (q_pop(&next) == PROTO_OK) {
            active_load_from_slot(&next);
            g_status.state = MOTION_RUNNING;
        }
    }

    motion_push_start_move(req.frameId);

    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "start_move",
              "%s", (g_status.state == MOTION_RUNNING ? "running" : "ignored"));
}
void motion_on_move_end(const uint8_t *frame, uint32_t len) {
	move_end_req_t req;
	if (!frame) return;
	proto_result_t st = move_end_req_decoder(frame, len, &req);
	if (st != PROTO_OK) {
		LOGA_THIS(LOG_STATE_ERROR, st, "move_end", "decode_fail");
		return;
	}

	// Solicita parada: limpa fila e marca estado
	g_q_head = g_q_tail = g_q_count = 0u;
	g_status.queue_depth = 0u;
	active_clear();
	g_status.state = MOTION_STOPPING;

	motion_push_move_end(req.frameId);

	LOGT_THIS(LOG_STATE_APPLIED, PROTO_OK, "move_end", "stopping");
}
