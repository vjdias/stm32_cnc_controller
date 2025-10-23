#include "Services/Motion/motion_service.h"
#include "Services/Motion/motion_hw.h"
#include "Protocol/Requests/move_end_request.h"
#include "Protocol/Requests/move_queue_add_request.h"
#include "Protocol/Requests/move_queue_status_request.h"
#include "Protocol/Requests/start_move_request.h"
#include "Protocol/Responses/move_end_response.h"
#include "Protocol/Responses/move_queue_add_ack_response.h"
#include "Protocol/Responses/move_queue_status_response.h"
#include "Protocol/Responses/start_move_response.h"
#include "Protocol/frame_defs.h"
#include "Services/Log/log_service.h"
#include "Services/Safety/safety_service.h"
#include "app.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

LOG_SVC_DEFINE(LOG_SVC_MOTION, "motion");
#define MOTION_AXIS_COUNT 3u
#define MOTION_QUEUE_CAPACITY 64u
typedef struct {
	uint32_t total_steps;
	uint32_t target_steps;
	uint32_t emitted_steps;
	uint16_t velocity_per_tick;
	uint16_t kp, ki, kd;
	uint8_t step_high;
    uint16_t warmup_ticks; // ticks de espera antes de emitir o primeiro STEP
} motion_axis_state_t;
typedef struct {
	move_queue_add_req_t req;
} motion_queue_entry_t;
enum {
	AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2,
};
enum {
	MOTION_ACK_OK = 0, MOTION_ACK_INVALID = 1, MOTION_ACK_QUEUE_FULL = 2,
};
static motion_status_t g_status;
static motion_axis_state_t g_axis_state[MOTION_AXIS_COUNT];
static volatile uint8_t g_has_active_segment = 0u;
static motion_queue_entry_t g_queue[MOTION_QUEUE_CAPACITY];
static uint8_t g_queue_head = 0u;
static uint8_t g_queue_tail = 0u;
static uint8_t g_queue_count = 0u;
static int64_t g_encoder_position[MOTION_AXIS_COUNT];
static uint32_t g_encoder_last_raw[MOTION_AXIS_COUNT];
static int64_t g_encoder_origin[MOTION_AXIS_COUNT];
// Flags de teste/demonstração
static volatile uint8_t g_demo_continuous = 0u; // quando 1, gera passos continuamente
static const uint16_t g_demo_speed_table[4] = { 5u, 10u, 20u, 40u }; // ksteps/s aprox (1kHz)
static volatile uint8_t g_demo_speed_idx = 1u; // default ≈10k steps/s (compatível com versão anterior)
static inline uint32_t motion_lock(void) {
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	return primask;
}
static inline void motion_unlock(uint32_t primask) {
	__set_PRIMASK(primask);
}
static inline uint32_t motion_total_for_axis(const move_queue_add_req_t *req,
		uint8_t axis) {
	switch (axis) {
	case AXIS_X:
		return req->sx;
	case AXIS_Y:
		return req->sy;
	case AXIS_Z:
	default:
		return req->sz;
	}
}
static inline uint16_t motion_velocity_for_axis(const move_queue_add_req_t *req,
		uint8_t axis) {
	switch (axis) {
	case AXIS_X:
		return req->vx;
	case AXIS_Y:
		return req->vy;
	case AXIS_Z:
	default:
		return req->vz;
	}
}
static inline uint16_t motion_kp_for_axis(const move_queue_add_req_t *req,
		uint8_t axis) {
	switch (axis) {
	case AXIS_X:
		return req->kp_x;
	case AXIS_Y:
		return req->kp_y;
	case AXIS_Z:
	default:
		return req->kp_z;
	}
}
static inline uint16_t motion_ki_for_axis(const move_queue_add_req_t *req,
		uint8_t axis) {
	switch (axis) {
	case AXIS_X:
		return req->ki_x;
	case AXIS_Y:
		return req->ki_y;
	case AXIS_Z:
	default:
		return req->ki_z;
	}
}
static inline uint16_t motion_kd_for_axis(const move_queue_add_req_t *req,
		uint8_t axis) {
	switch (axis) {
	case AXIS_X:
		return req->kd_x;
	case AXIS_Y:
		return req->kd_y;
	case AXIS_Z:
	default:
		return req->kd_z;
	}
}
static inline int8_t motion_clamp_error(int32_t value) {
	if (value > 127)
		return 127;
	if (value < -128)
		return -128;
	return (int8_t) value;
}
static void motion_refresh_status_locked(void) {
	g_status.queue_depth = (uint8_t) (g_queue_count
			+ (g_has_active_segment ? 1u : 0u));
	for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
		const motion_axis_state_t *ax = &g_axis_state[axis];
		uint32_t total = ax->total_steps;
		uint32_t emitted = ax->emitted_steps;
		uint8_t pct = 0u;
		if (g_has_active_segment && total > 0u) {
			uint64_t scaled = (uint64_t) emitted * 100u;
			pct = (uint8_t) (scaled / total);
			if (pct > 100u)
				pct = 100u;
		} else if (total == 0u && g_has_active_segment) {
			pct = 100u;
		} else if (!g_has_active_segment && total > 0u) {
			pct = (emitted >= total) ?
					100u : (uint8_t) (((uint64_t) emitted * 100u) / total);
		}
		int64_t actual_counts = g_encoder_position[axis]
				- g_encoder_origin[axis];
		if (actual_counts > (int64_t) INT32_MAX)
			actual_counts = INT32_MAX;
		else if (actual_counts < (int64_t) INT32_MIN)
			actual_counts = INT32_MIN;
		int32_t desired_counts = (int32_t) ax->target_steps;
		int32_t err = desired_counts - (int32_t) actual_counts;
		int8_t err8 = motion_clamp_error(err);
		switch (axis) {
		case AXIS_X:
			g_status.pctX = pct;
			g_status.pidErrX = err8;
			break;
		case AXIS_Y:
			g_status.pctY = pct;
			g_status.pidErrY = err8;
			break;
		case AXIS_Z:
		default:
			g_status.pctZ = pct;
			g_status.pidErrZ = err8;
			break;
		}
	}
}
static void motion_stop_all_axes_locked(void) {
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
                motion_hw_step_low(axis);
                motion_hw_enable(axis, 0u);
                g_axis_state[axis].total_steps = 0u;
                g_axis_state[axis].target_steps = 0u;
                g_axis_state[axis].emitted_steps = 0u;
		g_axis_state[axis].velocity_per_tick = 0u;
		g_axis_state[axis].kp = 0u;
		g_axis_state[axis].ki = 0u;
		g_axis_state[axis].kd = 0u;
		g_axis_state[axis].step_high = 0u;
	}
}
static void motion_queue_clear_locked(void) {
	g_queue_head = 0u;
	g_queue_tail = 0u;
	g_queue_count = 0u;
}
static proto_result_t motion_queue_push_locked(const move_queue_add_req_t *req) {
	if (g_queue_count >= MOTION_QUEUE_CAPACITY)
		return PROTO_ERR_RANGE;
	g_queue[g_queue_tail].req = *req;
	g_queue_tail = (uint8_t) ((g_queue_tail + 1u) % MOTION_QUEUE_CAPACITY);
	++g_queue_count;
	motion_refresh_status_locked();
	return PROTO_OK;
}
static int motion_queue_pop_locked(move_queue_add_req_t *out) {
	if (g_queue_count == 0u)
		return 0;
	if (out)
		*out = g_queue[g_queue_head].req;
	g_queue_head = (uint8_t) ((g_queue_head + 1u) % MOTION_QUEUE_CAPACITY);
	--g_queue_count;
	return 1;
}
static void motion_begin_segment_locked(const move_queue_add_req_t *seg) {
	if (!seg)
		return;
	g_has_active_segment = 1u;
	for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
		motion_axis_state_t *ax = &g_axis_state[axis];
		uint32_t total = motion_total_for_axis(seg, axis);
		uint16_t velocity = motion_velocity_for_axis(seg, axis);
		ax->total_steps = total;
		ax->target_steps = 0u;
		ax->emitted_steps = 0u;
		ax->velocity_per_tick = velocity;
		ax->kp = motion_kp_for_axis(seg, axis);
		ax->ki = motion_ki_for_axis(seg, axis);
		ax->kd = motion_kd_for_axis(seg, axis);
		ax->step_high = 0u;
            /* Aguardar estabilização do driver após EN=LOW (datasheet TMC5160 sugere
             * aguardar charge-pump/saídas antes de aplicar STEP). Com tick=100 kHz,
             * 500 ticks ≈ 5 ms. */
            ax->warmup_ticks = (ax->total_steps > 0u) ? 500u : 0u;
                motion_hw_step_low(axis);
                motion_hw_set_dir(axis,
                                (uint8_t) ((seg->dirMask >> axis) & 0x1u));
                if (total > 0u)
                        motion_hw_enable(axis, 1u);
                else
                        motion_hw_enable(axis, 0u);
		g_encoder_origin[axis] = g_encoder_position[axis];
	}
}
static uint8_t motion_try_start_next_locked(void) {
	move_queue_add_req_t next;
	if (!motion_queue_pop_locked(&next))
		return 0u;
	motion_begin_segment_locked(&next);
	return 1u;
}
static void motion_update_encoders(void) {
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
                uint32_t now = motion_hw_encoder_read_raw(axis);
                uint8_t bits = motion_hw_encoder_bits(axis);
                if (bits == 16u) {
                        uint16_t prev = (uint16_t) g_encoder_last_raw[axis];
                        int16_t delta = (int16_t) ((uint16_t) now - prev);
                        g_encoder_last_raw[axis] = (uint16_t) now;
                        g_encoder_position[axis] += delta;
                } else {
                        int32_t delta = (int32_t) (now - g_encoder_last_raw[axis]);
                        g_encoder_last_raw[axis] = now;
                        g_encoder_position[axis] += delta;
                }
        }
}
static void motion_send_queue_add_ack(uint8_t frame_id, uint8_t status) {
	uint8_t raw[6];
	move_queue_add_ack_resp_t resp = { frame_id, status };
	if (move_queue_add_ack_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) {
		LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "ack", "encode_fail");
		return;
	}
	if (app_resp_push(raw, (uint32_t) sizeof raw) != PROTO_OK) {
		LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "ack", "queue_full");
	}
}
static void motion_send_queue_status_response(uint8_t frame_id) {
	uint8_t raw[12];
	move_queue_status_resp_t resp = { .frameId = frame_id, .status =
			(uint8_t) g_status.state, .pidErrX = (uint8_t) g_status.pidErrX,
			.pidErrY = (uint8_t) g_status.pidErrY, .pidErrZ =
					(uint8_t) g_status.pidErrZ, .pctX = g_status.pctX, .pctY =
					g_status.pctY, .pctZ = g_status.pctZ, };
	if (move_queue_status_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) {
		LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "status", "encode_fail");
		return;
	}
	if (app_resp_push(raw, (uint32_t) sizeof raw) != PROTO_OK) {
		LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "status", "queue_full");
	}
}
static void motion_send_start_response(uint8_t frame_id) {
	uint8_t raw[4];
	start_move_resp_t resp = { frame_id };
	if (start_move_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK)
		return;
	if (app_resp_push(raw, (uint32_t) sizeof raw) != PROTO_OK) {
		LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "start", "resp_queue_full");
	}
}
static void motion_send_move_end_response(uint8_t frame_id) {
	uint8_t raw[4];
	move_end_resp_t resp = { frame_id };
	if (move_end_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK)
		return;
	if (app_resp_push(raw, (uint32_t) sizeof raw) != PROTO_OK) {
		LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "move_end",
				"resp_queue_full");
	}
}
void motion_service_init(void) {
	uint32_t primask = motion_lock();
	memset(&g_status, 0, sizeof g_status);
	memset(g_axis_state, 0, sizeof g_axis_state);
	memset(g_queue, 0, sizeof g_queue);
	memset(g_encoder_position, 0, sizeof g_encoder_position);
	memset(g_encoder_last_raw, 0, sizeof g_encoder_last_raw);
	memset(g_encoder_origin, 0, sizeof g_encoder_origin);
	/* Define velocidade inicial do demo para o menor valor da tabela */
	g_demo_speed_idx = 0u;
	g_status.state = MOTION_IDLE;
	g_queue_head = g_queue_tail = g_queue_count = 0u;
	g_has_active_segment = 0u;
        motion_stop_all_axes_locked();
        motion_refresh_status_locked();
        motion_unlock(primask);
        motion_hw_init();
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
                uint32_t raw = motion_hw_encoder_read_raw(axis);
                if (motion_hw_encoder_bits(axis) == 16u) {
                        g_encoder_last_raw[axis] = raw & 0xFFFFu;
                } else {
                        g_encoder_last_raw[axis] = raw;
                }
        }
        // Início regular dos timers: contadores zerados e evento de atualização
        __HAL_TIM_DISABLE(&htim6);
        __HAL_TIM_DISABLE(&htim7);
        __HAL_TIM_SET_COUNTER(&htim6, 0);
        __HAL_TIM_SET_COUNTER(&htim7, 0);
        __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
        __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);
        HAL_TIM_GenerateEvent(&htim6, TIM_EVENTSOURCE_UPDATE);
        HAL_TIM_GenerateEvent(&htim7, TIM_EVENTSOURCE_UPDATE);
        if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
                Error_Handler();
	if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK)
		Error_Handler();
	LOGT_THIS(LOG_STATE_START, PROTO_OK, "init", "timers_ready");
}
const motion_status_t* motion_status_get(void) {
	return &g_status;
}
void motion_on_tim6_tick(void) {
        if (g_status.state != MOTION_RUNNING || !g_has_active_segment)
                return;
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
                motion_axis_state_t *ax = &g_axis_state[axis];
                if (ax->warmup_ticks > 0u) {
                        --ax->warmup_ticks;
                        continue;
                }
                if (ax->step_high) {
                        motion_hw_step_low(axis);
                        ax->step_high = 0u;
                        continue;
                }
                if (ax->emitted_steps >= ax->total_steps)
                        continue;
                if (ax->emitted_steps < ax->target_steps) {
                        motion_hw_step_high(axis);
                        ax->step_high = 1u;
                        ++ax->emitted_steps;
                }
        }
	uint8_t finished = 1u;
	for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
		const motion_axis_state_t *ax = &g_axis_state[axis];
		if (ax->emitted_steps < ax->total_steps || ax->step_high) {
			finished = 0u;
			break;
		}
	}
	if (!finished)
		return;
	uint32_t primask = motion_lock();
	if (g_has_active_segment) {
		uint8_t confirm = 1u;
		for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
			const motion_axis_state_t *ax = &g_axis_state[axis];
			if (ax->emitted_steps < ax->total_steps || ax->step_high) {
				confirm = 0u;
				break;
			}
		}
		if (confirm) {
			if (motion_try_start_next_locked()) {
				g_status.state = MOTION_RUNNING;
			} else {
				g_has_active_segment = 0u;
				motion_stop_all_axes_locked();
				g_status.state = MOTION_DONE;
			}
			motion_refresh_status_locked();
		}
	}
	motion_unlock(primask);
}
void motion_on_tim7_tick(void) {
	motion_update_encoders();
	if (g_status.state == MOTION_RUNNING && g_has_active_segment) {
		for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
			motion_axis_state_t *ax = &g_axis_state[axis];
			if (ax->emitted_steps >= ax->total_steps)
				continue;
			uint32_t total = ax->total_steps;
			uint32_t target = ax->target_steps;
			uint32_t velocity = ax->velocity_per_tick;
			if (velocity == 0u) {
				target = total;
			} else {
				uint64_t next = (uint64_t) target + (uint64_t) velocity;
				if (next > total)
					next = total;
				target = (uint32_t) next;
			}
			ax->target_steps = target;
		}
	}
	uint32_t primask = motion_lock();
	motion_refresh_status_locked();
	motion_unlock(primask);
}
void motion_on_move_queue_add(const uint8_t *frame, uint32_t len) {
	move_queue_add_req_t req;
	uint8_t ack_status = MOTION_ACK_INVALID;
	uint8_t frame_id = 0u;
    if (!frame)
        return;
    proto_result_t decode_status = move_queue_add_req_decoder(frame, len, &req);
    if (decode_status != PROTO_OK) {
        motion_send_queue_add_ack(frame_id, ack_status);
        LOGA_THIS(LOG_STATE_ERROR, decode_status, "queue_add", "decode_fail");
        return;
    }
    frame_id = req.frameId;
    /* Bloqueia enfileiramento em condição de E-STOP */
    if (!safety_is_safe()) {
        motion_send_queue_add_ack(frame_id, ack_status);
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "queue_add", "blocked_safety");
        return;
    }
	uint32_t primask = motion_lock();
	proto_result_t push_status = motion_queue_push_locked(&req);
	if (push_status == PROTO_OK) {
		ack_status = MOTION_ACK_OK;
		if (g_status.state == MOTION_IDLE || g_status.state == MOTION_DONE)
			g_status.state = MOTION_QUEUED;
		motion_refresh_status_locked();
	} else {
		ack_status = MOTION_ACK_QUEUE_FULL;
	}
	motion_unlock(primask);
	motion_send_queue_add_ack(frame_id, ack_status);
	LOGA_THIS(LOG_STATE_RECEIVED, ack_status, "queue_add",
			"frame=%u dirMask=0x%02X queue=%u", (unsigned )frame_id,
			(unsigned )req.dirMask, (unsigned )g_status.queue_depth);
}
void motion_on_move_queue_status(const uint8_t *frame, uint32_t len) {
	move_queue_status_req_t req;
	if (move_queue_status_req_decoder(frame, len, &req) != PROTO_OK) {
		LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "queue_status",
				"decode_fail");
		return;
	}
	uint32_t primask = motion_lock();
	motion_refresh_status_locked();
	motion_unlock(primask);
	motion_send_queue_status_response(req.frameId);
	LOGA_THIS(LOG_STATE_RECEIVED, PROTO_OK, "queue_status",
			"state=%u depth=%u pct=(%u,%u,%u)", (unsigned )g_status.state,
			(unsigned )g_status.queue_depth, (unsigned )g_status.pctX,
			(unsigned )g_status.pctY, (unsigned )g_status.pctZ);
}
void motion_on_start_move(const uint8_t *frame, uint32_t len) {
	start_move_req_t req;
	if (start_move_req_decoder(frame, len, &req) != PROTO_OK) {
		LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "start_move",
				"decode_fail");
		return;
	}
	uint8_t started = 0u;
	uint32_t primask = motion_lock();
	if (!safety_is_safe()) {
		/* Segurança não liberada: ignora início */
		started = 0u;
	} else if (!g_has_active_segment) {
		if (motion_try_start_next_locked()) {
			g_status.state = MOTION_RUNNING;
			started = 1u;
		}
	} else {
		g_status.state = MOTION_RUNNING;
		started = 1u;
	}
	motion_refresh_status_locked();
	motion_unlock(primask);
	motion_send_start_response(req.frameId);
	LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "start_move",
			started ? "running" : "ignored");
}
void motion_on_move_end(const uint8_t *frame, uint32_t len) {
	move_end_req_t req;
	if (move_end_req_decoder(frame, len, &req) != PROTO_OK) {
		LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "move_end", "decode_fail");
		return;
	}
	uint32_t primask = motion_lock();
	motion_stop_all_axes_locked();
	motion_queue_clear_locked();
	g_has_active_segment = 0u;
	g_status.state = MOTION_STOPPING;
	motion_refresh_status_locked();
	motion_unlock(primask);
	motion_send_move_end_response(req.frameId);
	primask = motion_lock();
	g_status.state = MOTION_IDLE;
	motion_refresh_status_locked();
	motion_unlock(primask);
	LOGT_THIS(LOG_STATE_APPLIED, PROTO_OK, "move_end", "stopped");
}

/* =====================================================================
 *  DEMO DE MOVIMENTO (PARA TESTE EM BANCADA)
 *
 *  Objetivo: validar rapidamente o cabeamento/driver dos motores sem
 *  precisar preparar requisições do host. Mantém o código do serviço
 *  pronto para evoluir com comandos "de verdade" (queue-add/start-move)
 *  mas permite acionar um gerador de passos contínuos durante bring-up.
 *
 *  Uso (uma das opções abaixo):
 *    1) Ativar modo contínuo após o app_init (STM32), por exemplo:
 *         - Em Core/Src/main.c, após app_init():
 *             extern void motion_demo_set_continuous(uint8_t);
 *             motion_demo_set_continuous(1); // Liga
 *         - Quando quiser parar:
 *             motion_demo_set_continuous(0);
 *
 *    2) Injetar um segmento curto único (não contínuo):
 *         extern void motion_demo_set_enabled(uint8_t);
 *         motion_demo_set_enabled(1);
 *
 *  Observações:
 *   - Largura do pulso STEP = 1 período do TIM6. Ajuste PSC/ARR do TIM6
 *     conforme o tempo mínimo de pulso exigido pelo driver.
 *   - Taxa de incremento do alvo por eixo = velocity_per_tick em TIM7
 *     (≈ passos por ms se TIM7 ≈ 1 kHz). Ex.: 10 → ~10 k steps/s.
 *   - O modo contínuo ignora a fila e programa um segmento "virtual"
 *     com total_steps muito grande; use apenas para teste.
 * ===================================================================== */

// Enfileira um único segmento de teste e inicia execução (não contínuo)
void motion_demo_set_enabled(uint8_t enable)
{
    if (!enable) return;

    uint32_t primask = motion_lock();
    if (g_has_active_segment || g_queue_count > 0u) {
        motion_unlock(primask);
        return;
    }

    // Segmento de teste: XYZ para frente, alvos moderados
    move_queue_add_req_t req = {0};
    req.frameId = 0xEE;
    req.dirMask = 0x07;        // XYZ forward
    req.vx = 10; req.vy = 8; req.vz = 6; // ≈ 10k/8k/6k steps/s
    req.sx = 2000; req.sy = 1600; req.sz = 1200; // passos totais

    // Empilha na fila, se houver espaço
    if (g_queue_count < MOTION_QUEUE_CAPACITY) {
        g_queue[g_queue_tail].req = req;
        g_queue_tail = (uint8_t)((g_queue_tail + 1u) % MOTION_QUEUE_CAPACITY);
        g_queue_count++;
    }

    // Inicia imediatamente se possível
    if (!g_has_active_segment) {
        move_queue_add_req_t next;
        if (motion_queue_pop_locked(&next)) {
            motion_begin_segment_locked(&next);
            g_status.state = MOTION_RUNNING;
            motion_refresh_status_locked();
        }
    }
    motion_unlock(primask);
}

// Liga/desliga gerador contínuo de passos (ignora a fila)
void motion_demo_set_continuous(uint8_t enable)
{
    uint32_t primask = motion_lock();
    g_demo_continuous = (enable ? 1u : 0u);

    if (g_demo_continuous) {
        // Programa um "segmento" muito longo e velocidade fixa por eixo
        g_has_active_segment = 1u;
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            motion_axis_state_t *ax = &g_axis_state[axis];
            ax->total_steps = 0xFFFFFFFFu; // efetivamente contínuo
            ax->target_steps = 0u;
            ax->emitted_steps = 0u;
            ax->velocity_per_tick = (uint16_t)(g_demo_speed_table[g_demo_speed_idx & 0x3u] / 5u);
            ax->kp = ax->ki = ax->kd = 0u;
            ax->step_high = 0u;
            motion_hw_step_low(axis);
            motion_hw_set_dir(axis, 1u);   // forward
            motion_hw_enable(axis, 1u);     // liga driver (ativo em baixo)
            g_encoder_origin[axis] = g_encoder_position[axis];
        }
        g_status.state = MOTION_RUNNING;
        motion_refresh_status_locked();
    } else {
        // Para tudo e retorna a IDLE
        motion_stop_all_axes_locked();
        motion_queue_clear_locked();
        g_has_active_segment = 0u;
        g_status.state = MOTION_IDLE;
        motion_refresh_status_locked();
    }
    motion_unlock(primask);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (!htim)
		return;
	if (htim->Instance == TIM6) {
		motion_on_tim6_tick();
	} else if (htim->Instance == TIM7) {
		motion_on_tim7_tick();
	} else {
		(void) htim;
	}
}

/* ===== API pública auxiliar ===== */
void motion_emergency_stop(void)
{
    uint32_t primask = motion_lock();
    /* Desliga demo e qualquer segmento em andamento */
    g_demo_continuous = 0u;
    motion_stop_all_axes_locked();
    motion_queue_clear_locked();
    g_has_active_segment = 0u;
    /* Transição de estado segura */
    g_status.state = MOTION_STOPPING;
    motion_refresh_status_locked();
    motion_unlock(primask);

    /* Retorna a IDLE logo em seguida para refletir repouso */
    primask = motion_lock();
    g_status.state = MOTION_IDLE;
    motion_refresh_status_locked();
    motion_unlock(primask);
}

uint8_t motion_demo_is_active(void)
{
    return g_demo_continuous ? 1u : 0u;
}

void motion_demo_cycle_speed(void)
{
    /* Avança índice (0..3) */
    g_demo_speed_idx = (uint8_t)((g_demo_speed_idx + 1u) & 0x3u);
    if (!g_demo_continuous)
        return;
    /* Aplica nova velocidade imediatamente se demo está ativo */
    uint16_t v = (uint16_t)(g_demo_speed_table[g_demo_speed_idx & 0x3u] / 5u);
    uint32_t primask = motion_lock();
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        g_axis_state[axis].velocity_per_tick = v;
    }
    motion_unlock(primask);
}
