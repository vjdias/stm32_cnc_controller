#include "Services/Motion/motion_service.h"
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
#include "app.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

LOG_SVC_DEFINE(LOG_SVC_MOTION, "motion");
#define MOTION_AXIS_COUNT 3u
#define MOTION_QUEUE_CAPACITY 8u
typedef struct {
	GPIO_TypeDef *step_port;
	uint16_t step_pin;
	GPIO_TypeDef *dir_port;
	uint16_t dir_pin;
	GPIO_TypeDef *ena_port;
	uint16_t ena_pin;
	TIM_HandleTypeDef *encoder;
	uint8_t counter_bits;
} motion_axis_hw_t;
typedef struct {
	uint32_t total_steps;
	uint32_t target_steps;
	uint32_t emitted_steps;
	uint16_t velocity_per_tick;
	uint16_t kp, ki, kd;
	uint8_t step_high;
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
static const motion_axis_hw_t g_axis_hw[MOTION_AXIS_COUNT] =
		{
				{ GPIOB, GPIO_PIN_4, GPIOA, GPIO_PIN_3, GPIOC, GPIO_PIN_4,
						&htim2, 32u }, { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_2,
						GPIOC, GPIO_PIN_5, &htim5, 32u }, { GPIOB, GPIO_PIN_1,
						GPIOA, GPIO_PIN_2, GPIOA, GPIO_PIN_8, &htim3, 16u }, };
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
// Demo flags
static volatile uint8_t g_demo_continuous = 0u;
static inline void gpio_set_high(GPIO_TypeDef *port, uint16_t pin) {
	if (!port)
		return;
	port->BSRR = pin;
}
static inline void gpio_set_low(GPIO_TypeDef *port, uint16_t pin) {
	if (!port)
		return;
	port->BSRR = ((uint32_t) pin) << 16u;
}
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
static void motion_hw_set_direction(uint8_t axis, uint8_t dir) {
	const motion_axis_hw_t *hw = &g_axis_hw[axis];
	if (dir)
		gpio_set_high(hw->dir_port, hw->dir_pin);
	else
		gpio_set_low(hw->dir_port, hw->dir_pin);
}
static void motion_hw_enable_axis(uint8_t axis, uint8_t enable) {
	const motion_axis_hw_t *hw = &g_axis_hw[axis];
	if (enable)
		gpio_set_low(hw->ena_port, hw->ena_pin);
	else
		gpio_set_high(hw->ena_port, hw->ena_pin);
}
static void motion_hw_reset_step(uint8_t axis) {
	const motion_axis_hw_t *hw = &g_axis_hw[axis];
	gpio_set_low(hw->step_port, hw->step_pin);
}
static void motion_stop_all_axes_locked(void) {
	for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
		motion_hw_reset_step(axis);
		motion_hw_enable_axis(axis, 0u);
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
		motion_hw_reset_step(axis);
		motion_hw_set_direction(axis,
				(uint8_t) ((seg->dirMask >> axis) & 0x1u));
		if (total > 0u)
			motion_hw_enable_axis(axis, 1u);
		else
			motion_hw_enable_axis(axis, 0u);
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
	uint32_t now_x = __HAL_TIM_GET_COUNTER(g_axis_hw[AXIS_X].encoder);
	int32_t delta_x = (int32_t) (now_x - g_encoder_last_raw[AXIS_X]);
	g_encoder_last_raw[AXIS_X] = now_x;
	g_encoder_position[AXIS_X] += delta_x;
	uint32_t now_y = __HAL_TIM_GET_COUNTER(g_axis_hw[AXIS_Y].encoder);
	int32_t delta_y = (int32_t) (now_y - g_encoder_last_raw[AXIS_Y]);
	g_encoder_last_raw[AXIS_Y] = now_y;
	g_encoder_position[AXIS_Y] += delta_y;
	uint32_t now_z = __HAL_TIM_GET_COUNTER(g_axis_hw[AXIS_Z].encoder) & 0xFFFFu;
	int16_t delta_z = (int16_t) ((uint16_t) now_z
			- (uint16_t) g_encoder_last_raw[AXIS_Z]);
	g_encoder_last_raw[AXIS_Z] = now_z & 0xFFFFu;
	g_encoder_position[AXIS_Z] += delta_z;
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
	g_status.state = MOTION_IDLE;
	g_queue_head = g_queue_tail = g_queue_count = 0u;
	g_has_active_segment = 0u;
	motion_stop_all_axes_locked();
	motion_refresh_status_locked();
	motion_unlock(primask);
	__HAL_TIM_SET_COUNTER(&htim2, 0u);
	__HAL_TIM_SET_COUNTER(&htim5, 0u);
	__HAL_TIM_SET_COUNTER(&htim3, 0u);
	if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL) != HAL_OK)
		Error_Handler();
	if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
		Error_Handler();
	g_encoder_last_raw[AXIS_X] = __HAL_TIM_GET_COUNTER(&htim2);
	g_encoder_last_raw[AXIS_Y] = __HAL_TIM_GET_COUNTER(&htim5);
	g_encoder_last_raw[AXIS_Z] = __HAL_TIM_GET_COUNTER(&htim3) & 0xFFFFu;
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
		const motion_axis_hw_t *hw = &g_axis_hw[axis];
		if (ax->step_high) {
			gpio_set_low(hw->step_port, hw->step_pin);
			ax->step_high = 0u;
			continue;
		}
		if (ax->emitted_steps >= ax->total_steps)
			continue;
		if (ax->emitted_steps < ax->target_steps) {
			gpio_set_high(hw->step_port, hw->step_pin);
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
	if (!g_has_active_segment) {
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

// ===============================================
//  Demo helpers (para teste em bancada)
// ===============================================

// Enfileira um segmento simples e inicia execução se possível
void motion_demo_set_enabled(uint8_t enable)
{
    if (!enable) return;
    uint32_t primask = motion_lock();
    if (g_has_active_segment || g_queue_count > 0u) {
        motion_unlock(primask);
        return;
    }

    move_queue_add_req_t req = {0};
    req.frameId = 0xEE;
    req.dirMask = 0x07; // XYZ forward
    req.vx = 10; req.vy = 8; req.vz = 6; // passos por tick (TIM7)
    req.sx = 2000; req.sy = 1600; req.sz = 1200; // passos totais
    req.kp_x = req.kp_y = req.kp_z = 0;
    req.ki_x = req.ki_y = req.ki_z = 0;
    req.kd_x = req.kd_y = req.kd_z = 0;

    // Empilha na fila
    if (g_queue_count < MOTION_QUEUE_CAPACITY) {
        g_queue[g_queue_tail].req = req;
        g_queue_tail = (uint8_t)((g_queue_tail + 1u) % MOTION_QUEUE_CAPACITY);
        g_queue_count++;
    }

    // Inicia imediatamente
    if (!g_has_active_segment) {
        move_queue_add_req_t next;
        (void)motion_queue_pop_locked(&next);
        motion_begin_segment_locked(&next);
        g_status.state = MOTION_RUNNING;
        motion_refresh_status_locked();
    }
    motion_unlock(primask);
}

// Liga/desliga movimento contínuo (gera passos sem parar)
void motion_demo_set_continuous(uint8_t enable)
{
    uint32_t primask = motion_lock();
    g_demo_continuous = (enable ? 1u : 0u);
    if (g_demo_continuous) {
        // Programa um segmento "infinito"
        g_has_active_segment = 1u;
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            motion_axis_state_t *ax = &g_axis_state[axis];
            ax->total_steps = 0xFFFFFFFFu;
            ax->target_steps = 0u;
            ax->emitted_steps = 0u;
            ax->velocity_per_tick = 10; // ~10k steps/s com TIM7=1kHz
            ax->kp = ax->ki = ax->kd = 0u;
            ax->step_high = 0u;
            motion_hw_reset_step(axis);
            motion_hw_set_direction(axis, 1u);
            motion_hw_enable_axis(axis, 1u);
            g_encoder_origin[axis] = g_encoder_position[axis];
        }
        g_status.state = MOTION_RUNNING;
        motion_refresh_status_locked();
    } else {
        // Para e volta para IDLE
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
