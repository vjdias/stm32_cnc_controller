#include "app.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

#include "Services/Log/log_service.h"
#include "Services/Motion/motion_service.h"
#include "Services/Motion/motion_hw.h"
#include "Services/Safety/safety_service.h"

#include "Protocol/Requests/move_queue_add_request.h"
#include "Protocol/Responses/move_queue_add_ack_response.h"
#include "Protocol/Requests/move_queue_status_request.h"
#include "Protocol/Responses/move_queue_status_response.h"
#include "Protocol/Requests/start_move_request.h"
#include "Protocol/Responses/start_move_response.h"
#include "Protocol/Requests/move_end_request.h"
#include "Protocol/Responses/move_end_response.h"
// Novos: set_origin e encoder_status
#include "Protocol/Requests/set_origin_request.h"
#include "Protocol/Responses/set_origin_response.h"
#include "Protocol/Requests/encoder_status_request.h"
#include "Protocol/Responses/encoder_status_response.h"
#include "Protocol/Requests/set_microsteps_request.h"

LOG_SVC_DEFINE(LOG_SVC_MOTION, "motion");


#define MOTION_AXIS_COUNT        3u
#define MOTION_QUEUE_CAPACITY    64u

/* =======================
 *  Timings p/ TMC5160 + DDA
 *  (ajuste MOTION_TIM6_HZ se mudar TIM6)
 * ======================= */
#define MOTION_TIM6_HZ                 50000u   /* 50 kHz -> 20 us por tick */
#define MOTION_STEP_HIGH_TICKS         1u       /* largura do STEP: >= 1 tick (>=20us) */
#define MOTION_DIR_SETUP_TICKS         1u       
#define MOTION_ENABLE_SETTLE_TICKS     2u       

#define MOTION_STEP_LOW_TICKS          1u

#define Q16_1                          (1u<<16)
#define Q16_FROM_UINT(x)               ((uint32_t)(x) << 16)
#define Q16_DIV_UINT(numer,den)        ((uint32_t)(((uint64_t)(numer) << 16) / (uint32_t)(den)))
#ifndef MOTION_DEBUG_ENCODERS
#define MOTION_DEBUG_ENCODERS          1
#endif
#ifndef MOTION_DEBUG_FLOW
#define MOTION_DEBUG_FLOW              1
#endif
#ifndef MOTION_DEBUG_TIM6_PRINTS
#define MOTION_DEBUG_TIM6_PRINTS       0
#endif
#ifndef MOTION_DEBUG_STEP_DECIM
#define MOTION_DEBUG_STEP_DECIM        500u
#endif

#define DEMO_ACCEL_SPS2                200000u  /* ~50 ms p/ ir a 10k sps */

/* Limite físico de passos/s com largura mínima de STEP e tLOW implícito */
#define MOTION_MIN_LOW_TICKS   ( (MOTION_STEP_LOW_TICKS) ? (MOTION_STEP_LOW_TICKS) : 1u )
#define MOTION_MAX_SPS         ( (MOTION_TIM6_HZ) / ( (MOTION_STEP_HIGH_TICKS) + (MOTION_MIN_LOW_TICKS) ) )

/* =======================
 *  Tipos/estados
 * ======================= */
typedef struct {
    uint32_t total_steps;
    uint32_t target_steps;        /* caminho fila/protocolo preservado */
    uint32_t emitted_steps;

    uint16_t velocity_per_tick;   /* ~ steps/ms (se TIM7 ~1kHz). 10 => ~10k steps/s */

    /* Ganhos PID (mantidos) */
    uint16_t kp, ki, kd;

    uint8_t  step_high;           /* >0 = quantos ticks faltam em ALTO */
    uint8_t  step_low;            /* >0 = aguarda ticks em BAIXO (tLOW) */

    /* --- DDA + rampa --- */
    uint32_t dda_accum_q16;       /* acumulador de fase Q16.16 */
    uint32_t dda_inc_q16;         /* incremento por tick do TIM6 (Q16.16) */
    uint32_t v_target_sps;        /* steps/s alvo */
    uint32_t v_actual_sps;        /* steps/s efetivo com rampa */
    uint32_t accel_sps2;          /* steps/s^2 */

    uint8_t  en_settle_ticks;
        uint8_t  dir_settle_ticks;
} motion_axis_state_t;

typedef struct {
    move_queue_add_req_t req;
} motion_queue_entry_t;

enum { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };
enum { MOTION_ACK_OK = 0, MOTION_ACK_INVALID = 1, MOTION_ACK_QUEUE_FULL = 2 };

/* =======================
 *  Estado global
 * ======================= */
static motion_status_t g_status;
static motion_axis_state_t g_axis_state[MOTION_AXIS_COUNT];
static volatile uint8_t g_has_active_segment = 0u;

static motion_queue_entry_t g_queue[MOTION_QUEUE_CAPACITY];
static uint8_t g_queue_head = 0u;
static uint8_t g_queue_tail = 0u;
static uint8_t g_queue_count = 0u;
static uint8_t g_active_frame_id = 0u; 

static int64_t  g_encoder_position[MOTION_AXIS_COUNT];
static uint32_t g_encoder_last_raw[MOTION_AXIS_COUNT];
static int64_t  g_encoder_origin[MOTION_AXIS_COUNT];
static int32_t  g_encoder_delta_tick[MOTION_AXIS_COUNT]; /* delta de contagens por tick TIM7 */
static int32_t  g_origin_base32[MOTION_AXIS_COUNT]; /* offset externo (origin-set) */

/* PI de velocidade baseado no encoder
 * Notas:
 *  - Ganhos kp/ki/kd recebidos via protocolo são inteiros de 16 bits.
 *  - A saída da correção é escalada por 2^-8 (>> 8) para manter estabilidade.
 */
#define MOTION_PI_SHIFT              8
#define MOTION_PI_I_CLAMP            (200000)
#define MOTION_PI_CORR_MAX_SPS       (MOTION_MAX_SPS)
// Controle PI desativado por padrão (somente telemetria)
//#ifndef MOTION_PI_ENABLE
#define MOTION_PI_ENABLE 1
//#endif

/* =======================
 *  Calibração simples (sem floats)
 * ======================= */
/*
 * Passo do motor: 0,9° => 400 passos/rotação (STEPS_PER_REV_BASE)
 * Ajuste MICROSTEP_FACTOR se o TMC estiver configurado com microstepping
 * (ex.: 16 => 6400 passos/rotação no DDA).
 */
//#ifndef MICROSTEP_FACTOR
#define MICROSTEP_FACTOR 256
//#endif
#define STEPS_PER_REV_BASE   400u
#define DDA_STEPS_PER_REV    (STEPS_PER_REV_BASE * MICROSTEP_FACTOR)
/* Encoders por rotação (fornecido): X/Z = 40000, Y = 2500 */
static const uint32_t ENC_COUNTS_PER_REV[3] = { 40000u, 2500u, 40000u}; // X,Y,Z 
static volatile uint16_t g_microstep_factor = MICROSTEP_FACTOR;
static inline uint32_t dda_steps_per_rev(void) { return STEPS_PER_REV_BASE * (uint32_t)g_microstep_factor; }

/* Deadband em passos para o PI de posição (evita tremor próximo de zero) */
#ifndef MOTION_PI_DEADBAND_STEPS
#define MOTION_PI_DEADBAND_STEPS 10
#endif

/* Derivada filtrada (para reduzir ruído do encoder no D-term) */
static int32_t g_pi_d_filt[MOTION_AXIS_COUNT];
/* Acumulador de aceleração (converter a_sps2/1000 com maior precisão) */
static uint32_t g_v_accum[MOTION_AXIS_COUNT];
static int32_t g_pi_i_accum[MOTION_AXIS_COUNT];
static int32_t g_pi_prev_err[MOTION_AXIS_COUNT];
// Sombras 32-bit para uso com SWV Data Trace/Graph
volatile int32_t g_enc_abs32[MOTION_AXIS_COUNT];
volatile int32_t g_enc_rel32[MOTION_AXIS_COUNT];

static uint32_t g_step_print_count[MOTION_AXIS_COUNT] = {0,0,0};

static volatile uint8_t g_demo_continuous = 0u; /* 1 = gera passos continuamente (modo DEMO) */
static const uint16_t  g_demo_speed_table[4] = { 5u, 10u, 20u, 40u }; /* ksteps/s aprox (TIM7 ~1kHz) */
static volatile uint8_t g_demo_speed_idx = 1u;

/* =======================
 *  Helpers de lock
 * ======================= */
static inline uint32_t motion_lock(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}
static inline void motion_unlock(uint32_t primask) {
    __set_PRIMASK(primask);
}

/* =======================
 *  Helpers de acesso por eixo
 * ======================= */
static inline uint32_t motion_total_for_axis(const move_queue_add_req_t *req, uint8_t axis) {
    switch (axis) {
        case AXIS_X: return req->sx;
        case AXIS_Y: return req->sy;
        case AXIS_Z:
        default:     return req->sz;
    }
}

static inline uint16_t motion_velocity_for_axis(const move_queue_add_req_t *req, uint8_t axis) {
    switch (axis) {
        case AXIS_X: return req->vx;
        case AXIS_Y: return req->vy;
        case AXIS_Z:
        default:     return req->vz;
    }
}

static inline uint16_t motion_kp_for_axis(const move_queue_add_req_t *req, uint8_t axis) {
    switch (axis) {
        case AXIS_X: return req->kp_x;
        case AXIS_Y: return req->kp_y;
        case AXIS_Z:
        default:     return req->kp_z;
    }
}

static inline uint16_t motion_ki_for_axis(const move_queue_add_req_t *req, uint8_t axis) {
    switch (axis) {
        case AXIS_X: return req->ki_x;
        case AXIS_Y: return req->ki_y;
        case AXIS_Z:
        default:     return req->ki_z;
    }
}

static inline uint16_t motion_kd_for_axis(const move_queue_add_req_t *req, uint8_t axis) {
    switch (axis) {
        case AXIS_X: return req->kd_x;
        case AXIS_Y: return req->kd_y;
        case AXIS_Z:
        default:     return req->kd_z;
    }
}

static inline int8_t motion_clamp_error(int32_t value) {
    if (value > 127)  return 127;
    if (value < -128) return -128;
    return (int8_t)value;
}

/* Soma restante (em passos) no eixo, incluindo segmento ativo + fila
 * Usado para decidir desaceleração suave no final da lista de movimentos. */
static uint32_t motion_remaining_steps_total_for_axis(uint8_t axis)
{
    uint32_t rem = 0u;
    if (axis < MOTION_AXIS_COUNT) {
        const motion_axis_state_t *ax = &g_axis_state[axis];
        if (ax->total_steps > ax->emitted_steps)
            rem += (ax->total_steps - ax->emitted_steps);
    }
    for (uint8_t i = 0; i < g_queue_count; ++i) {
        uint8_t idxq = (uint8_t)((g_queue_head + i) % MOTION_QUEUE_CAPACITY);
        const move_queue_add_req_t *q = &g_queue[idxq].req;
        rem += motion_total_for_axis(q, axis);
    }
    return rem;
}

/* =======================
 *  Status e fila
 * ======================= */
static void motion_refresh_status_locked(void) {
    g_status.queue_depth = (uint8_t)(g_queue_count + (g_has_active_segment ? 1u : 0u));

    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        const motion_axis_state_t *ax = &g_axis_state[axis];
        uint32_t total = ax->total_steps;
        uint32_t emitted = ax->emitted_steps;
        uint8_t pct = 0u;

        if (g_has_active_segment && total > 0u) {
            uint64_t scaled = (uint64_t)emitted * 100u;
            pct = (uint8_t)(scaled / total);
            if (pct > 100u) pct = 100u;
        } else if (total == 0u && g_has_active_segment) {
            pct = 100u;
        } else if (!g_has_active_segment && total > 0u) {
            pct = (emitted >= total) ? 100u : (uint8_t)(((uint64_t)emitted * 100u) / total);
        }

        /* Erro em UNIDADES DE PASSOS (alinhado ao PI de posição)
         * desired_steps = passos emitidos no segmento (target_steps)
         * actual_steps  = encoder_rel convertido para passos DDA
         */
        int64_t enc_rel = g_encoder_position[axis] - g_encoder_origin[axis];
        if (enc_rel > (int64_t)INT32_MAX) enc_rel = INT32_MAX;
        else if (enc_rel < (int64_t)INT32_MIN) enc_rel = INT32_MIN;
        int64_t num = enc_rel * (int64_t)dda_steps_per_rev();
        int32_t actual_steps = (ENC_COUNTS_PER_REV[axis]
                                ? (int32_t)(num / (int64_t)ENC_COUNTS_PER_REV[axis])
                                : 0);
        int32_t desired_steps = (int32_t)ax->target_steps;
        int32_t err = desired_steps - actual_steps;
        int8_t  err8 = motion_clamp_error(err);

        switch (axis) {
            case AXIS_X: g_status.pctX = pct; g_status.pidErrX = err8; break;
            case AXIS_Y: g_status.pctY = pct; g_status.pidErrY = err8; break;
            case AXIS_Z:
            default:     g_status.pctZ = pct; g_status.pidErrZ = err8; break;
        }
    }
}


static void motion_stop_all_axes_locked(void) {
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        motion_hw_step_low(axis);
        motion_hw_enable(axis, 0u);

        motion_axis_state_t *ax = &g_axis_state[axis];
        ax->total_steps       = 0u;
        ax->target_steps      = 0u;
        ax->emitted_steps     = 0u;

        ax->velocity_per_tick = 0u;
        ax->kp = 0u; ax->ki = 0u; ax->kd = 0u;

        /* limpa controle de pulso/guardas */
        ax->step_high         = 0u;
        ax->step_low          = 0u;
        ax->en_settle_ticks   = 0u;
        ax->dir_settle_ticks  = 0u;

        /* limpa DDA/rampa */
        ax->dda_accum_q16     = 0u;
        ax->dda_inc_q16       = 0u;
        ax->v_target_sps      = 0u;
        ax->v_actual_sps      = 0u;
        ax->accel_sps2        = 0u;
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
    g_queue_tail = (uint8_t)((g_queue_tail + 1u) % MOTION_QUEUE_CAPACITY);
    ++g_queue_count;
    motion_refresh_status_locked();
    return PROTO_OK;
}

static int motion_queue_pop_locked(move_queue_add_req_t *out) {
    if (g_queue_count == 0u) return 0;
    if (out) *out = g_queue[g_queue_head].req;
    g_queue_head = (uint8_t)((g_queue_head + 1u) % MOTION_QUEUE_CAPACITY);
    --g_queue_count;
    return 1;
}

static void motion_begin_segment_locked(const move_queue_add_req_t *seg) {
    if (!seg) return;

    g_has_active_segment = 1u;
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        motion_axis_state_t *ax = &g_axis_state[axis];
        uint32_t total   = motion_total_for_axis(seg, axis);
        uint16_t velTick = motion_velocity_for_axis(seg, axis);

        ax->total_steps       = total;
        ax->target_steps      = 0u;
        ax->emitted_steps     = 0u;

        ax->velocity_per_tick = velTick;
        ax->kp = motion_kp_for_axis(seg, axis);
        ax->ki = motion_ki_for_axis(seg, axis);
        ax->kd = motion_kd_for_axis(seg, axis);

        /* guardas para atender DIR/ENABLE timings do TMC5160 */
        ax->step_high         = 0u;
        ax->step_low          = 0u; 
        ax->en_settle_ticks   = (total > 0u) ? MOTION_ENABLE_SETTLE_TICKS : 0u;
        ax->dir_settle_ticks  = MOTION_DIR_SETUP_TICKS;

        ax->dda_accum_q16     = 0u;
        ax->dda_inc_q16       = 0u;
        ax->v_target_sps      = ((uint32_t)velTick) * 1000u;  /* steps/s alvo (derivado do seu campo) */
        if (ax->v_target_sps > MOTION_MAX_SPS) ax->v_target_sps = MOTION_MAX_SPS;
        /* Preserva v_actual_sps ao encadear segmentos (rampa só no início da lista) */
        if (g_status.state != MOTION_RUNNING) {
            ax->v_actual_sps  = 0u;
        }
        ax->accel_sps2        = DEMO_ACCEL_SPS2;

        motion_hw_step_low(axis);
        motion_hw_set_dir(axis, (uint8_t)((seg->dirMask >> axis) & 0x1u));

        if (total > 0u) motion_hw_enable(axis, 1u);
        else            motion_hw_enable(axis, 0u);

        /* Não zera origem automaticamente; mantém a referência definida via set_origin */
        g_encoder_delta_tick[axis] = 0;
        g_pi_i_accum[axis] = 0;
        g_pi_prev_err[axis] = 0;
    }
#if MOTION_DEBUG_FLOW
    printf("[FLOW begin_segment id=%u dirMask=0x%02X V(x,y,z)=(%u,%u,%u) S(x,y,z)=(%lu,%lu,%lu) ]\r\n",
           (unsigned)seg->frameId,
           (unsigned)seg->dirMask,
           (unsigned)seg->vx, (unsigned)seg->vy, (unsigned)seg->vz,
           (unsigned long)motion_total_for_axis(seg, AXIS_X),
           (unsigned long)motion_total_for_axis(seg, AXIS_Y),
           (unsigned long)motion_total_for_axis(seg, AXIS_Z));
#endif
}

static uint8_t motion_try_start_next_locked(void) {
    move_queue_add_req_t next;
    if (!motion_queue_pop_locked(&next))
        return 0u;
    motion_begin_segment_locked(&next);
    g_active_frame_id = next.frameId;
#if MOTION_DEBUG_FLOW
    printf("[FLOW pop_next id=%u remaining=%u]\\r\\n", (unsigned)g_active_frame_id, (unsigned)g_queue_count);
#endif
    return 1u;
}

/* =======================
 *  Encoders
 * ======================= */
static void motion_update_encoders(void) {
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        uint32_t now = motion_hw_encoder_read_raw(axis);
        uint8_t bits = motion_hw_encoder_bits(axis);
        if (bits == 16u) {
            uint16_t prev = (uint16_t)g_encoder_last_raw[axis];
            int16_t delta = (int16_t)((uint16_t)now - prev);
            g_encoder_last_raw[axis] = (uint16_t)now;
            g_encoder_position[axis] += delta;
            g_encoder_delta_tick[axis] = (int32_t)delta;
#if MOTION_DEBUG_ENCODERS
            if (delta != 0) {
                long abs = (long)g_encoder_position[axis];
                long rel = (long)(g_encoder_position[axis] - g_encoder_origin[axis]);
                printf("[ENC axis=%u raw=%u delta=%d abs=%ld rel=%ld]\r\n",
                       (unsigned)axis,
                       (unsigned)now,
                       (int)delta,
                       abs,
                       rel);
            }
#endif
        } else {
            int32_t delta = (int32_t)(now - g_encoder_last_raw[axis]);
            g_encoder_last_raw[axis] = now;
            g_encoder_position[axis] += delta;
            g_encoder_delta_tick[axis] = delta;
#if MOTION_DEBUG_ENCODERS
            if (delta != 0) {
                long abs = (long)g_encoder_position[axis];
                long rel = (long)(g_encoder_position[axis] - g_encoder_origin[axis]);
                printf("[ENC axis=%u raw=%lu delta=%ld abs=%ld rel=%ld]\r\n",
                       (unsigned)axis,
                       (unsigned long)now,
                       (long)delta,
                       abs,
                       rel);
            }
#endif
        }
    }
}

/* =======================
 *  Envio de respostas
 * ======================= */
static void motion_send_queue_add_ack(uint8_t frame_id, uint8_t status) {
    uint8_t raw[6];
    move_queue_add_ack_resp_t resp = { frame_id, status };
    if (move_queue_add_ack_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "ack", "encode_fail");
        return;
    }
    if (app_resp_push(raw, (uint32_t)sizeof raw) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "ack", "queue_full");
    }
}

static void motion_send_queue_status_response(uint8_t frame_id) {
    uint8_t raw[12];
    move_queue_status_resp_t resp = {
        .frameId = frame_id,
        .status  = (uint8_t)g_status.state,
        .pidErrX = (uint8_t)g_status.pidErrX,
        .pidErrY = (uint8_t)g_status.pidErrY,
        .pidErrZ = (uint8_t)g_status.pidErrZ,
        .pctX    = g_status.pctX,
        .pctY    = g_status.pctY,
        .pctZ    = g_status.pctZ,
    };
    if (move_queue_status_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "status", "encode_fail");
        return;
    }
    if (app_resp_push(raw, (uint32_t)sizeof raw) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "status", "queue_full");
    }
}

static void motion_send_start_response(uint8_t frame_id, uint8_t status, uint8_t depth) {
    uint8_t raw[6];
    start_move_resp_t resp; resp.frameId = frame_id; resp.status = status; resp.depth = depth;
    if (start_move_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) return;
    if (app_resp_push(raw, (uint32_t)sizeof raw) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "start", "resp_queue_full");
    }
}

static void motion_send_move_end_response(uint8_t frame_id, uint8_t status) {
    uint8_t raw[5];
    move_end_resp_t resp; resp.frameId = frame_id; resp.status = status;
    if (move_end_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) return;
    if (app_resp_push(raw, (uint32_t)sizeof raw) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "move_end", "resp_queue_full");
    }
}


/* =======================
 *  Init
 * ======================= */
void motion_service_init(void) {
    uint32_t primask = motion_lock();

    memset(&g_status, 0, sizeof g_status);
    memset(g_axis_state, 0, sizeof g_axis_state);
    memset(g_queue, 0, sizeof g_queue);
    memset(g_encoder_position, 0, sizeof g_encoder_position);
    memset(g_encoder_last_raw, 0, sizeof g_encoder_last_raw);
    memset(g_encoder_origin, 0, sizeof g_encoder_origin);
    memset(g_encoder_delta_tick, 0, sizeof g_encoder_delta_tick);
    memset(g_pi_i_accum, 0, sizeof g_pi_i_accum);
    memset(g_pi_prev_err, 0, sizeof g_pi_prev_err);
    memset(g_origin_base32, 0, sizeof g_origin_base32);
    memset(g_pi_d_filt, 0, sizeof g_pi_d_filt);
    memset(g_v_accum, 0, sizeof g_v_accum);

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

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) Error_Handler();

    LOGT_THIS(LOG_STATE_START, PROTO_OK, "init", "timers_ready");
}

const motion_status_t* motion_status_get(void) {
    return &g_status;
}

/* =======================
 *  Tick do TIM6 (50 kHz)
 *  - fecha largura de pulso
 *  - DEMO: DDA suave
 *  - Fila: caminho original
 * ======================= */
void motion_on_tim6_tick(void)
{
    if (g_status.state != MOTION_RUNNING || !g_has_active_segment)
        return;

    /* 1) Fecha pulsos altos pendentes (garante largura do STEP) */
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        motion_axis_state_t *ax = &g_axis_state[axis];
        if (ax->step_high) {
            if (--ax->step_high == 0u) {
                motion_hw_step_low(axis);
                ax->step_low = MOTION_STEP_LOW_TICKS; /* Para voltar ao comportamento anterior, defina MOTION_STEP_LOW_TICKS=0u */
            }
        } else if (ax->step_low) {
            --ax->step_low;
        }
    }

    if (g_demo_continuous) {
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            motion_axis_state_t *ax = &g_axis_state[axis];

            if (ax->emitted_steps >= ax->total_steps) continue;

            /* guardas de ENABLE e DIR (atendem setup/hold do TMC5160) */
            if (ax->en_settle_ticks)  { ax->en_settle_ticks--;  continue; }
            if (ax->dir_settle_ticks) { ax->dir_settle_ticks--; continue; }

            if (ax->step_high) continue; /* ainda segurando pulso ALTO */
            if (ax->step_low)  continue; 

            /* DDA: acumula fase e emite STEP ao cruzar 1.0 */
            ax->dda_accum_q16 += ax->dda_inc_q16;
            if (ax->dda_accum_q16 >= Q16_1) {
                ax->dda_accum_q16 -= Q16_1;

                motion_hw_step_high(axis);
                ax->step_high = MOTION_STEP_HIGH_TICKS;
                ++ax->emitted_steps;
#if MOTION_DEBUG_FLOW
                #if MOTION_DEBUG_TIM6_PRINTS
                if ((++g_step_print_count[axis] % MOTION_DEBUG_STEP_DECIM) == 1u) {
                    printf("[STEP axis=%u emitted=%lu target=%lu total=%lu]\r\n",
                           (unsigned)axis,
                           (unsigned long)ax->emitted_steps,
                           (unsigned long)ax->target_steps,
                           (unsigned long)ax->total_steps);
                }
                #endif
#endif
            }
        }
    }
    else {
        /* 3) Caminho original (fila): preservado */
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            motion_axis_state_t *ax = &g_axis_state[axis];

            if (ax->step_high) continue;
            if (ax->step_low)  continue; 
            if (ax->emitted_steps >= ax->total_steps) continue;

            if (ax->en_settle_ticks)  { ax->en_settle_ticks--;  continue; }
            if (ax->dir_settle_ticks) { ax->dir_settle_ticks--; continue; }

            /* DDA (fila): acumula fase e emite STEP no cruzamento de 1.0 */
            ax->dda_accum_q16 += ax->dda_inc_q16;
            if (ax->dda_accum_q16 >= Q16_1) {
                ax->dda_accum_q16 -= Q16_1;
                if (ax->emitted_steps < ax->total_steps) {
                    motion_hw_step_high(axis);
                    ax->step_high = MOTION_STEP_HIGH_TICKS;
                    ++ax->emitted_steps;
                    ax->target_steps = ax->emitted_steps;
#if MOTION_DEBUG_FLOW
                    #if MOTION_DEBUG_TIM6_PRINTS
                    if ((++g_step_print_count[axis] % MOTION_DEBUG_STEP_DECIM) == 1u) {
                        printf("[STEP axis=%u emitted=%lu target=%lu total=%lu]\\r\\n",
                               (unsigned)axis,
                               (unsigned long)ax->emitted_steps,
                               (unsigned long)ax->target_steps,
                               (unsigned long)ax->total_steps);
                    }
                    #endif
#endif
                }
            }
        }
    }
    uint32_t primask = motion_lock();
    if (g_has_active_segment) {
        uint8_t confirm = 1u;
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            const motion_axis_state_t *ax = &g_axis_state[axis];
            if (ax->emitted_steps < ax->total_steps || ax->step_high) {
                confirm = 0u; break;
            }
        }
        if (confirm) {
            if (motion_try_start_next_locked()) {
                g_status.state = MOTION_RUNNING;
#if MOTION_DEBUG_FLOW
                printf("[FLOW next_segment started]\r\n");
#endif
            } else {
                g_has_active_segment = 0u;
                motion_stop_all_axes_locked();
                g_status.state = MOTION_DONE;
                motion_send_move_end_response(g_active_frame_id, 0u /* natural_done */);
#if MOTION_DEBUG_ENCODERS
                printf("[ENC DONE abs=(%ld,%ld,%ld) rel=(%ld,%ld,%ld) target=(%lu,%lu,%lu)]\r\n",
                       (long)g_enc_abs32[AXIS_X],
                       (long)g_enc_abs32[AXIS_Y],
                       (long)g_enc_abs32[AXIS_Z],
                       (long)g_enc_rel32[AXIS_X],
                       (long)g_enc_rel32[AXIS_Y],
                       (long)g_enc_rel32[AXIS_Z],
                       (unsigned long)g_axis_state[AXIS_X].total_steps,
                       (unsigned long)g_axis_state[AXIS_Y].total_steps,
                       (unsigned long)g_axis_state[AXIS_Z].total_steps);
#endif
            }
            motion_refresh_status_locked();
        }
    }
    motion_unlock(primask);
}

/* =======================
 *  Tick do TIM7 (~1 kHz)
 *  - Atualiza encoders
 *  - DEMO: rampa e dda_inc
 *  - Fila: sua original de target_steps
 * ======================= */
void motion_on_tim7_tick(void)
{
    motion_update_encoders();

    // Atualiza sombras 32-bit para SWV/Data Trace (4 bytes por amostra)
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        g_enc_abs32[axis] = (int32_t)g_encoder_position[axis];
        g_enc_rel32[axis] = (int32_t)(g_encoder_position[axis] - g_encoder_origin[axis]);
    }

    /* DEMO: aplica rampa e calcula incremento do DDA */
    if (g_status.state == MOTION_RUNNING && g_has_active_segment && g_demo_continuous) {
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            motion_axis_state_t *ax = &g_axis_state[axis];
            if (ax->emitted_steps >= ax->total_steps) continue;

            /* Aceleração integrada: acumula a/1000 e aplica passos discretos em v */
            g_v_accum[axis] += ax->accel_sps2; /* steps/s^2 * 1ms */
            uint32_t steps_avail = 0u;
            while (g_v_accum[axis] >= 1000u) { g_v_accum[axis] -= 1000u; steps_avail++; }
            while (steps_avail--) {
                if (ax->v_actual_sps < ax->v_target_sps) {
                    ax->v_actual_sps++;
                    if (ax->v_actual_sps > ax->v_target_sps) ax->v_actual_sps = ax->v_target_sps;
                } else if (ax->v_actual_sps > ax->v_target_sps) {
                    if (ax->v_actual_sps > 0u) ax->v_actual_sps--;
                    if (ax->v_actual_sps < ax->v_target_sps) ax->v_actual_sps = ax->v_target_sps;
                }
            }
            if (ax->v_actual_sps > MOTION_MAX_SPS) ax->v_actual_sps = MOTION_MAX_SPS;
            ax->dda_inc_q16 = Q16_DIV_UINT(ax->v_actual_sps, MOTION_TIM6_HZ);
        }
    }
    /* Caminho da fila: rampa trapezoidal (acelera/cruza/desacelera) e define incremento DDA */
    if (g_status.state == MOTION_RUNNING && g_has_active_segment && !g_demo_continuous) {
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            motion_axis_state_t *ax = &g_axis_state[axis];
            /* Mesmo que o segmento ativo para este eixo tenha zerado, podemos ter
               passos remanescentes na fila — mantemos a rampa global da lista. */

            uint32_t v_cmd_sps = ((uint32_t)ax->velocity_per_tick) * 1000u; /* alvo/cruzeiro */
            /* PI de posição: ajusta v_cmd_sps com base no erro posicional */
#if MOTION_PI_ENABLE
            if ((ax->kp | ax->ki | ax->kd) != 0u) {
                /* desired (em passos DDA) vs actual convertido de contagens do encoder para passos DDA */
                int32_t desired = (int32_t)ax->target_steps;
                int64_t enc_rel = g_encoder_position[axis] - g_encoder_origin[axis];
                /* actual_steps ≈ enc_rel * (DDA_STEPS_PER_REV / ENC_COUNTS_PER_REV) */
                int64_t num = enc_rel * (int64_t)dda_steps_per_rev();
                int32_t actual = 0;
                if (ENC_COUNTS_PER_REV[axis] > 0u) {
                    int64_t q = num / (int64_t)ENC_COUNTS_PER_REV[axis];
                    if (q > INT32_MAX) q = INT32_MAX; else if (q < INT32_MIN) q = INT32_MIN;
                    actual = (int32_t)q;
                }
                int32_t err = desired - actual;
                /* Deadband simples */
                if (err > -((int32_t)MOTION_PI_DEADBAND_STEPS) && err < (int32_t)MOTION_PI_DEADBAND_STEPS) {
                    err = 0;
                }
                /* Integral com anti-windup (em unidades de passos) */
                int32_t iacc = g_pi_i_accum[axis] + err;
                if (iacc > MOTION_PI_I_CLAMP) iacc = MOTION_PI_I_CLAMP;
                else if (iacc < -MOTION_PI_I_CLAMP) iacc = -MOTION_PI_I_CLAMP;
                int32_t draw = err - g_pi_prev_err[axis];
                g_pi_prev_err[axis] = err;
                /* Derivada filtrada: g_pi_d_filt += (draw - g_pi_d_filt) >> alpha */
                const int32_t alpha = 8; /* filtro leve (1..16) */
                g_pi_d_filt[axis] = g_pi_d_filt[axis] + ((draw - g_pi_d_filt[axis]) >> alpha);
                int32_t pterm = ((int32_t)ax->kp * err) >> MOTION_PI_SHIFT;      /* steps/s */
                int32_t iterm = ((int32_t)ax->ki * iacc) >> MOTION_PI_SHIFT;     /* steps/s */
                int32_t dterm = (ax->kd != 0u) ? (((int32_t)ax->kd * g_pi_d_filt[axis]) >> MOTION_PI_SHIFT) : 0; /* steps/s */
                int32_t corr = pterm + iterm + dterm; /* correção em steps/s */
                if (corr > (int32_t)MOTION_PI_CORR_MAX_SPS) corr = (int32_t)MOTION_PI_CORR_MAX_SPS;
                else if (corr < -(int32_t)MOTION_PI_CORR_MAX_SPS) corr = -(int32_t)MOTION_PI_CORR_MAX_SPS;
                int32_t v_adj = (int32_t)v_cmd_sps + corr;
                if (v_adj < 0) v_adj = 0;
                if (v_adj > (int32_t)MOTION_MAX_SPS) v_adj = (int32_t)MOTION_MAX_SPS; /* limite físico */
                v_cmd_sps = (uint32_t)v_adj;
                /* Anti-windup por saturação: só aceita a integral quando não saturou */
                if (!(v_adj == 0 || v_adj == (int32_t)MOTION_MAX_SPS)) {
                    g_pi_i_accum[axis] = iacc;
                }
            }
#endif
            uint32_t a_sps2    = (ax->accel_sps2 > 0u) ? ax->accel_sps2 : DEMO_ACCEL_SPS2;

            /* Distância restante total (ativo + fila) em passos */
            uint32_t rem_steps = motion_remaining_steps_total_for_axis(axis);

            /* Distância necessária para frear de v para 0: s = v^2 / (2a) */
            uint32_t v_now = ax->v_actual_sps;
            uint32_t s_brake = 0u;
            if (a_sps2 > 0u && v_now > 0u) {
                uint64_t vv = (uint64_t)v_now * (uint64_t)v_now;
                uint64_t denom = (uint64_t)(2u * a_sps2);
                s_brake = (uint32_t)(vv / denom);
            }

            /* Política de rampa:
             * - Se já estamos perto do final (rem_steps <= s_brake): desacelera.
             * - Caso contrário, acelera até v_cmd_sps; se passou, reduz até v_cmd_sps. */
            /* Aceleração integrada: usa g_v_accum para passos discretos de v */
            g_v_accum[axis] += a_sps2; /* steps/s^2 * 1ms */
            uint32_t steps_avail = 0u;
            while (g_v_accum[axis] >= 1000u) { g_v_accum[axis] -= 1000u; steps_avail++; }
            while (steps_avail--) {
                if (rem_steps <= s_brake) {
                    /* Desacelera */
                    if (ax->v_actual_sps > 0u) ax->v_actual_sps--;
                } else if (ax->v_actual_sps < v_cmd_sps) {
                    ax->v_actual_sps++;
                    if (ax->v_actual_sps > v_cmd_sps) ax->v_actual_sps = v_cmd_sps;
                } else if (ax->v_actual_sps > v_cmd_sps) {
                    if (ax->v_actual_sps > 0u) ax->v_actual_sps--;
                    if (ax->v_actual_sps < v_cmd_sps) ax->v_actual_sps = v_cmd_sps;
                }
            }

            /* Se não há mais nada a emitir neste eixo, força zero */
            if (rem_steps == 0u) ax->v_actual_sps = 0u;
            if (v_cmd_sps > MOTION_MAX_SPS) v_cmd_sps = MOTION_MAX_SPS;
            if (ax->v_actual_sps > MOTION_MAX_SPS) ax->v_actual_sps = MOTION_MAX_SPS;

            /* Incremento do DDA a 50 kHz */
            ax->dda_inc_q16 = Q16_DIV_UINT(ax->v_actual_sps, MOTION_TIM6_HZ);
        }
    }

}

/* =======================
 *  Handlers de protocolo
 * ======================= */
void motion_on_move_queue_add(const uint8_t *frame, uint32_t len) {

    move_queue_add_req_t req;
    uint8_t ack_status = MOTION_ACK_INVALID;
    uint8_t frame_id = 0u;

    if (!frame) return;
    proto_result_t decode_status = move_queue_add_req_decoder(frame, len, &req);
    if (decode_status != PROTO_OK) {
        motion_send_queue_add_ack(frame_id, ack_status);
        LOGA_THIS(LOG_STATE_ERROR, decode_status, "queue_add", "decode_fail");
        return;
    }
    frame_id = req.frameId;

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
#if MOTION_DEBUG_FLOW
        printf("[FLOW queue_add ok id=%u dir=0x%02X V=(%u,%u,%u) S=(%lu,%lu,%lu) depth=%u]\r\n",
               (unsigned)frame_id,
               (unsigned)req.dirMask,
               (unsigned)req.vx, (unsigned)req.vy, (unsigned)req.vz,
               (unsigned long)req.sx, (unsigned long)req.sy, (unsigned long)req.sz,
               (unsigned)g_status.queue_depth);
#endif
    } else {
        ack_status = MOTION_ACK_QUEUE_FULL;
    }
    motion_unlock(primask);

    motion_send_queue_add_ack(frame_id, ack_status);
    LOGA_THIS(LOG_STATE_RECEIVED, ack_status, "queue_add",
              "frame=%u dirMask=0x%02X queue=%u",
              (unsigned)frame_id, (unsigned)req.dirMask, (unsigned)g_status.queue_depth);
}

void motion_on_move_queue_status(const uint8_t *frame, uint32_t len) {
    move_queue_status_req_t req;
    if (move_queue_status_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "queue_status", "decode_fail");
        return;
    }
    uint32_t primask = motion_lock();
    motion_refresh_status_locked();
    motion_unlock(primask);

    motion_send_queue_status_response(req.frameId);
    LOGA_THIS(LOG_STATE_RECEIVED, PROTO_OK, "queue_status",
              "state=%u depth=%u pct=(%u,%u,%u)",
              (unsigned)g_status.state, (unsigned)g_status.queue_depth,
              (unsigned)g_status.pctX, (unsigned)g_status.pctY, (unsigned)g_status.pctZ);
}

void motion_on_start_move(const uint8_t *frame, uint32_t len) {
    start_move_req_t req;
    if (start_move_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "start_move", "decode_fail");
        return;
    }
    uint8_t started = 0u;
    uint32_t primask = motion_lock();

#if MOTION_DEBUG_FLOW
    {
        uint8_t depth = (uint8_t)(g_queue_count + (g_has_active_segment ? 1u : 0u));
        printf("[FLOW start_move request depth=%u active=%u ids=(",
               (unsigned)depth, (unsigned)g_active_frame_id);
        for (uint8_t i = 0; i < g_queue_count; ++i) {
            uint8_t idxq = (uint8_t)((g_queue_head + i) % MOTION_QUEUE_CAPACITY);
            unsigned id = (unsigned)g_queue[idxq].req.frameId;
            printf(i ? ",%u" : "%u", id);
        }
        printf(") ]\r\n");
    }
#endif

    if (!safety_is_safe()) {
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

    (void)HAL_TIM_Base_Start_IT(&htim6);
    (void)HAL_TIM_Base_Start_IT(&htim7);

    motion_send_start_response(req.frameId, started ? 0u : 1u, g_status.queue_depth);
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "start_move", started ? "running" : "ignored");
#if MOTION_DEBUG_FLOW
    printf("[FLOW start_move %s]\r\n", started ? "running" : "ignored");
#endif
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

    motion_send_move_end_response(req.frameId, 1u /* stopped by host */);

    primask = motion_lock();
    g_status.state = MOTION_IDLE;
    motion_refresh_status_locked();
    motion_unlock(primask);

    LOGT_THIS(LOG_STATE_APPLIED, PROTO_OK, "move_end", "stopped");
}

/* =======================
 *  set_origin e encoder_status (telemetria)
 * ======================= */
void motion_on_set_origin(const uint8_t *frame, uint32_t len) {
    set_origin_req_t req;
    if (set_origin_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "set_origin", "decode_fail");
        return;
    }
    uint8_t m = req.mask & 0x07u;
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        if (m & (1u << axis)) {
            /* Define base externa como a posição absoluta atual e zera relativo */
            int64_t pos = g_encoder_position[axis];
            if (pos > INT32_MAX) pos = INT32_MAX; else if (pos < INT32_MIN) pos = INT32_MIN;
            g_origin_base32[axis] = (int32_t)pos;
            g_encoder_origin[axis] = g_encoder_position[axis];
        }
    }
    set_origin_resp_t resp;
    resp.frameId = req.frameId;
    resp.x0 = g_origin_base32[AXIS_X];
    resp.y0 = g_origin_base32[AXIS_Y];
    resp.z0 = g_origin_base32[AXIS_Z];
    uint8_t raw[16];
    if (set_origin_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    }
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "set_origin", "mask=0x%02X mode=%u", (unsigned)req.mask, (unsigned)req.mode);
}

void motion_on_encoder_status(const uint8_t *frame, uint32_t len) {
    encoder_status_req_t req;
    if (encoder_status_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "enc_status", "decode_fail");
        return;
    }
    /* posição_rel = position - origin; posição_abs = origin_base + posição_rel */
    int32_t rel[3];
    int32_t abs[3];
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        int64_t r = g_encoder_position[axis] - g_encoder_origin[axis];
        if (r > INT32_MAX) r = INT32_MAX; else if (r < INT32_MIN) r = INT32_MIN;
        rel[axis] = (int32_t)r;
        int64_t a = (int64_t)g_origin_base32[axis] + (int64_t)rel[axis];
        if (a > INT32_MAX) a = INT32_MAX; else if (a < INT32_MIN) a = INT32_MIN;
        abs[axis] = (int32_t)a;
    }
    encoder_status_resp_t resp;
    resp.frameId = req.frameId;
    resp.pidErrX = (uint8_t)g_status.pidErrX;
    resp.pidErrY = (uint8_t)g_status.pidErrY;
    resp.pidErrZ = (uint8_t)g_status.pidErrZ;
    resp.delta = 0; /* opcional */
    resp.absX = abs[AXIS_X];
    resp.absY = abs[AXIS_Y];
    resp.absZ = abs[AXIS_Z];
    uint8_t raw[20];
    if (encoder_status_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    }
}

void motion_on_set_microsteps(const uint8_t *frame, uint32_t len) {
    set_microsteps_req_t req;
    if (set_microsteps_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "set_microsteps", "decode_fail");
        return;
    }
    if (g_status.state == MOTION_RUNNING) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "set_microsteps", "busy_running");
        return;
    }
    uint16_t ms = (req.microsteps == 0u) ? 1u : req.microsteps;
    if (ms > 256u) ms = 256u;
    g_microstep_factor = ms;
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "set_microsteps", "ms=%u", (unsigned)ms);
}

/* =====================================================================
 *  DEMO DE MOVIMENTO (PARA TESTE EM BANCADA)
 *  - DDA por TIM6 (50 kHz) com rampa no TIM7 (~1 kHz)
 *  - Timings de STEP/DIR/ENABLE conforme TMC5160
 * ===================================================================== */
void motion_demo_set_enabled(uint8_t enable)
{
    if (!enable) return;

    uint32_t primask = motion_lock();
    if (g_has_active_segment || g_queue_count > 0u) {
        motion_unlock(primask);
        return;
    }

    /* Segmento de teste: XYZ para frente, alvos moderados */
    move_queue_add_req_t req = {0};
    req.frameId = 0xEE;
    req.dirMask = 0x07;            /* XYZ forward */
    req.vx = 10; req.vy = 8; req.vz = 6;      
    req.sx = 2000; req.sy = 1600; req.sz = 1200;

    if (g_queue_count < MOTION_QUEUE_CAPACITY) {
        g_queue[g_queue_tail].req = req;
        g_queue_tail = (uint8_t)((g_queue_tail + 1u) % MOTION_QUEUE_CAPACITY);
        g_queue_count++;
    }

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

void motion_demo_set_continuous(uint8_t enable)
{
    uint32_t primask = motion_lock();
    g_demo_continuous = (enable ? 1u : 0u);

    if (g_demo_continuous) {
        g_has_active_segment = 1u;

        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            motion_axis_state_t *ax = &g_axis_state[axis];

            ax->total_steps       = 0xFFFFFFFFu; 
            ax->target_steps      = 0u;
            ax->emitted_steps     = 0u;

            /* tabela em "k steps/s" (~1 kHz) -> converte para steps/s */
            uint16_t vtab         = g_demo_speed_table[g_demo_speed_idx & 0x3u];
            ax->velocity_per_tick = vtab;
            ax->v_target_sps      = ((uint32_t)vtab) * 1000u;
            if (ax->v_target_sps > MOTION_MAX_SPS) ax->v_target_sps = MOTION_MAX_SPS;
            ax->v_actual_sps      = 0u;
            ax->accel_sps2        = DEMO_ACCEL_SPS2;

            ax->dda_accum_q16     = 0u;
            ax->dda_inc_q16       = 0u;

            ax->step_high         = 0u;
            ax->step_low          = 0u;
            ax->en_settle_ticks   = MOTION_ENABLE_SETTLE_TICKS;
            ax->dir_settle_ticks  = MOTION_DIR_SETUP_TICKS;

            motion_hw_step_low(axis);
            motion_hw_set_dir(axis, 1u);     /* forward */
            motion_hw_enable(axis, 1u);      /* ativo em baixo no TMC5160 */
            /* Não zera origem automaticamente; mantém a referência definida via set_origin */
            g_encoder_delta_tick[axis] = 0;
            g_pi_i_accum[axis] = 0;
            g_pi_prev_err[axis] = 0;
        }
        g_status.state = MOTION_RUNNING;
        motion_refresh_status_locked();
    } else {
        motion_stop_all_axes_locked();
        motion_queue_clear_locked();
        g_has_active_segment = 0u;
        g_status.state = MOTION_IDLE;
        motion_refresh_status_locked();
    }
    motion_unlock(primask);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (!htim) return;
    if (htim->Instance == TIM6) {
        motion_on_tim6_tick();
    } else if (htim->Instance == TIM7) {
        motion_on_tim7_tick();
    } else {
        (void)htim;
    }
}

void motion_emergency_stop(void)
{
    uint32_t primask = motion_lock();

    g_demo_continuous = 0u;
    motion_stop_all_axes_locked();
    motion_queue_clear_locked();
    g_has_active_segment = 0u;

    g_status.state = MOTION_STOPPING;
    motion_refresh_status_locked();
    motion_unlock(primask);

    primask = motion_lock();
    g_status.state = MOTION_IDLE;
    motion_refresh_status_locked();
    motion_unlock(primask);
    /* Notifica término por emergência (se houver frame ativo) */
    if (g_active_frame_id) {
        motion_send_move_end_response(g_active_frame_id, 2u /* emergency */);
    }
}

uint8_t motion_demo_is_active(void)
{
    return g_demo_continuous ? 1u : 0u;
}

void motion_demo_cycle_speed(void)
{
    g_demo_speed_idx = (uint8_t)((g_demo_speed_idx + 1u) & 0x3u);

    /* Se demo  ativo, atualiza v_target_sps imediatamente (rampa cuida do resto) */
    if (g_demo_continuous) {
        uint16_t vtab = g_demo_speed_table[g_demo_speed_idx & 0x3u];
        uint32_t primask = motion_lock();
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            g_axis_state[axis].velocity_per_tick = vtab;
            g_axis_state[axis].v_target_sps      = ((uint32_t)vtab) * 1000u;
            if (g_axis_state[axis].v_target_sps > MOTION_MAX_SPS) g_axis_state[axis].v_target_sps = MOTION_MAX_SPS;
        }
        motion_unlock(primask);
    }
}
