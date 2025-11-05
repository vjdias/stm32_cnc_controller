#include "app.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "stm32l4xx.h"  /* CoreDebug/DBGMCU/ITM (checagem SWO ativa) */

#include "Services/Log/log_service.h"
#include "Services/Motion/motion_service.h"
#include "Services/Motion/motion_hw.h"
#include "Services/Safety/safety_service.h"

// Includes de Protocolo
#include "Protocol/Requests/move_queue_add_request.h"
#include "Protocol/Responses/move_queue_add_ack_response.h"
#include "Protocol/Requests/move_queue_status_request.h"
#include "Protocol/Responses/move_queue_status_response.h"
#include "Protocol/Requests/start_move_request.h"
#include "Protocol/Responses/start_move_response.h"
#include "Protocol/Requests/move_end_request.h"
#include "Protocol/Responses/move_end_response.h"
#include "Protocol/Requests/set_origin_request.h"
#include "Protocol/Responses/set_origin_response.h"
#include "Protocol/Requests/encoder_status_request.h"
#include "Protocol/Responses/encoder_status_response.h"
#include "Protocol/Requests/set_microsteps_request.h"
// Novas mensagens de estimativa/diagnóstico
#include "Protocol/Requests/motion_estimate_request.h"
#include "Protocol/Responses/motion_estimate_response.h"
#include "Protocol/Requests/diag_ctrl_request.h"
#include "Protocol/Responses/diag_ctrl_response.h"
#include "Protocol/Requests/set_enc_ppr_request.h"
#include "Protocol/Responses/set_enc_ppr_response.h"
#include "Protocol/Requests/model_run_request.h"
#include "Protocol/Responses/model_run_response.h"

LOG_SVC_DEFINE(LOG_SVC_MOTION, "motion");


#define MOTION_AXIS_COUNT       3u
#define MOTION_QUEUE_CAPACITY   64u

/* =======================
 * Timings p/ TMC5160 + DDA
 * ======================= */
#define MOTION_TIM6_HZ              50000u      /* 50 kHz -> 20 us por tick (Loop Rápido "Operário" DDA) */
#define MOTION_TIM7_HZ              1000u       /* 1 kHz -> 1 ms por tick (Loop Lento "Estrategista" CASC/PID) */
#define MOTION_TIM7_PERIOD_MS       (1000u / MOTION_TIM7_HZ) // 1ms

#define MOTION_STEP_HIGH_TICKS      1u          /* largura do STEP: >= 1 tick (>=20us) */
#define MOTION_DIR_SETUP_TICKS      1u
#define MOTION_ENABLE_SETTLE_TICKS  2u
#define MOTION_STEP_LOW_TICKS       1u

#define Q16_1                   (1u<<16)
#define Q16_FROM_UINT(x)        ((uint32_t)(x) << 16)
#define Q16_DIV_UINT(numer,den) ((uint32_t)(((uint64_t)(numer) << 16) / (uint32_t)(den)))

#ifndef MOTION_DEBUG_ENCODERS
#define MOTION_DEBUG_ENCODERS   1
#endif
#ifndef MOTION_DEBUG_FLOW
#define MOTION_DEBUG_FLOW       1
#endif
#ifndef MOTION_DEBUG_TIM6_PRINTS
#define MOTION_DEBUG_TIM6_PRINTS    0
#endif
#ifndef MOTION_DEBUG_STEP_DECIM
#define MOTION_DEBUG_STEP_DECIM     500u
#endif

#define DEMO_ACCEL_SPS2         200000u  /* steps/s^2 */

/* Limite físico de passos/s */
#define MOTION_MIN_LOW_TICKS    ( (MOTION_STEP_LOW_TICKS) ? (MOTION_STEP_LOW_TICKS) : 1u )
#define MOTION_MAX_SPS          ( (MOTION_TIM6_HZ) / ( (MOTION_STEP_HIGH_TICKS) + (MOTION_MIN_LOW_TICKS) ) )

/* =======================
 * Tipos/estados
 * ======================= */
typedef struct {
    uint32_t total_steps;       // Alvo final (magnitude) do segmento (ex: 300)
    uint32_t emitted_steps;     // Contador de pulsos (apenas para status)

    uint16_t velocity_per_tick; // Velocidade "ideal" (v*) vinda do PC (em steps/s)

    /* Ganhos PID (mantidos) */
    uint16_t kp, ki, kd;

    uint8_t  step_high;         /* >0 = quantos ticks faltam em ALTO (TIM6) */
    uint8_t  step_low;          /* >0 = aguarda ticks em BAIXO (TIM6) */

    /* --- DDA + rampa --- */
    uint32_t dda_accum_q16;     /* Acumulador de fase Q16.16 (TIM6) */
    uint32_t dda_inc_q16;       /* Incremento por tick do TIM6 (Q16.16) (definido pelo TIM7) */
    uint32_t v_target_sps;      /* steps/s alvo (v* do PC) (usado pelo TIM7) */
    uint32_t v_actual_sps;      /* steps/s efetivo com rampa (estado da rampa no TIM7) */
    uint32_t accel_sps2;        /* steps/s^2 (TIM7) */

    /* --- Diagnóstico --- */
    uint32_t start_time_ms;

    uint8_t  en_settle_ticks;
    uint8_t  dir_settle_ticks;
} motion_axis_state_t;

typedef struct {
    move_queue_add_req_t req;
} motion_queue_entry_t;

enum { AXIS_X = 0, AXIS_Y = 1, AXIS_Z = 2 };
enum { MOTION_ACK_OK = 0, MOTION_ACK_INVALID = 1, MOTION_ACK_QUEUE_FULL = 2 };

/* =======================
 * Estado global
 * ======================= */
static motion_status_t g_status;
static motion_axis_state_t g_axis_state[MOTION_AXIS_COUNT];
static volatile uint8_t g_has_active_segment = 0u;

static motion_queue_entry_t g_queue[MOTION_QUEUE_CAPACITY];
static uint8_t g_queue_head = 0u;
static uint8_t g_queue_tail = 0u;
static uint8_t g_queue_count = 0u;
static uint8_t g_active_frame_id = 0u; 

// --- Variáveis do Encoder ---
static int64_t  g_encoder_position[MOTION_AXIS_COUNT];  // Posição absoluta (em contagens)
static uint32_t g_encoder_last_raw[MOTION_AXIS_COUNT];  // Última leitura (para wrap-around)
static int64_t  g_encoder_origin[MOTION_AXIS_COUNT];    // Posição absoluta do encoder no *início* do segmento CASC
static int64_t  g_user_origin_enc[MOTION_AXIS_COUNT];   // Posição absoluta do encoder no comando "set_origin" do usuário

// --- Variáveis do CASC (Cross-Axis Sync Control) ---
static int32_t  g_casc_total_steps_s32[MOTION_AXIS_COUNT]; // Alvo DDA (s*) (com sinal, ex: +300, +50, +10)

// --- Variáveis do PID (Controlador de Posição) ---
#define MOTION_PI_SHIFT         8
#define MOTION_PI_I_CLAMP       (200000)
#define MOTION_PI_CORR_MAX_SPS  (MOTION_MAX_SPS)
#define MOTION_PI_ENABLE        1
#define MOTION_PI_DEADBAND_STEPS 10

static int32_t g_pi_d_filt[MOTION_AXIS_COUNT];   // Filtro da Derivada
static uint32_t g_v_accum[MOTION_AXIS_COUNT];    // Acumulador da Rampa (para aceleração)
static int32_t g_pi_i_accum[MOTION_AXIS_COUNT];  // Acumulador do Integrador (I-term)
static int32_t g_pi_prev_err[MOTION_AXIS_COUNT]; // Erro anterior (para D-term)

// --- Sombras 32-bit para Debug (SWV Data Trace/Graph) ---
volatile int32_t g_enc_abs32[MOTION_AXIS_COUNT];
volatile int32_t g_enc_rel_user_s32[MOTION_AXIS_COUNT]; // Relativo ao 'set_origin' (g_user_origin_enc)
volatile int32_t g_casc_err_s32[MOTION_AXIS_COUNT]; // Erro de CASC (Alvo_Sync - Atual_Sync)

static uint32_t g_step_print_count[MOTION_AXIS_COUNT] = {0,0,0};
static volatile uint8_t g_demo_continuous = 0u;
static const uint16_t g_demo_speed_table[4] = { 5u, 10u, 20u, 40u };
static volatile uint8_t g_demo_speed_idx = 1u;

/* =======================
 * Calibração (Sem floats)
 * ======================= */
#define MICROSTEP_FACTOR 256
#define STEPS_PER_REV_BASE  400u
static uint32_t g_enc_counts_per_rev[3] = { 40000u, 2500u, 40000u}; // X,Y,Z (ajustável via protocolo)
static volatile uint16_t g_microstep_factor = MICROSTEP_FACTOR;
static inline uint32_t dda_steps_per_rev(void) { return STEPS_PER_REV_BASE * (uint32_t)g_microstep_factor; }

/* =======================
 * Estimador de parâmetros (acel/const/decel)
 * ======================= */
typedef struct {
    volatile uint8_t armed;    // armado (após queue_add)
    volatile uint8_t active;   // coletando (após detectar movimento do encoder)
    volatile uint8_t valid;    // possui resultados válidos (após finalizar)
    int8_t  axis;              // eixo alvo (0..2) ou -1

    // Janela de amostragem leve (10 ms)
    uint16_t win_ms;
    int32_t  last_pos;
    int32_t  acc_pos;
    int32_t  v_prev_sps;

    // Acumuladores
    int64_t sum_accel;  uint32_t cnt_accel;
    int64_t sum_cruise; uint32_t cnt_cruise;
    int64_t sum_decel;  uint32_t cnt_decel;

    // Resultados
    int32_t avg_accel_sps2;
    int32_t avg_cruise_sps;
    int32_t avg_decel_sps2;
} motion_estimator_t;

static volatile uint8_t g_diag_swo_spd_enable = 0u; // bit0 habilita "SPD:" via SWO
static motion_estimator_t g_est;

static inline void est_reset(motion_estimator_t *e) {
    memset((void*)e, 0, sizeof(*e));
    e->axis = -1;
    e->win_ms = 0;
    e->v_prev_sps = 0;
}

// =======================
// Gerador RAW (modelo) — sem CASC/PID, frequência fixa
// =======================
typedef struct {
    volatile uint8_t active;  // 1=rodando
    uint8_t axis;             // 0..2
    uint8_t dir;              // 0=pos,1=neg
    uint32_t freq_sps;        // steps/s
    uint32_t ppr;             // pulsos/volta (encoder)
    uint32_t turns;           // voltas a cumprir
    int32_t  enc_start;       // posição inicial do encoder (contagens)
    // Acumulador Q16 p/ gerar passos com TIM6 (sem vínculo ao DDA do CASC)
    uint32_t accum_q16;
    uint32_t inc_q16;         // (freq_sps / TIM6_HZ) em Q16
} raw_gen_t;

static raw_gen_t g_raw;

static inline void raw_reset(void) { memset(&g_raw, 0, sizeof g_raw); }

static inline void est_arm_for_axis(int8_t axis, int32_t start_pos) {
    est_reset(&g_est);
    g_est.axis = axis;
    g_est.armed = 1u;
    g_est.last_pos = start_pos;
}

static inline void est_finalize_if_possible(void) {
    if (!g_est.active && !g_est.armed) return;
    // Calcula médias (ou zera)
    g_est.avg_accel_sps2  = (g_est.cnt_accel  ? (int32_t)(g_est.sum_accel  / (int64_t)g_est.cnt_accel)  : 0);
    g_est.avg_cruise_sps  = (g_est.cnt_cruise ? (int32_t)(g_est.sum_cruise / (int64_t)g_est.cnt_cruise) : 0);
    g_est.avg_decel_sps2  = (g_est.cnt_decel  ? (int32_t)(g_est.sum_decel  / (int64_t)g_est.cnt_decel)  : 0);
    g_est.valid = 1u;
    g_est.active = 0u;
    g_est.armed = 0u;
}

/* =======================
 * Helpers de lock (Sem mudança)
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
 * Helpers de acesso por eixo (Sem mudança)
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

/* =======================
 * Conversão Encoder <-> DDA (NOVO)
 * ======================= */

/**
 * @brief Converte a posição absoluta do encoder (contagens) para a posição DDA (passos)
 * @note  Usa matemática 64-bit para evitar overflow.
 */
static int64_t motion_conv_enc_to_dda(int64_t enc_counts, uint8_t axis) {
    if (g_enc_counts_per_rev[axis] == 0) return 0;
    // (enc * dda_steps) / enc_per_rev
    int64_t num = enc_counts * (int64_t)dda_steps_per_rev();
    // Adiciona metade do denominador para arredondamento correto (round-to-nearest)
    num += (int64_t)g_enc_counts_per_rev[axis] / 2;
    return num / (int64_t)g_enc_counts_per_rev[axis];
}

/* =======================
 * Lógica de Rampa (Movido para Helper)
 * ======================= */
/**
 * @brief Atualiza a rampa de velocidade para um eixo, baseado na velocidade CASC
 * e na distância restante do *eixo mestre*.
 */
static uint32_t motion_update_rampa(uint8_t axis, uint32_t v_cmd_casc, uint32_t rem_steps_mestre)
{
    motion_axis_state_t *ax = &g_axis_state[axis];
    uint32_t a_sps2  = (ax->accel_sps2 > 0u) ? ax->accel_sps2 : DEMO_ACCEL_SPS2;
    uint32_t v_now = ax->v_actual_sps;

    // Distância necessária para frear de v_now para 0: s = v^2 / (2a)
    uint32_t s_brake = 0u;
    if (a_sps2 > 0u && v_now > 0u) {
        uint64_t vv = (uint64_t)v_now * (uint64_t)v_now;
        uint64_t denom = (uint64_t)(2u * a_sps2);
        if (denom > 0) {
            s_brake = (uint32_t)(vv / denom);
        }
    }

    // Aceleração integrada: acumula a/1000 e aplica passos discretos em v
    g_v_accum[axis] += a_sps2;
    uint32_t steps_avail = 0u;
    while (g_v_accum[axis] >= MOTION_TIM7_HZ) {
        g_v_accum[axis] -= MOTION_TIM7_HZ;
        steps_avail++;
    }

    while (steps_avail--) {
        // [MODIFICADO] A decisão de desacelerar usa a distância do MESTRE
        if (rem_steps_mestre <= s_brake) {
            // Desacelera (estamos na rampa de parada)
            if (ax->v_actual_sps > 0u) ax->v_actual_sps--;
        } else if (ax->v_actual_sps < v_cmd_casc) {
            // Acelera
            ax->v_actual_sps++;
            if (ax->v_actual_sps > v_cmd_casc) ax->v_actual_sps = v_cmd_casc;
        } else if (ax->v_actual_sps > v_cmd_casc) {
            // Desacelera (para atingir a velocidade de cruzeiro)
            if (ax->v_actual_sps > 0u) ax->v_actual_sps--;
            if (ax->v_actual_sps < v_cmd_casc) ax->v_actual_sps = v_cmd_casc;
        }
    }

    // Se não há mais nada a emitir (mestre chegou), força zero
    if (rem_steps_mestre == 0u) ax->v_actual_sps = 0u;
    if (ax->v_actual_sps > MOTION_MAX_SPS) ax->v_actual_sps = MOTION_MAX_SPS;

    return ax->v_actual_sps;
}


/* =======================
 * Status e fila (MODIFICADO)
 * ======================= */
static void motion_refresh_status_locked(void) {
    g_status.queue_depth = (uint8_t)(g_queue_count + (g_has_active_segment ? 1u : 0u));

    // --- [LÓGICA CASC "PERCENTUAL" PARA STATUS] ---

    int8_t master_axis = -1;
    int64_t mestre_prog_num = 0; // Progresso (em passos DDA) do mestre
    int64_t mestre_prog_den = 1; // Alvo (em passos DDA) do mestre

    // 1. Acha o mestre (eixo percentualmente mais ATRASADO)
    if (g_has_active_segment) {
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            int32_t total_s32 = g_casc_total_steps_s32[axis];
            if (total_s32 == 0) continue; // Eixo não está em movimento

            // Posição DDA atual relativa ao início do CASC
            int64_t pos_enc_rel = g_encoder_position[axis] - g_encoder_origin[axis];
            int64_t pos_rel_dda = motion_conv_enc_to_dda(pos_enc_rel, axis);

            int64_t prog_num = pos_rel_dda;
            int64_t prog_den = (int64_t)total_s32;

            // Lida com direção negativa (compara valores absolutos)
            if (prog_den < 0) {
                prog_num = -prog_num;
                prog_den = -prog_den;
            }
            if (prog_num < 0) prog_num = 0; // Garante progresso positivo
            if (prog_num > prog_den) prog_num = prog_den; // Clamp (chegou)

            if (master_axis == -1) {
                master_axis = axis;
                mestre_prog_num = prog_num;
                mestre_prog_den = (prog_den == 0) ? 1 : prog_den;
            } else {
                // É (prog_i / prog_den_i) < (mestre_prog_num / mestre_prog_den) ?
                int64_t prog_i_64 = prog_num * mestre_prog_den;
                int64_t prog_mestre_64 = mestre_prog_num * prog_den;

                if (prog_i_64 < prog_mestre_64) {
                    master_axis = axis;
                    mestre_prog_num = prog_num;
                    mestre_prog_den = (prog_den == 0) ? 1 : prog_den;
                }
            }
        }
    }

    // 2. Calcula Pct e Erro de cada eixo baseado no mestre
    uint8_t pct_mestre = 0;
    if (mestre_prog_den > 0) {
        pct_mestre = (uint8_t)((mestre_prog_num * 100L) / mestre_prog_den);
    }

    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        int32_t total_s32 = g_casc_total_steps_s32[axis];
        uint8_t pct = 0u;
        int32_t err = 0;

        if (g_has_active_segment && total_s32 != 0) {
            // Pct = O percentual do mestre é o percentual de todos
            pct = pct_mestre;

            // Erro = O erro CASC calculado no último tick do TIM7
            err = g_casc_err_s32[axis];

        } else if ((!g_has_active_segment && g_status.state == MOTION_DONE) || total_s32 == 0) {
            // Movimento terminou, ou eixo está parado
            pct = 100u;
            err = 0;
        } else {
            // Estado ocioso, antes de começar
            pct = 0u;
            err = 0;
        }

        int8_t err8 = motion_clamp_error(err);
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
        ax->total_steps     = 0u;
        ax->emitted_steps   = 0u;

        ax->velocity_per_tick = 0u;
        ax->kp = 0u; ax->ki = 0u; ax->kd = 0u;

        ax->step_high         = 0u;
        ax->step_low          = 0u;
        ax->en_settle_ticks   = 0u;
        ax->dir_settle_ticks  = 0u;

        ax->dda_accum_q16     = 0u;
        ax->dda_inc_q16       = 0u;
        ax->v_target_sps      = 0u;
        ax->v_actual_sps      = 0u;
        ax->accel_sps2        = 0u;

        // Zera estado CASC e PID
        g_casc_total_steps_s32[axis] = 0;
        g_pi_i_accum[axis] = 0;
        g_pi_prev_err[axis] = 0;
        g_pi_d_filt[axis] = 0;
        g_v_accum[axis] = 0;
        g_casc_err_s32[axis] = 0;
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
        uint32_t total  = motion_total_for_axis(seg, axis);
        uint16_t velTick = motion_velocity_for_axis(seg, axis);
        uint8_t dir     = (uint8_t)((seg->dirMask >> axis) & 0x1u);

        // --- [MODIFICADO] ---
        // Salva os alvos e posições de início para o CASC

        ax->total_steps     = total; // Alvo final (magnitude, ex: 300)
        ax->emitted_steps   = 0u;    // Zera o contador de pulsos

        // Salva a posição absoluta atual do encoder como o "zero" deste segmento
        g_encoder_origin[axis] = g_encoder_position[axis];

        // Salva o alvo final com sinal (direção)
        g_casc_total_steps_s32[axis] = dir ? (int32_t)total : -((int32_t)total);

        ax->start_time_ms = HAL_GetTick();

        ax->velocity_per_tick = velTick;
        ax->kp = motion_kp_for_axis(seg, axis);
        ax->ki = motion_ki_for_axis(seg, axis);
        ax->kd = motion_kd_for_axis(seg, axis);

        if (ax->kp == 0 && ax->ki == 0 && ax->kd == 0) {
             ax->kp = 10; ax->ki = 2; ax->kd = 5; // Ganhos Padrão
        }

        ax->step_high         = 0u;
        ax->step_low          = 0u; 
        ax->en_settle_ticks   = (total > 0u) ? MOTION_ENABLE_SETTLE_TICKS : 0u;
        ax->dir_settle_ticks  = MOTION_DIR_SETUP_TICKS;

        ax->dda_accum_q16     = 0u;
        ax->dda_inc_q16       = 0u;

        // v_target_sps é a velocidade "ideal" (interpolada) vinda do PC
        // O campo 'velocity_per_tick' (vx, vy, vz) agora é steps/s
        ax->v_target_sps      = (uint32_t)velTick;
        if (ax->v_target_sps > MOTION_MAX_SPS) ax->v_target_sps = MOTION_MAX_SPS;

        // Zera a velocidade atual da rampa se o movimento estava parado
        if (g_status.state != MOTION_RUNNING) {
            ax->v_actual_sps  = 0u;
        }
        ax->accel_sps2        = DEMO_ACCEL_SPS2;

        motion_hw_step_low(axis);
        motion_hw_set_dir(axis, dir);
        if (total > 0u) motion_hw_enable(axis, 1u);
        else            motion_hw_enable(axis, 0u);

        // Zera o estado do PID/CASC
        g_pi_i_accum[axis] = 0;
        g_pi_prev_err[axis] = 0;
        g_pi_d_filt[axis] = 0;
        g_v_accum[axis] = 0;
        g_casc_err_s32[axis] = 0;
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
    return 1u;
}

static void motion_update_encoders(void) {
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        uint32_t now = motion_hw_encoder_read_raw(axis);
        uint8_t bits = motion_hw_encoder_bits(axis);
        if (bits == 16u) {
            uint16_t prev = (uint16_t)g_encoder_last_raw[axis];
            int16_t delta = (int16_t)((uint16_t)now - prev);
            g_encoder_last_raw[axis] = (uint16_t)now;
            g_encoder_position[axis] += delta;
        } else {
            int32_t delta = (int32_t)(now - g_encoder_last_raw[axis]);
            g_encoder_last_raw[axis] = now;
            g_encoder_position[axis] += delta;
        }
    }
}

/* =======================
 * Envio de respostas (Sem mudanças)
 * ======================= */
static void motion_send_queue_add_ack(uint8_t frame_id, uint8_t status) {
    uint8_t raw[6];
    move_queue_add_ack_resp_t resp = { frame_id, status };
    if (move_queue_add_ack_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) return;
    app_resp_push(raw, (uint32_t)sizeof raw);
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
    if (move_queue_status_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) return;
    app_resp_push(raw, (uint32_t)sizeof raw);
}
static void motion_send_start_response(uint8_t frame_id, uint8_t status, uint8_t depth) {
    uint8_t raw[6];
    start_move_resp_t resp; resp.frameId = frame_id; resp.status = status; resp.depth = depth;
    if (start_move_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) return;
    app_resp_push(raw, (uint32_t)sizeof raw);
}
static void motion_send_move_end_response(uint8_t frame_id, uint8_t status) {
    uint8_t raw[5];
    move_end_resp_t resp; resp.frameId = frame_id; resp.status = status;
    if (move_end_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) return;
    app_resp_push(raw, (uint32_t)sizeof raw);
}


/* =======================
 * Init (Modificado)
 * ======================= */
void motion_service_init(void) {
    uint32_t primask = motion_lock();

    memset(&g_status, 0, sizeof g_status);
    memset(g_axis_state, 0, sizeof g_axis_state);
    memset(g_queue, 0, sizeof g_queue);
    memset(g_encoder_position, 0, sizeof g_encoder_position);
    memset(g_encoder_last_raw, 0, sizeof g_encoder_last_raw);
    memset(g_encoder_origin, 0, sizeof g_encoder_origin);
    memset(g_user_origin_enc, 0, sizeof g_user_origin_enc); // [NOVO]
    memset(g_pi_i_accum, 0, sizeof g_pi_i_accum);
    memset(g_pi_prev_err, 0, sizeof g_pi_prev_err);
    memset(g_pi_d_filt, 0, sizeof g_pi_d_filt);
    memset(g_v_accum, 0, sizeof g_v_accum);
    memset(g_casc_total_steps_s32, 0, sizeof g_casc_total_steps_s32); // [NOVO]
    memset(g_casc_err_s32, 0, sizeof g_casc_err_s32); // [NOVO]

    g_status.state = MOTION_IDLE;
    g_queue_head = g_queue_tail = g_queue_count = 0u;
    g_has_active_segment = 0u;

    motion_stop_all_axes_locked();
    motion_refresh_status_locked();
    motion_unlock(primask);

    motion_hw_init();

    // Lê a posição inicial do encoder
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        uint32_t raw = motion_hw_encoder_read_raw(axis);
        if (motion_hw_encoder_bits(axis) == 16u) {
            g_encoder_last_raw[axis] = raw & 0xFFFFu;
        } else {
            g_encoder_last_raw[axis] = raw;
        }
        // Define a origem inicial (absoluta e do usuário) como a posição de boot
        g_encoder_position[axis] = 0; // Ou leia o valor salvo da EEPROM
        g_encoder_origin[axis] = 0;
        g_user_origin_enc[axis] = 0;
    }

    if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK) Error_Handler();
    if (HAL_TIM_Base_Start_IT(&htim7) != HAL_OK) Error_Handler();

    LOGT_THIS(LOG_STATE_START, PROTO_OK, "init", "timers_ready");
}

const motion_status_t* motion_status_get(void) {
    return &g_status;
}

/* =======================
 * Tick do TIM6 (50 kHz) - "OPERÁRIO"
 * (Simplificado para ser "burro")
 * ======================= */
void motion_on_tim6_tick(void)
{
    // RAW generator takes precedence
    if (g_raw.active) {
        uint8_t axis = g_raw.axis;
        motion_axis_state_t *ax = &g_axis_state[axis];
        // close high pulses
        if (ax->step_high) {
            if (--ax->step_high == 0u) {
                motion_hw_step_low(axis);
                ax->step_low = MOTION_MIN_LOW_TICKS;
            }
            return;
        }
        if (ax->step_low) { ax->step_low--; return; }

        // phase accumulation independent from CASC DDA
        g_raw.accum_q16 += g_raw.inc_q16;
        if (g_raw.accum_q16 >= Q16_1) {
            g_raw.accum_q16 -= Q16_1;
            motion_hw_step_high(axis);
            ax->step_high = MOTION_STEP_HIGH_TICKS;
        }
        return;
    }

    if (g_status.state != MOTION_RUNNING)
        return;

    /* 1) Fecha pulsos altos pendentes (garante largura do STEP) */
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        motion_axis_state_t *ax = &g_axis_state[axis];
        if (ax->step_high) {
            if (--ax->step_high == 0u) {
                motion_hw_step_low(axis);
                // [FIX 1] Garante que t_low seja *pelo menos* 1,
                // para consistência com o cálculo de MOTION_MAX_SPS.
                ax->step_low = MOTION_MIN_LOW_TICKS;
            }
        } else if (ax->step_low) {
            --ax->step_low;
        }
    }

    /* 2) Lógica DDA "Burra" */
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        motion_axis_state_t *ax = &g_axis_state[axis];

        // Se o "Chefe" (TIM7) zerou o incremento, o DDA para.
        if (ax->dda_inc_q16 == 0) {
            ax->dda_accum_q16 = 0; // Zera o acumulador
            continue;
        }

        if (ax->step_high) continue; // Ainda gerando pulso
        if (ax->step_low)  continue; // Em tempo de baixa (t_low)

        // Se houver guardas de tempo, espera
        if (ax->en_settle_ticks)  { ax->en_settle_ticks--;  continue; }
        if (ax->dir_settle_ticks) { ax->dir_settle_ticks--; continue; }

        /* DDA: acumula fase e emite STEP no cruzamento de 1.0 */
        ax->dda_accum_q16 += ax->dda_inc_q16;
        if (ax->dda_accum_q16 >= Q16_1) {
            ax->dda_accum_q16 -= Q16_1;

            motion_hw_step_high(axis);
            ax->step_high = MOTION_STEP_HIGH_TICKS;
            ++ax->emitted_steps; // Apenas para estatísticas

            // [REMOVIDO] O TIM6 não sabe mais quando o movimento termina.
            // [REMOVIDO] O TIM6 não atualiza mais o target do PID.
        }
    }

    // [REMOVIDO] A lógica de 'if (confirm)' e 'motion_try_start_next_locked'
    // foi MOVIDA para o TIM7, que é quem sabe quando o movimento CASC termina.
}

/* =======================
 * Tick do TIM7 (1 kHz) - "CHEFE CASC/PID"
 * (LÓGICA TOTALMENTE NOVA)
 * ======================= */
void motion_on_tim7_tick(void)
{
    motion_update_encoders();

    // Atualiza sombras 32-bit para SWV/Data Trace (para debug)
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        g_enc_abs32[axis] = (int32_t)g_encoder_position[axis];
        g_enc_rel_user_s32[axis] = (int32_t)(g_encoder_position[axis] - g_user_origin_enc[axis]);
    }

    /* Telemetria leve + Estimador (janela de 10 ms) */
    {
        // Checa SWO ativo (porta 0)
        int swo_enabled = ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) &&
                           (DBGMCU->CR & DBGMCU_CR_TRACE_IOEN) &&
                           (ITM->TCR & ITM_TCR_ITMENA_Msk) &&
                           (ITM->TER & (1UL << 0)));

        // Reporter SWO (opcional)
        static uint16_t telem_elapsed_ms = 0u;
        static int32_t last_enc_sample[3] = {0,0,0};
        const uint16_t period_ms = 100u; // 10 Hz

        telem_elapsed_ms += MOTION_TIM7_PERIOD_MS; // 1ms por tick
        if (swo_enabled && g_diag_swo_spd_enable && g_status.state == MOTION_RUNNING && telem_elapsed_ms >= period_ms) {
            telem_elapsed_ms = 0u;
            for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
                int32_t now = (int32_t)g_encoder_position[axis];
                int32_t delta = now - last_enc_sample[axis];
                last_enc_sample[axis] = now;
                int32_t meas_sps = 0;
                if (period_ms > 0u) {
                    int64_t tmp = (int64_t)delta * 1000LL;
                    meas_sps = (int32_t)(tmp / (int64_t)period_ms);
                }
                uint32_t cmd_sps = g_axis_state[axis].v_target_sps;
                char label = (axis == AXIS_X) ? 'X' : (axis == AXIS_Y) ? 'Y' : 'Z';
                uint32_t elapsed_ms = HAL_GetTick() - g_axis_state[axis].start_time_ms;
                uint32_t steps = g_axis_state[axis].emitted_steps;
                printf("SPD:%c ts=%lu cmd=%lu meas=%ld steps=%lu sps\r\n",
                       label, elapsed_ms, (unsigned long)cmd_sps, (long)meas_sps, (unsigned long)steps);
            }
        }

        // Estimador leve (eixo alvo)
        const uint16_t W_MS = 10u; // janela de 10 ms
        const int32_t A_EPS = 1000; // limiar simples p/ classificar aceleração (steps/s^2)
        int8_t ax = g_est.axis;
        if (g_est.armed && ax >= 0 && ax < (int8_t)MOTION_AXIS_COUNT) {
            int32_t pos_now = (int32_t)g_encoder_position[(uint8_t)ax];
            int32_t dpos = pos_now - g_est.last_pos;
            if (!g_est.active) {
                if (dpos != 0) {
                    g_est.active = 1u; // começou a mover (encoder)
                    g_est.acc_pos = 0;
                    g_est.win_ms = 0;
                    g_est.v_prev_sps = 0;
                }
            }
            if (g_est.active) {
                g_est.acc_pos += dpos;
                g_est.last_pos = pos_now;
                g_est.win_ms += MOTION_TIM7_PERIOD_MS; // 1 ms
                if (g_est.win_ms >= W_MS) {
                    // v (steps/s)
                    int32_t v_curr = 0;
                    int64_t tmp = (int64_t)g_est.acc_pos * 1000LL;
                    v_curr = (int32_t)(tmp / (int64_t)g_est.win_ms);
                    // a (steps/s^2)
                    int32_t a_curr = (int32_t)(((int64_t)(v_curr - g_est.v_prev_sps) * 1000LL) / (int64_t)g_est.win_ms);
                    // Classificação
                    if (a_curr > A_EPS) {
                        g_est.sum_accel += (int64_t)a_curr;
                        g_est.cnt_accel++;
                    } else if (a_curr < -A_EPS) {
                        g_est.sum_decel += (int64_t)(-a_curr); // magnitude positiva
                        g_est.cnt_decel++;
                    } else {
                        g_est.sum_cruise += (int64_t)v_curr;
                        g_est.cnt_cruise++;
                    }
                    g_est.v_prev_sps = v_curr;
                    g_est.acc_pos = 0;
                    g_est.win_ms = 0;
                }
            }
        }
    }

    /* DEMO: usa a lógica de rampa antiga */
    if (g_status.state == MOTION_RUNNING && g_has_active_segment && g_demo_continuous) {
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
             motion_axis_state_t *ax = &g_axis_state[axis];
             if (ax->emitted_steps >= ax->total_steps) continue;

             g_v_accum[axis] += ax->accel_sps2;
             uint32_t steps_avail = 0u;
             while (g_v_accum[axis] >= MOTION_TIM7_HZ) { g_v_accum[axis] -= MOTION_TIM7_HZ; steps_avail++; }
             while (steps_avail--) {
                 if (ax->v_actual_sps < ax->v_target_sps) {
                     ax->v_actual_sps++;
                     if (ax->v_actual_sps > ax->v_target_sps) ax->v_actual_sps = ax->v_target_sps;
                 } else if (ax->v_actual_sps > ax->v_target_sps) {
                     if (ax->v_actual_sps > 0u) ax->v_actual_sps--;
                     if (ax->v_actual_sps < ax->v_target_sps) ax->v_actual_sps = ax->v_target_sps;
                 }
             }

             ax->dda_inc_q16 = Q16_DIV_UINT(ax->v_actual_sps, MOTION_TIM6_HZ);
        }
        return; // Sai da ISR
    }

    // Finalização do modo RAW (modelo): checa encoder e encerra
    if (g_raw.active) {
        int32_t now = (int32_t)g_encoder_position[g_raw.axis];
        int64_t rel = (int64_t)now - (int64_t)g_raw.enc_start;
        if (rel < 0) rel = -rel;
        uint64_t target = (uint64_t)g_raw.turns * (uint64_t)g_raw.ppr;
        if ((uint64_t)rel >= target) {
            // Parar gerador
            uint32_t primask = motion_lock();
            motion_hw_step_low(g_raw.axis);
            motion_hw_enable(g_raw.axis, 0u);
            g_raw.active = 0u;
            motion_unlock(primask);
            // Finaliza estimador
            est_finalize_if_possible();
            // Estado geral idle
            primask = motion_lock();
            g_status.state = MOTION_IDLE;
            motion_refresh_status_locked();
            motion_unlock(primask);
            // Notifica término ao host para compatibilidade com --wait-end
            motion_send_move_end_response(g_active_frame_id, 0u /* natural_done */);
            g_active_frame_id = 0u;
        }
        // Não executar CASC quando RAW ativo
        return;
    }

    /* Caminho da fila: LÓGICA CASC "PERCENTUAL" */
    if (g_status.state == MOTION_RUNNING && g_has_active_segment && !g_demo_continuous) {

        // --- ETAPA 1: ACHAR O "MESTRE" (Eixo percentualmente mais ATRASADO) ---
        int8_t master_axis = -1;
        // uint32_t max_total_steps = 0; // [AFINAMENTO 1] Removido, não era usado

        int64_t mestre_prog_num = 0; // Numerador (progresso DDA) do mestre
        int64_t mestre_prog_den = 1; // Denominador (alvo DDA) do mestre (nunca 0)

        uint8_t all_axes_finished = 1u; // Começa assumindo que todos terminaram

        for (uint8_t i = 0; i < MOTION_AXIS_COUNT; ++i) {
            int32_t total_s32 = g_casc_total_steps_s32[i];
            if (total_s32 == 0) continue; // Este eixo não está se movendo

            all_axes_finished = 0u; // Pelo menos um eixo deve se mover

            // Posição DDA atual relativa ao início do segmento
            int64_t pos_enc_rel = g_encoder_position[i] - g_encoder_origin[i];
            int64_t pos_rel_dda = motion_conv_enc_to_dda(pos_enc_rel, i);

            int64_t prog_num = pos_rel_dda;
            int64_t prog_den = (int64_t)total_s32;

            // Lida com direção negativa (compara valores absolutos)
            if (prog_den < 0) {
                prog_num = -prog_num;
                prog_den = -prog_den;
            }
            if (prog_num < 0) prog_num = 0; // Garante progresso positivo
            if (prog_num > prog_den) prog_num = prog_den; // Clamp (chegou)

            if (master_axis == -1) {
                // Este é o primeiro eixo móvel, ele é o mestre por padrão
                master_axis = i;
                mestre_prog_num = prog_num;
                mestre_prog_den = (prog_den == 0) ? 1 : prog_den;
            } else {
                // Compara o mestre atual com este eixo 'i'
                // É (prog_i / prog_den_i) < (mestre_prog_num / mestre_prog_den) ?
                int64_t prog_i_64 = prog_num * mestre_prog_den;
                int64_t prog_mestre_64 = mestre_prog_num * prog_den;

                if (prog_i_64 < prog_mestre_64) {
                    // Este eixo 'i' está mais atrasado. Ele é o novo mestre.
                    master_axis = i;
                    mestre_prog_num = prog_num;
                    mestre_prog_den = (prog_den == 0) ? 1 : prog_den;
                }
            }

            // [AFINAMENTO 1] Rastreio do max_total_steps removido
        }

        if (master_axis == -1 || all_axes_finished) {
            // Nenhum eixo tem movimento (todos os alvos são 0) OU
            // A flag `all_axes_finished` nunca foi zerada.
            goto _TIM7_FINISH_MOVE;
        }

        // --- ETAPA 2: LOOP DE CONTROLE (PID + CASC + RAMPA) ---

        // Distância restante do MESTRE (para a rampa de desaceleração)
        uint32_t rem_steps_mestre = 0;
        if(mestre_prog_den > mestre_prog_num) {
            rem_steps_mestre = (uint32_t)(mestre_prog_den - mestre_prog_num);
        }

        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            motion_axis_state_t *ax = &g_axis_state[axis];
            int32_t total_s32 = g_casc_total_steps_s32[axis];

            // Pega a velocidade "ideal" (interpolada) vinda do PC
            uint32_t v_cmd_sps_ideal = ax->v_target_sps;

            if (total_s32 == 0) {
                 // Este eixo não deve se mover.
                 ax->v_actual_sps = 0;
                 ax->dda_inc_q16 = 0;
                 g_casc_err_s32[axis] = 0;
                 continue; // Vai para o próximo eixo
            }

            // --- ETAPA 2a: CÁLCULO DO "ALVO SINCRONIZADO" (CASC) ---
            // Qual deveria ser a posição *deste* eixo, com base no progresso do mestre?
            // desired_sync_dda = (mestre_prog_num / mestre_prog_den) * total_steps_s32
            int64_t num_sync_64 = mestre_prog_num * (int64_t)total_s32;
            num_sync_64 += (mestre_prog_den / 2); // Arredondamento
            int32_t desired_dda_steps = (int32_t)(num_sync_64 / mestre_prog_den);

            // --- ETAPA 2b: CÁLCULO DO PID (CASC + PID) ---
            // O PID agora tenta fazer o encoder seguir o "alvo sincronizado".

            int32_t v_final_sps = 0; // A velocidade final ajustada

#if MOTION_PI_ENABLE
            if ((ax->kp | ax->ki | ax->kd) != 0u) {
                // Posição ATUAL do encoder (relativa ao início, em passos DDA)
                int64_t pos_enc_rel = g_encoder_position[axis] - g_encoder_origin[axis];
                int32_t actual_dda_steps = (int32_t)motion_conv_enc_to_dda(pos_enc_rel, axis);

                // O ERRO é a diferença entre o Alvo Sincronizado e a Posição Atual
                int32_t err = desired_dda_steps - actual_dda_steps;
                g_casc_err_s32[axis] = err; // Para debug

                if (err > -((int32_t)MOTION_PI_DEADBAND_STEPS) && err < (int32_t)MOTION_PI_DEADBAND_STEPS) {
                    err = 0;
                }

                int32_t iacc = g_pi_i_accum[axis] + err;
                if (iacc > MOTION_PI_I_CLAMP) iacc = MOTION_PI_I_CLAMP;
                else if (iacc < -MOTION_PI_I_CLAMP) iacc = -MOTION_PI_I_CLAMP;

                int32_t draw = err - g_pi_prev_err[axis];
                g_pi_prev_err[axis] = err;

                const int32_t alpha = 8; // filtro D
                g_pi_d_filt[axis] = g_pi_d_filt[axis] + ((draw - g_pi_d_filt[axis]) >> alpha);

                int32_t pterm = ((int32_t)ax->kp * err) >> MOTION_PI_SHIFT;
                int32_t iterm = ((int32_t)ax->ki * iacc) >> MOTION_PI_SHIFT;
                int32_t dterm = (ax->kd != 0u) ? (((int32_t)ax->kd * g_pi_d_filt[axis]) >> MOTION_PI_SHIFT) : 0;

                int32_t corr = pterm + iterm + dterm; // Correção em steps/s
                if (corr > (int32_t)MOTION_PI_CORR_MAX_SPS) corr = (int32_t)MOTION_PI_CORR_MAX_SPS;
                else if (corr < -(int32_t)MOTION_PI_CORR_MAX_SPS) corr = -(int32_t)MOTION_PI_CORR_MAX_SPS;

                // A velocidade final é a "ideal" do PC + a "correção" do CASC/PID
                v_final_sps = (int32_t)v_cmd_sps_ideal + corr;

                if (v_final_sps < 0) v_final_sps = 0;
                if (v_final_sps > (int32_t)MOTION_MAX_SPS) v_final_sps = (int32_t)MOTION_MAX_SPS;

                // Anti-windup
                if (!(v_final_sps == 0 || v_final_sps == (int32_t)MOTION_MAX_SPS)) {
                    g_pi_i_accum[axis] = iacc;
                }
            } else {
                v_final_sps = (int32_t)v_cmd_sps_ideal; // Sem PID, usa a velocidade do PC
            }
#else
            v_final_sps = (int32_t)v_cmd_sps_ideal; // Compilação sem PID
#endif

            // --- ETAPA 2c: CÁLCULO DE RAMPA ---
            // A rampa agora é aplicada à velocidade CASC+PID (`v_final_sps`),
            // mas usa a distância restante do *mestre* (`rem_steps_mestre`) para desacelerar.
            uint32_t v_ramped = motion_update_rampa(axis, (uint32_t)v_final_sps, rem_steps_mestre);

            // --- ETAPA 2d: DAR ORDEM AO "OPERÁRIO" (TIM6) ---
            ax->dda_inc_q16 = Q16_DIV_UINT(v_ramped, MOTION_TIM6_HZ);
        }

        // --- ETAPA 3: VERIFICAR FIM DO MOVIMENTO (Lógica movida do TIM6) ---
        // O movimento termina quando o mestre (o mais lento) chega a 100%.
        if (mestre_prog_num >= mestre_prog_den) {
_TIM7_FINISH_MOVE:
            // O mestre terminou. Marca este segmento como concluído.
            uint32_t primask = motion_lock();
            if (motion_try_start_next_locked()) {
                // Próximo segmento na fila, continua rodando
                g_status.state = MOTION_RUNNING;
#if MOTION_DEBUG_FLOW
                printf("[FLOW next_segment started]\r\n");
#endif
            } else {
                // Fila vazia, para tudo
                g_has_active_segment = 0u;
                motion_stop_all_axes_locked(); // Zera dda_inc_q16
                g_status.state = MOTION_DONE;
                motion_send_move_end_response(g_active_frame_id, 0u /* natural_done */);
#if MOTION_DEBUG_FLOW
                printf("[FLOW queue_empty, move_done]\r\n");
#endif
            }
            motion_refresh_status_locked();
            motion_unlock(primask);
            // Finaliza estimador
            est_finalize_if_possible();
        }
    }
}

/* =======================
 * Handlers de protocolo
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

    // Validação: O PC deve enviar velocidades interpoladas
    // (Esta é uma verificação de sanidade, opcional)
    uint32_t s_max = 0;
    uint32_t v_max = 0;
    uint32_t s_all[3] = {req.sx, req.sy, req.sz};
    uint32_t v_all[3] = {req.vx, req.vy, req.vz};

    for(int i=0; i<3; i++) {
        if (s_all[i] > s_max) {
            s_max = s_all[i];
            v_max = v_all[i];
        }
    }

    if (s_max > 0 && v_max > 0) {
        for(int i=0; i<3; i++) {
            // Verifica se a proporção V/S é a mesma do eixo mestre
            // (v_i * s_max) == (v_max * s_i) ?
            uint64_t v_check = (uint64_t)v_all[i] * (uint64_t)s_max;
            uint64_t s_check = (uint64_t)v_max * (uint64_t)s_all[i];

            // Permite uma pequena tolerância
            uint64_t diff = (v_check > s_check) ? (v_check - s_check) : (s_check - v_check);
            uint64_t tolerance = v_check / 100; // Tolerância de 1%

            if (diff > tolerance) {
                 LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "queue_add", "non-interpolated velocities");
                 // Não rejeita, mas avisa. O CASC/PID vai corrigir.
            }
        }
    }

    uint32_t primask = motion_lock();
    proto_result_t push_status = motion_queue_push_locked(&req);
    if (push_status == PROTO_OK) {
        ack_status = MOTION_ACK_OK;
        if (g_status.state == MOTION_IDLE || g_status.state == MOTION_DONE)
            g_status.state = MOTION_QUEUED;
        // Arma o estimador se exatamente um eixo tem passos > 0
        int axes_nonzero = 0;
        int8_t axis_idx = -1;
        uint32_t steps[3] = {req.sx, req.sy, req.sz};
        for (int i = 0; i < 3; ++i) {
            if (steps[i] > 0) { axes_nonzero++; axis_idx = (int8_t)i; }
        }
        if (axes_nonzero == 1 && axis_idx >= 0) {
            int32_t pos0 = (int32_t)g_encoder_position[(uint8_t)axis_idx];
            est_arm_for_axis(axis_idx, pos0);
        } else {
            est_reset(&g_est); // não definido para movimento multi-eixos
        }
        motion_refresh_status_locked();
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

    if (!safety_is_safe()) {
        started = 0u;
    } else if (!g_has_active_segment) {
        if (motion_try_start_next_locked()) {
            g_status.state = MOTION_RUNNING;
            started = 1u;
        }
    } else {
        g_status.state = MOTION_RUNNING; // Permite 'start' para resumir
        started = 1u;
    }

    motion_refresh_status_locked();
    motion_unlock(primask);

    (void)HAL_TIM_Base_Start_IT(&htim6);
    (void)HAL_TIM_Base_Start_IT(&htim7);

    motion_send_start_response(req.frameId, started ? 0u : 1u, g_status.queue_depth);
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "start_move", started ? "running" : "ignored");
}

void motion_on_move_end(const uint8_t *frame, uint32_t len) {
    move_end_req_t req;
    if (move_end_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "move_end", "decode_fail");
        return;
    }

    // [FIX 2] Captura o frame ID *antes* de parar tudo
    uint8_t frame_id_to_notify = g_active_frame_id;

    uint32_t primask = motion_lock();
    motion_stop_all_axes_locked();
    motion_queue_clear_locked();
    g_has_active_segment = 0u;
    g_active_frame_id = 0u;
    g_status.state = MOTION_STOPPING;
    motion_refresh_status_locked();
    motion_unlock(primask);

    // Usa o frame ID da *requisição*, não o 'g_active_frame_id'
    motion_send_move_end_response(req.frameId, 1u /* stopped by host */);

    primask = motion_lock();
    g_status.state = MOTION_IDLE;
    motion_refresh_status_locked();
    motion_unlock(primask);

    LOGT_THIS(LOG_STATE_APPLIED, PROTO_OK, "move_end", "stopped");
}

/* =======================
 * set_origin e encoder_status (Modificado)
 * ======================= */
void motion_on_set_origin(const uint8_t *frame, uint32_t len) {
    set_origin_req_t req;
    if (set_origin_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "set_origin", "decode_fail");
        return;
    }

    if (g_status.state == MOTION_RUNNING) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "set_origin", "busy_running");
        return;
    }

    uint8_t m = req.mask & 0x07u;
    uint32_t primask = motion_lock();
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        if (m & (1u << axis)) {
            // [MODIFICADO] "Zera" a origem do *usuário*
            g_user_origin_enc[axis] = g_encoder_position[axis];

            // Também zera a origem do *segmento CASC* (para status)
            g_encoder_origin[axis] = g_encoder_position[axis];
        }
    }
    motion_unlock(primask);

    set_origin_resp_t resp;
    resp.frameId = req.frameId;
    // Reporta a nova origem (posição absoluta do encoder)
    resp.x0 = (int32_t)(g_user_origin_enc[AXIS_X] & 0xFFFFFFFF);
    resp.y0 = (int32_t)(g_user_origin_enc[AXIS_Y] & 0xFFFFFFFF);
    resp.z0 = (int32_t)(g_user_origin_enc[AXIS_Z] & 0xFFFFFFFF);

    uint8_t raw[16];
    if (set_origin_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    }
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "set_origin", "mask=0x%02X", (unsigned)req.mask);
}

void motion_on_encoder_status(const uint8_t *frame, uint32_t len) {
    encoder_status_req_t req;
    if (encoder_status_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "enc_status", "decode_fail");
        return;
    }

    // [MODIFICADO] Reporta a posição 'abs' como a relativa ao 'set_origin'
    int32_t rel_user[3];
    int32_t abs_enc[3];

    uint32_t primask = motion_lock();
    motion_refresh_status_locked(); // Atualiza g_status.pidErrX/Y/Z
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        int64_t a = g_encoder_position[axis];
        if (a > INT32_MAX) a = INT32_MAX; else if (a < INT32_MIN) a = INT32_MIN;
        abs_enc[axis] = (int32_t)a;

        // Posição relativa à "origem" do usuário
        int64_t r = (int64_t)abs_enc[axis] - (int64_t)g_user_origin_enc[axis];
        if (r > INT32_MAX) r = INT32_MAX; else if (r < INT32_MIN) r = INT32_MIN;
        rel_user[axis] = (int32_t)r;
    }
    motion_unlock(primask);

    encoder_status_resp_t resp;
    resp.frameId = req.frameId;
    resp.pidErrX = (uint8_t)g_status.pidErrX;
    resp.pidErrY = (uint8_t)g_status.pidErrY;
    resp.pidErrZ = (uint8_t)g_status.pidErrZ;
    resp.delta = 0;
    resp.absX = rel_user[AXIS_X]; // Reporta a Posição Relativa do Usuário
    resp.absY = rel_user[AXIS_Y];
    resp.absZ = rel_user[AXIS_Z];

    uint8_t raw[20];
    if (encoder_status_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    }
}

void motion_on_motion_estimate(const uint8_t *frame, uint32_t len) {
    motion_estimate_req_t req;
    if (motion_estimate_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "motion_est", "decode_fail");
        return;
    }
    motion_estimate_resp_t resp;
    resp.frameId = req.frameId;
    // Se não validado, responde zeros (requisito do usuário)
    resp.avgAccel  = g_est.valid ? g_est.avg_accel_sps2 : 0;
    resp.avgCruise = g_est.valid ? g_est.avg_cruise_sps : 0;
    resp.avgDecel  = g_est.valid ? g_est.avg_decel_sps2 : 0;
    uint8_t raw[16];
    if (motion_estimate_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    }
}

void motion_on_diag_ctrl(const uint8_t *frame, uint32_t len) {
    diag_ctrl_req_t req;
    if (diag_ctrl_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "diag_ctrl", "decode_fail");
        return;
    }
    // bit0 controla o SPD via SWO
    g_diag_swo_spd_enable = (req.flags & 0x01) ? 1u : 0u;
    diag_ctrl_resp_t resp;
    resp.frameId = req.frameId;
    resp.flags = (uint8_t)(g_diag_swo_spd_enable & 0x01);
    uint8_t raw[5];
    if (diag_ctrl_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    }
}

void motion_on_set_enc_ppr(const uint8_t *frame, uint32_t len) {
    set_enc_ppr_req_t req;
    if (set_enc_ppr_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "set_enc_ppr", "decode_fail");
        return;
    }
    // Não permitir alterar durante movimento
    if (g_status.state == MOTION_RUNNING) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "set_enc_ppr", "busy_running");
        return;
    }
    uint8_t a = req.axis;
    if (a > 2) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "set_enc_ppr", "bad_axis");
        return;
    }
    uint32_t ppr = (req.ppr == 0u) ? 1u : req.ppr; // evita zero
    uint32_t primask = motion_lock();
    g_enc_counts_per_rev[a] = ppr;
    motion_unlock(primask);

    set_enc_ppr_resp_t resp;
    resp.frameId = req.frameId;
    resp.axis = a;
    resp.ppr = ppr;
    uint8_t raw[9];
    if (set_enc_ppr_resp_encoder(&resp, raw, sizeof raw) == PROTO_OK) {
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    }
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "set_enc_ppr", "axis=%u ppr=%lu", (unsigned)a, (unsigned long)ppr);
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

void motion_on_model_run(const uint8_t *frame, uint32_t len) {
    model_run_req_t req;
    if (model_run_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "model_run", "decode_fail");
        return;
    }
    if (g_status.state == MOTION_RUNNING || g_has_active_segment || g_raw.active) {
        model_run_resp_t r = { req.frameId, 1 };
        uint8_t raw[5]; if (model_run_resp_encoder(&r, raw, sizeof raw) == PROTO_OK) (void)app_resp_push(raw, (uint32_t)sizeof raw);
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "model_run", "busy");
        return;
    }
    uint8_t a = req.axis;
    if (a > 2 || req.turns == 0 || req.turns > 20 || req.freq_sps == 0) {
        model_run_resp_t r = { req.frameId, 2 };
        uint8_t raw[5]; if (model_run_resp_encoder(&r, raw, sizeof raw) == PROTO_OK) (void)app_resp_push(raw, (uint32_t)sizeof raw);
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "model_run", "bad_args");
        return;
    }
    uint32_t primask = motion_lock();
    raw_reset();
    g_raw.axis = a;
    g_raw.dir = req.dir ? 1u : 0u;
    g_raw.freq_sps = req.freq_sps;
    g_raw.turns = req.turns;
    g_raw.ppr = g_enc_counts_per_rev[a];
    g_raw.accum_q16 = 0u;
    g_raw.inc_q16 = Q16_DIV_UINT(g_raw.freq_sps, MOTION_TIM6_HZ);
    motion_hw_step_low(a);
    motion_hw_set_dir(a, g_raw.dir);
    motion_hw_enable(a, 1u);
    g_raw.enc_start = (int32_t)g_encoder_position[a];
    g_raw.active = 1u;
    g_status.state = MOTION_RUNNING;
    // Armazena frameId ativo para enviar MOVE_END ao concluir o RAW
    g_active_frame_id = req.frameId;
    motion_unlock(primask);

    // Estimador só para este eixo
    est_arm_for_axis((int8_t)a, (int32_t)g_encoder_position[a]);

    model_run_resp_t r = { req.frameId, 0 };
    uint8_t raw[5]; if (model_run_resp_encoder(&r, raw, sizeof raw) == PROTO_OK) (void)app_resp_push(raw, (uint32_t)sizeof raw);
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "model_run", "axis=%u freq=%lu turns=%u", (unsigned)a, (unsigned long)req.freq_sps, (unsigned)req.turns);
}

/* =====================================================================
 * DEMO DE MOVIMENTO (Sem mudanças)
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
    req.dirMask = 0x07;       /* XYZ forward */
    // As velocidades agora são steps/s
    req.vx = 10000; req.vy = 8000; req.vz = 6000;
    req.sx = 20000; req.sy = 16000; req.sz = 12000;

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

            ax->total_steps     = 0xFFFFFFFFu;
            ax->emitted_steps   = 0u;

            uint16_t vtab         = g_demo_speed_table[g_demo_speed_idx & 0x3u];
            ax->velocity_per_tick = vtab * 1000u; // Converte k_sps para sps
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
            motion_hw_set_dir(axis, 1u);   /* forward */
            motion_hw_enable(axis, 1u);

            // Zera origens e estado PID
            g_encoder_origin[axis] = g_encoder_position[axis];
            g_casc_total_steps_s32[axis] = 0;
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

/* =======================
 * Handlers de Interrupção
 * ======================= */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (!htim) return;

    if (htim->Instance == TIM6) {
        // --- "OPERÁRIO" RÁPIDO @ 50kHz ---
        motion_on_tim6_tick();

    } else if (htim->Instance == TIM7) {
        // --- "CHEFE" ESTRATEGISTA @ 1kHz ---
        motion_on_tim7_tick();
    }
}

/* =======================
 * Parada de Emergência (FIX 2 Aplicado)
 * ======================= */
void motion_emergency_stop(void)
{
    uint32_t primask = motion_lock();

    // [FIX 2] Captura o estado ANTES de zerar
    uint8_t was_active = g_has_active_segment;
    uint8_t frame_id_to_notify = g_active_frame_id;

    g_demo_continuous = 0u;
    motion_stop_all_axes_locked();
    motion_queue_clear_locked();
    g_has_active_segment = 0u;
    g_active_frame_id = 0u; // Zera aqui

    g_status.state = MOTION_STOPPING;
    motion_refresh_status_locked();
    motion_unlock(primask);

    primask = motion_lock();
    g_status.state = MOTION_IDLE;
    motion_refresh_status_locked();
    motion_unlock(primask);

    // [FIX 2] Envia resposta se estava ativo,
    // independentemente do valor do frame_id (0 é válido)
    if (was_active) {
        motion_send_move_end_response(frame_id_to_notify, 2u /* emergency */);
    }
}

uint8_t motion_demo_is_active(void)
{
    return g_demo_continuous ? 1u : 0u;
}

void motion_demo_cycle_speed(void)
{
    g_demo_speed_idx = (uint8_t)((g_demo_speed_idx + 1u) & 0x3u);

    if (g_demo_continuous) {
        uint16_t vtab = g_demo_speed_table[g_demo_speed_idx & 0x3u];
        uint32_t v_sps = ((uint32_t)vtab) * 1000u;
        if (v_sps > MOTION_MAX_SPS) v_sps = MOTION_MAX_SPS;

        uint32_t primask = motion_lock();
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            g_axis_state[axis].velocity_per_tick = v_sps;
            g_axis_state[axis].v_target_sps      = v_sps;
        }
        motion_unlock(primask);
    }
}
