#include "app.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "stm32l4xx.h"  /* ITM/DBGMCU presence for optional binary SWO */
#include "lptim.h"      /* hlptim1 for Y encoder (LPTIM1) */

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
#include "Protocol/Requests/set_microsteps_axes_request.h"

LOG_SVC_DEFINE(LOG_SVC_MOTION, "motion");


#define MOTION_AXIS_COUNT        3u
// Aumenta a capacidade da fila de movimentos para reduzir "queue_full"
// Cada entrada consome ~42 bytes (move_queue_add_req_t); 256 => ~10.8 KB
#define MOTION_QUEUE_CAPACITY    256u

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
#define MOTION_DEBUG_FLOW              0
#endif
#ifndef MOTION_DEBUG_TIM6_PRINTS
#define MOTION_DEBUG_TIM6_PRINTS       0
#endif
#ifndef MOTION_DEBUG_STEP_DECIM
#define MOTION_DEBUG_STEP_DECIM        500u
#endif

/* =======================
 *  Diagnóstico de travamentos/bloqueios (desligado por padrão)
 *  - Emite snapshots compactos em intervalos e quando detecta "stall"
 *  - Completamente desacoplado e controlado por macros
 * ======================= */
#ifndef MOTION_DIAG_ENABLE
#define MOTION_DIAG_ENABLE              1u   /* 0=off, 1=on (ativado p/ debug) */
#endif
#ifndef MOTION_DIAG_INTERVAL_MS
#define MOTION_DIAG_INTERVAL_MS         200u /* período de log periódico */
#endif
#ifndef MOTION_DIAG_STALL_MS
#define MOTION_DIAG_STALL_MS            150u /* ms sem STEP considerado stall */
#endif
#ifndef MOTION_DIAG_VERBOSE
#define MOTION_DIAG_VERBOSE             1u   /* 0=leve, 1=inclui p/i/d/corr/v_adj no DIAG */
#endif
/* Print pontual de condição suspeita (rem>0 && v_cmd==0 && queue>0) */
#ifndef MOTION_BUGCHECK_ENABLE
#define MOTION_BUGCHECK_ENABLE          1u
#endif

/* Impressão periódica de encoder a cada N pulsos (selecionável por eixo) */
#ifndef MOTION_ENC_PRINT_EVERY_N_ENABLE
#define MOTION_ENC_PRINT_EVERY_N_ENABLE 1u
#endif
#ifndef MOTION_ENC_PRINT_N
#define MOTION_ENC_PRINT_N              10u
#endif
#ifndef MOTION_ENC_PRINT_MASK
#define MOTION_ENC_PRINT_MASK           0x03u /* bits X=1,Y=2,Z=4; X+Y */
#endif

// Emissão de CSV/telemetria
#ifndef MOTION_CSV_AT_STEP
#define MOTION_CSV_AT_STEP             1u
#endif
#ifndef MOTION_CSV_STEP_DECIM
#define MOTION_CSV_STEP_DECIM          100u   /* imprime a cada N passos após o primeiro */
#endif

// Amostragem periódica do CSV (em ms). Ajuste conforme necessário.
#ifndef MOTION_CSV_SAMPLE_MS
#define MOTION_CSV_SAMPLE_MS            1u
#endif

// Produção rígida a cada 1 ms no TIM6 (50 kHz) com ring buffer (SPSC)
#ifndef MOTION_CSV_PRODUCE_IN_TIM6
#define MOTION_CSV_PRODUCE_IN_TIM6      0u   /* amostra e imprime no TIM7, sempre após atualizar encoders */
#endif
#ifndef MOTION_CSV_TEXT_ENABLE
#define MOTION_CSV_TEXT_ENABLE           0u   /* 0=desliga printf textual; 1=liga printf textual */
#endif
#ifndef MOTION_CSV_RING_CAP
#define MOTION_CSV_RING_CAP             512u /* 16 bytes × 512 × 3 eixos ≈ 24 KB */
#endif
#ifndef MOTION_CSV_DRAIN_PER_TICK
#define MOTION_CSV_DRAIN_PER_TICK       128u /* aumenta drenagem para aliviar backlog rapidamente */
#endif

/* =======================
 *  Compatibilidade "progress" (interactive_old_sim)
 *  - Seleciona mestre por menor progresso (emitted/total)
 *  - Rampa guiada pelo restante do mestre (fila inclusa)
 *  - Conclusão global quando restante do mestre chega a zero
 * ======================= */
#ifndef MOTION_PROGRESS_MODE
#define MOTION_PROGRESS_MODE 1
#endif

/* =======================
 *  Throttle por erro (modo progress)
 *  - Reduz feed dos eixos não-mestres conforme |erro| cresce
 *  - Fórmula (per-mille):
 *      scale = 1 - (1 - min_frac) * min(|err|/threshold, 1)
 *      v_cmd <- v_cmd * scale
 *  - Parâmetros alinhados ao interactive_old_sim (threshold=200, min_frac=0.25)
 * ======================= */
#ifndef MOTION_ERR_THROTTLE_ENABLE
#define MOTION_ERR_THROTTLE_ENABLE 1
#endif
#ifndef MOTION_ERR_THROTTLE_THRESHOLD
#define MOTION_ERR_THROTTLE_THRESHOLD 200u /* steps */
#endif
#ifndef MOTION_ERR_THROTTLE_MIN_PERMILLE
#define MOTION_ERR_THROTTLE_MIN_PERMILLE 250u /* 0.25 => 25% do v_cmd */
#endif

// Opções de formato/rápidez (padrões conservadores p/ compatibilidade)
#ifndef MOTION_CSV_INCLUDE_ID
#define MOTION_CSV_INCLUDE_ID           0u   /* adiciona um id incremental por eixo */
#endif
#ifndef MOTION_CSV_TIME_IN_TICKS
#define MOTION_CSV_TIME_IN_TICKS        0u   /* 0=imprime tempo em ms (compat), 1=ticks TIM6 (mais barato) */
#endif
#ifndef MOTION_CSV_BIN_ITM
#define MOTION_CSV_BIN_ITM              0u   /* 1=encaminha frame binário via ITM (menos CPU/bytes) */
#endif

// Conversão de tempo baseada no clock do TIM6
// Ex.: 50 kHz => 50 ticks por milissegundo
#define T6_TICKS_PER_MS                 (MOTION_TIM6_HZ / 1000u)

// Contador monotônico de ticks do TIM6 (incrementa a cada update)
static volatile uint32_t g_tim6_ticks = 0u;

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
static uint8_t  g_queue_head  = 0u;
static uint8_t  g_queue_tail  = 0u;
static uint16_t g_queue_count = 0u;  /* suporta até 256 entradas */
static uint8_t g_active_frame_id = 0u; 

/* Soma dos passos restantes na FILA (exclui segmento ativo) por eixo.
 * Mantida em O(1) nos push/pop para permitir consultas rápidas no TIM6. */
static uint32_t g_queue_rem_steps[MOTION_AXIS_COUNT];

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
// Ganhos inteiros (kp/ki/kd) operam em formato Q8 (>> 8). O emissor deve
// fornecer os valores já escalados. Não há multiplicação interna por 256.
// FUTURO: avaliar request alternativo com ganhos em 32 bits (ex.: escala 10^4
// como no LED em centi-Hz) ou IEEE-754, preservando o MOVE_QUEUE_ADD atual
// para compatibilidade retroativa.
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

/* Encoders por rotação (fornecido / medido):
 * - Preencher X/Y/Z com os pulsos reais de cada encoder por volta.
 *   Exemplo: X/Z = 40000, Y = 5000 (ou 2500, se você medir isso na bancada).
 */
static const uint32_t ENC_COUNTS_PER_REV[3] = { 40000u, 5000u, 40000u}; // X,Y,Z

/* Microstep real do TMC em cada eixo (1,2,4,...,256) */
static volatile uint16_t g_microstep_factor[MOTION_AXIS_COUNT] = {
    MICROSTEP_FACTOR, MICROSTEP_FACTOR, MICROSTEP_FACTOR
};

/* Quantos "steps físicos" existem em 1 volta de eixo (por eixo):
 * 400 passos base do motor (0,9°) × microstep configurado no TMC daquele eixo.
 */
static inline uint32_t control_steps_per_rev_axis(uint8_t axis)
{
    if (axis >= MOTION_AXIS_COUNT) axis = 0;
    return STEPS_PER_REV_BASE * (uint32_t)g_microstep_factor[axis];
}

/* Mantemos o alias para DDA, se em algum lugar ainda usar esse nome. */
static inline uint32_t dda_steps_per_rev_axis(uint8_t axis)
{
    return control_steps_per_rev_axis(axis);
}

/* Converte posição relativa do encoder -> steps físicos do eixo,
 * na mesma unidade de ax->target_steps / emitted_steps.
 */
static inline int32_t motion_actual_steps_from_encoder(uint8_t axis)
{
    if (axis >= MOTION_AXIS_COUNT) axis = 0;

    /* posição relativa do encoder (contagens) */
    int64_t enc_rel = g_encoder_position[axis] - g_encoder_origin[axis];

    /* converte contagens -> steps físicos:
     * actual_steps ≈ enc_rel * (400 * microstep_axis) / ENC_COUNTS_PER_REV[axis]
     */
    int64_t num = enc_rel * (int64_t)control_steps_per_rev_axis(axis);

    if (ENC_COUNTS_PER_REV[axis] == 0u) {
        return 0;
    }

    int64_t q = num / (int64_t)ENC_COUNTS_PER_REV[axis];

    if (q > (int64_t)INT32_MAX) q = (int64_t)INT32_MAX;
    else if (q < (int64_t)INT32_MIN) q = (int64_t)INT32_MIN;

    return (int32_t)q;
}

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

#if MOTION_DIAG_ENABLE
/* Buffers de diagnóstico (capturam valores relevantes do controle) */
static uint32_t g_dbg_v_cmd_sps[MOTION_AXIS_COUNT];
static uint32_t g_dbg_scale_pm[MOTION_AXIS_COUNT];
static int32_t  g_dbg_err[MOTION_AXIS_COUNT];
static int32_t  g_dbg_pterm[MOTION_AXIS_COUNT];
static int32_t  g_dbg_iterm[MOTION_AXIS_COUNT];
static int32_t  g_dbg_dterm[MOTION_AXIS_COUNT];
static int32_t  g_dbg_corr[MOTION_AXIS_COUNT];
static int32_t  g_dbg_v_adj[MOTION_AXIS_COUNT];
static int8_t   g_dbg_master_axis;
/* Stall detection */
static uint32_t g_diag_last_emit[MOTION_AXIS_COUNT];
static uint16_t g_diag_no_step_ms[MOTION_AXIS_COUNT];
static uint8_t  g_diag_stall_reported[MOTION_AXIS_COUNT];
static uint32_t g_diag_next_print_ms;
/* Bugcheck pontual */
static uint8_t  g_bugcheck_reported[MOTION_AXIS_COUNT];
#endif

// Telemetria CSV: tempo(ms) desde início da variação do encoder,
// posição relativa do encoder e total de passos emitidos desde o início
static volatile uint8_t  g_csv_active[MOTION_AXIS_COUNT];
static volatile uint32_t g_csv_t0_ms[MOTION_AXIS_COUNT];
static volatile uint32_t g_csv_stepcount[MOTION_AXIS_COUNT];
static volatile uint32_t g_csv_next_ms[MOTION_AXIS_COUNT];
static volatile uint8_t  g_csv_armed[MOTION_AXIS_COUNT]; // 1=armado no start_move; ativa ao primeiro STEP

// Referência de tempo baseada em ticks do TIM6 (mais estável que SysTick em prints SWO)
static volatile uint32_t g_csv_t0_t6[MOTION_AXIS_COUNT];
static volatile uint32_t g_csv_next_t6[MOTION_AXIS_COUNT];
static volatile uint32_t g_csv_seq[MOTION_AXIS_COUNT]; /* id incremental por eixo */

#if MOTION_CSV_PRODUCE_IN_TIM6
typedef struct {
    uint32_t id;      /* id incremental */
    uint32_t t6;      /* tempo relativo em ticks do TIM6 */
    int32_t  rel;     /* encoder relativo */
    uint32_t steps;   /* total de steps emitidos */
} motion_csv_sample_t;

static volatile motion_csv_sample_t g_csv_ring[MOTION_AXIS_COUNT][MOTION_CSV_RING_CAP];
static volatile uint16_t g_csv_rhead[MOTION_AXIS_COUNT];
static volatile uint16_t g_csv_rtail[MOTION_AXIS_COUNT];
static volatile uint32_t g_csv_next_prod_t6; /* agenda global: a cada 50 ticks (1ms) */

static inline void csv_ring_reset(void)
{
    for (uint8_t a = 0; a < MOTION_AXIS_COUNT; ++a) {
        g_csv_rhead[a] = 0u;
        g_csv_rtail[a] = 0u;
    }
    g_csv_next_prod_t6 = 0u;
}

static inline void csv_ring_push(uint8_t axis, uint32_t id, uint32_t t6, int32_t rel, uint32_t steps)
{
    uint16_t h = g_csv_rhead[axis];
    uint16_t n = (uint16_t)((h + 1u) % MOTION_CSV_RING_CAP);
    if (n == g_csv_rtail[axis]) {
        /* ring cheio: descarta o mais antigo (avança tail) para não bloquear produtor */
        g_csv_rtail[axis] = (uint16_t)((g_csv_rtail[axis] + 1u) % MOTION_CSV_RING_CAP);
    }
    g_csv_ring[axis][h].id    = id;
    g_csv_ring[axis][h].t6    = t6;
    g_csv_ring[axis][h].rel   = rel;
    g_csv_ring[axis][h].steps = steps;
    g_csv_rhead[axis] = n;
}

static inline int csv_ring_pop(uint8_t axis, motion_csv_sample_t *out)
{
    uint16_t t = g_csv_rtail[axis];
    if (t == g_csv_rhead[axis]) return 0;
    if (out) {
        * (motion_csv_sample_t*)out = g_csv_ring[axis][t];
    }
    g_csv_rtail[axis] = (uint16_t)((t + 1u) % MOTION_CSV_RING_CAP);
    return 1;
}
#endif /* MOTION_CSV_PRODUCE_IN_TIM6 */

// Forward declarations for lock helpers used below
static inline uint32_t motion_lock(void);
static inline void     motion_unlock(uint32_t primask);

// Checagem local de habilitação do SWO/ITM (porta 0)
static inline int motion_swo_enabled(void)
{
    return ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) &&
            (DBGMCU->CR & DBGMCU_CR_TRACE_IOEN) &&
            (ITM->TCR & ITM_TCR_ITMENA_Msk) &&
            (ITM->TER & (1UL << 0)));
}

static inline void motion_csv_print(uint8_t axis, uint32_t id, uint32_t t_val, int32_t rel, uint32_t steps)
{
#if MOTION_CSV_BIN_ITM
    // Frame binário compacto: [tag(1) axis(1) id(4) t(4) rel(4) steps(4)]
    // tag='E' (0x45) para encoder; axis=0..2
    if (motion_swo_enabled()) {
        ITM_SendChar('E');
        ITM_SendChar((uint32_t)axis);
        uint32_t v;
        v = id;      for (int i=0;i<4;i++) ITM_SendChar((v >> (i*8)) & 0xFF);
        v = t_val;   for (int i=0;i<4;i++) ITM_SendChar((v >> (i*8)) & 0xFF);
        v = (uint32_t)rel; for (int i=0;i<4;i++) ITM_SendChar((v >> (i*8)) & 0xFF);
        v = steps;   for (int i=0;i<4;i++) ITM_SendChar((v >> (i*8)) & 0xFF);
        ITM_SendChar('\n');
        return;
    }
#endif

#if MOTION_CSV_TEXT_ENABLE
#if MOTION_CSV_INCLUDE_ID
    // Texto: axis,id,time,rel,steps (rel como int32 para evitar wrap sem sinal)
    printf("%u,%lu,%lu,%d,%lu\r\n",
           (unsigned)axis,
           (unsigned long)id,
           (unsigned long)t_val,
           (int)rel,
           (unsigned long)steps);
#else
    // Compatibilidade: axis,time,rel,steps
    printf("%u,%lu,%d,%lu\r\n",
           (unsigned)axis,
           (unsigned long)t_val,
           (int)rel,
           (unsigned long)steps);
#endif
#endif /* MOTION_CSV_TEXT_ENABLE */
}

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

/* Forward decl. para função usada pela seleção de mestre (progress) */
static uint32_t motion_remaining_steps_total_for_axis(uint8_t axis);

/* =======================
 *  Seleção de Mestre (modo progress)
 * ======================= */
#if MOTION_PROGRESS_MODE
static inline int8_t motion_select_master_axis_progress(void)
{
    int8_t master = -1;
    uint32_t m_num = 0u, m_den = 1u;

    /* 1) Preferir o menor progresso ENTRE eixos que ainda têm trabalho (ativo+fila)
          e que participam do segmento atual (total_steps>0). */
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        /* total restante (ativo + fila) em O(1) */
        const motion_axis_state_t *ax = &g_axis_state[axis];
        uint32_t active = (ax->total_steps > ax->emitted_steps)
                          ? (ax->total_steps - ax->emitted_steps) : 0u;
        uint32_t rem_total = active + g_queue_rem_steps[axis];
        if (rem_total == 0u) continue; /* não escolher quem já acabou de vez */
        if (ax->total_steps == 0u) continue; /* não participa do segmento atual */

        uint32_t num = ax->emitted_steps; /* progresso acumulado */
        uint32_t den = ax->total_steps;   /* tamanho do segmento */
        if (master < 0) { master = (int8_t)axis; m_num = num; m_den = den; }
        else if ((uint64_t)num * (uint64_t)m_den < (uint64_t)m_num * (uint64_t)den) {
            master = (int8_t)axis; m_num = num; m_den = den;
        }
    }

    /* 2) Fallback: se ninguém do segmento atual tem rem_total>0, escolha
          o eixo com MAIOR rem_total geral (apontando para o próximo da fila). */
    if (master < 0) {
        uint32_t best_rem = 0u;
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            const motion_axis_state_t *ax = &g_axis_state[axis];
            uint32_t active = (ax->total_steps > ax->emitted_steps)
                              ? (ax->total_steps - ax->emitted_steps) : 0u;
            uint32_t rem_total = active + g_queue_rem_steps[axis];
            if (rem_total > best_rem) { best_rem = rem_total; master = (int8_t)axis; }
        }
    }

    return master;
}
#endif /* MOTION_PROGRESS_MODE */

#if MOTION_DIAG_ENABLE
/* =======================
 *  Diagnóstico: snapshots e detecção de stall
 * ======================= */
static inline void motion_diag_tick(void)
{
    /* Atualiza contadores de "sem passo" por eixo (1ms por tick TIM7) */
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        const motion_axis_state_t *ax = &g_axis_state[axis];
        uint8_t expected = (g_status.state == MOTION_RUNNING) && g_has_active_segment && (ax->total_steps > ax->emitted_steps);
        if (!expected) {
            g_diag_no_step_ms[axis] = 0u;
            g_diag_last_emit[axis]  = ax->emitted_steps;
            g_diag_stall_reported[axis] = 0u;
            continue;
        }
        if (ax->emitted_steps == g_diag_last_emit[axis]) {
            if (g_diag_no_step_ms[axis] < 0xFFFFu) g_diag_no_step_ms[axis]++;
        } else {
            g_diag_no_step_ms[axis] = 0u;
            g_diag_last_emit[axis]  = ax->emitted_steps;
            g_diag_stall_reported[axis] = 0u;
        }
    }

    /* Evento de STALL: nenhum passo por MOTION_DIAG_STALL_MS enquanto ainda há trabalho */
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        const motion_axis_state_t *ax = &g_axis_state[axis];
        uint32_t active_rem = (ax->total_steps > ax->emitted_steps) ? (ax->total_steps - ax->emitted_steps) : 0u;
        if (active_rem == 0u) continue;
        if (g_diag_no_step_ms[axis] >= (uint16_t)MOTION_DIAG_STALL_MS && !g_diag_stall_reported[axis]) {
            /* Snapshot compacto com variáveis que podem bloquear geração de passos */
            int32_t enc_rel = (int32_t)(g_encoder_position[axis] - g_encoder_origin[axis]);
            int32_t actual  = motion_actual_steps_from_encoder(axis);
            printf("[STALL axis=%u] rem=%lu v_cmd=%lu v_act=%lu incQ16=%lu high=%u low=%u en_set=%u dir_set=%u err=%ld p=%ld i=%ld d=%ld corr=%ld v_adj=%ld ms=%u qdepth=%u actv=%u\r\n",
                   (unsigned)axis,
                   (unsigned long)active_rem,
                   (unsigned long)g_dbg_v_cmd_sps[axis],
                   (unsigned long)ax->v_actual_sps,
                   (unsigned long)ax->dda_inc_q16,
                   (unsigned)ax->step_high,
                   (unsigned)ax->step_low,
                   (unsigned)ax->en_settle_ticks,
                   (unsigned)ax->dir_settle_ticks,
                   (long)g_dbg_err[axis],
                   (long)g_dbg_pterm[axis],
                   (long)g_dbg_iterm[axis],
                   (long)g_dbg_dterm[axis],
                   (long)g_dbg_corr[axis],
                   (long)g_dbg_v_adj[axis],
                   (unsigned)g_diag_no_step_ms[axis],
                   (unsigned)g_status.queue_depth,
                   (unsigned)g_has_active_segment);
            printf("[STALL axis=%u] enc_rel=%ld actual_steps=%ld ms_factor=%u enc_cpr=%lu scale_pm=%lu master=%d\r\n",
                   (unsigned)axis,
                   (long)enc_rel,
                   (long)actual,
                   (unsigned)g_microstep_factor[axis],
                   (unsigned long)ENC_COUNTS_PER_REV[axis],
                   (unsigned long)g_dbg_scale_pm[axis],
                   (int)g_dbg_master_axis);
            g_diag_stall_reported[axis] = 1u; /* evita flood até próximo passo */
        }
    }

    /* Bugcheck pontual: rem>0 && v_cmd==0 && fila>0 */
#if MOTION_BUGCHECK_ENABLE
    {
        uint8_t active_mask = 0u;
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            const motion_axis_state_t *ax = &g_axis_state[axis];
            if (ax->total_steps > ax->emitted_steps) active_mask |= (1u << axis);
        }
        uint16_t qdepth = g_queue_count; /* somente fila (exclui ativo) */
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            const motion_axis_state_t *ax = &g_axis_state[axis];
            uint32_t active_rem = (ax->total_steps > ax->emitted_steps) ? (ax->total_steps - ax->emitted_steps) : 0u;
            uint32_t rem_total = active_rem + g_queue_rem_steps[axis];
            if (rem_total > 0u && g_dbg_v_cmd_sps[axis] == 0u && qdepth > 0u) {
                if (!g_bugcheck_reported[axis]) {
                    g_bugcheck_reported[axis] = 1u;
                    printf("[BUG axis=%u] rem=%lu v_cmd=0 qdepth=%u mask=0x%02X\r\n",
                           (unsigned)axis,
                           (unsigned long)rem_total,
                           (unsigned)qdepth,
                           (unsigned)active_mask);
                }
            } else {
                g_bugcheck_reported[axis] = 0u; /* libera nova emissão quando sair da condição */
            }
        }
    }
#endif

    /* Log periódico opcional */
    uint32_t now_ms = g_tim6_ticks / (uint32_t)T6_TICKS_PER_MS;
    if ((int32_t)(now_ms - g_diag_next_print_ms) >= 0) {
        g_diag_next_print_ms = now_ms + (uint32_t)MOTION_DIAG_INTERVAL_MS;
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            const motion_axis_state_t *ax = &g_axis_state[axis];
            uint32_t active_rem = (ax->total_steps > ax->emitted_steps) ? (ax->total_steps - ax->emitted_steps) : 0u;
            printf("[DIAG axis=%u] state=%u actv=%u rem=%lu v_cmd=%lu v_act=%lu incQ16=%lu hi=%u lo=%u en=%u dir=%u err=%ld scale_pm=%lu\r\n",
                   (unsigned)axis,
                   (unsigned)g_status.state,
                   (unsigned)g_has_active_segment,
                   (unsigned long)active_rem,
                   (unsigned long)g_dbg_v_cmd_sps[axis],
                   (unsigned long)ax->v_actual_sps,
                   (unsigned long)ax->dda_inc_q16,
                   (unsigned)ax->step_high,
                   (unsigned)ax->step_low,
                   (unsigned)ax->en_settle_ticks,
                   (unsigned)ax->dir_settle_ticks,
                   (long)g_dbg_err[axis],
                   (unsigned long)g_dbg_scale_pm[axis]);
            /* Linha detalhada opcional (p/i/d/corr/v_adj) */
#if MOTION_DIAG_VERBOSE
            printf("[DIAG axis=%u] p=%ld i=%ld d=%ld corr=%ld v_adj=%ld\r\n",
                   (unsigned)axis,
                   (long)g_dbg_pterm[axis],
                   (long)g_dbg_iterm[axis],
                   (long)g_dbg_dterm[axis],
                   (long)g_dbg_corr[axis],
                   (long)g_dbg_v_adj[axis]);
#endif
        }
    }
}
#endif /* MOTION_DIAG_ENABLE */
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
    for (uint16_t i = 0; i < g_queue_count; ++i) {
        uint8_t idxq = (uint8_t)(((uint16_t)g_queue_head + i) % MOTION_QUEUE_CAPACITY);
        const move_queue_add_req_t *q = &g_queue[idxq].req;
        rem += motion_total_for_axis(q, axis);
    }
    return rem;
}

/* =======================
 *  Status e fila
 * ======================= */
static void motion_refresh_status_locked(void) {
    /* depth reportado cabe em 8 bits; faz clamp para 255 */
    uint32_t depth32 = (uint32_t)g_queue_count + (uint32_t)(g_has_active_segment ? 1u : 0u);
    if (depth32 > 255u) depth32 = 255u;
    g_status.queue_depth = (uint8_t)depth32;

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

        /* Erro em UNIDADES DE STEPS FÍSICOS
         * desired_steps = passos emitidos no segmento (target_steps)
         * actual_steps  = encoder convertido para steps físicos do eixo
         */
        int32_t actual_steps  = motion_actual_steps_from_encoder(axis);
        int32_t desired_steps = (int32_t)ax->target_steps;
        int32_t err           = desired_steps - actual_steps;
        int8_t  err8          = motion_clamp_error(err);

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
    for (uint8_t a = 0; a < MOTION_AXIS_COUNT; ++a) g_queue_rem_steps[a] = 0u;
}

static proto_result_t motion_queue_push_locked(const move_queue_add_req_t *req) {
    if (g_queue_count >= MOTION_QUEUE_CAPACITY)
        return PROTO_ERR_RANGE;
    g_queue[g_queue_tail].req = *req;
    g_queue_tail = (uint8_t)((g_queue_tail + 1u) % MOTION_QUEUE_CAPACITY);
    ++g_queue_count;
    /* Atualiza soma de passos restantes na FILA (por eixo) */
    for (uint8_t a = 0; a < MOTION_AXIS_COUNT; ++a) {
        g_queue_rem_steps[a] += motion_total_for_axis(req, a);
    }
    motion_refresh_status_locked();
    return PROTO_OK;
}

static int motion_queue_pop_locked(move_queue_add_req_t *out) {
    if (g_queue_count == 0u) return 0;
    move_queue_add_req_t tmp = g_queue[g_queue_head].req;
    if (out) *out = tmp;
    g_queue_head = (uint8_t)((g_queue_head + 1u) % MOTION_QUEUE_CAPACITY);
    --g_queue_count;
    /* Remove da soma de fila aquilo que saiu da fila */
    for (uint8_t a = 0; a < MOTION_AXIS_COUNT; ++a) {
        uint32_t s = motion_total_for_axis(&tmp, a);
        if (g_queue_rem_steps[a] >= s) g_queue_rem_steps[a] -= s;
        else g_queue_rem_steps[a] = 0u;
    }
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
    printf("[FLOW pop_next id=%u remaining=%u]\r\n", (unsigned)g_active_frame_id, (unsigned)g_queue_count);
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
            // Prints antigos removidos; CSV emitido no TIM7
#endif
        } else {
            int32_t delta = (int32_t)(now - g_encoder_last_raw[axis]);
            g_encoder_last_raw[axis] = now;
            g_encoder_position[axis] += delta;
            g_encoder_delta_tick[axis] = delta;
#if MOTION_DEBUG_ENCODERS
            // Prints antigos removidos; CSV emitido no TIM7
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
#if MOTION_DIAG_ENABLE
    memset(g_dbg_v_cmd_sps, 0, sizeof g_dbg_v_cmd_sps);
    memset(g_dbg_scale_pm, 0, sizeof g_dbg_scale_pm);
    memset(g_dbg_err, 0, sizeof g_dbg_err);
    memset(g_dbg_pterm, 0, sizeof g_dbg_pterm);
    memset(g_dbg_iterm, 0, sizeof g_dbg_iterm);
    memset(g_dbg_dterm, 0, sizeof g_dbg_dterm);
    memset(g_dbg_corr, 0, sizeof g_dbg_corr);
    memset(g_dbg_v_adj, 0, sizeof g_dbg_v_adj);
    g_dbg_master_axis = -1;
    memset(g_diag_last_emit, 0, sizeof g_diag_last_emit);
    memset(g_diag_no_step_ms, 0, sizeof g_diag_no_step_ms);
    memset(g_diag_stall_reported, 0, sizeof g_diag_stall_reported);
    g_diag_next_print_ms = 0u;
    memset(g_bugcheck_reported, 0, sizeof g_bugcheck_reported);
#endif
    for (uint8_t a = 0; a < MOTION_AXIS_COUNT; ++a) g_queue_rem_steps[a] = 0u;
    memset((void*)g_csv_active, 0, sizeof g_csv_active);
    memset((void*)g_csv_t0_ms, 0, sizeof g_csv_t0_ms);
    memset((void*)g_csv_stepcount, 0, sizeof g_csv_stepcount);
    memset((void*)g_csv_next_ms, 0, sizeof g_csv_next_ms);
    memset((void*)g_csv_armed, 0, sizeof g_csv_armed);
    memset((void*)g_csv_t0_t6, 0, sizeof g_csv_t0_t6);
    memset((void*)g_csv_next_t6, 0, sizeof g_csv_next_t6);
    memset((void*)g_csv_seq, 0, sizeof g_csv_seq);
    g_tim6_ticks = 0u;
#if MOTION_CSV_PRODUCE_IN_TIM6
    csv_ring_reset();
#endif

    g_status.state = MOTION_IDLE;
    g_queue_head = g_queue_tail = g_queue_count = 0u;
    g_has_active_segment = 0u;

    motion_stop_all_axes_locked();
    motion_refresh_status_locked();
    motion_unlock(primask);

    motion_hw_init();

    /* Debug: imprime configuração do encoder do eixo Y (LPTIM1) */
#if MOTION_DEBUG_ENCODERS
    {
        const char *pol_str = "?";
        const char *xmode_str = "?";
#ifdef LPTIM_CLOCKPOLARITY_RISING
        if (hlptim1.Init.UltraLowPowerClock.Polarity == LPTIM_CLOCKPOLARITY_RISING) {
            pol_str = "RISING"; xmode_str = "1x";
        }
#endif
#ifdef LPTIM_CLOCKPOLARITY_FALLING
        if (hlptim1.Init.UltraLowPowerClock.Polarity == LPTIM_CLOCKPOLARITY_FALLING) {
            pol_str = "FALLING"; xmode_str = "1x";
        }
#endif
#ifdef LPTIM_CLOCKPOLARITY_RISING_FALLING
        if (hlptim1.Init.UltraLowPowerClock.Polarity == LPTIM_CLOCKPOLARITY_RISING_FALLING) {
            pol_str = "RISING_FALLING"; xmode_str = "2x";
        }
#endif
        printf("[ENC_Y] bits=%u ENC_COUNTS_PER_REV=%lu microstep=%u pol=%s (modo %s)\r\n",
               (unsigned)motion_hw_encoder_bits(AXIS_Y),
               (unsigned long)ENC_COUNTS_PER_REV[AXIS_Y],
               (unsigned)g_microstep_factor[AXIS_Y],
               pol_str, xmode_str);
    }
#endif

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
    // Incrementa base de tempo de 50 kHz para telemetria
    g_tim6_ticks++;
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
                g_csv_stepcount[axis]++;
                if (!g_csv_active[axis]) {
                    g_csv_active[axis] = 1u;
                    g_csv_armed[axis]  = 0u;
                    uint32_t now_t6 = g_tim6_ticks;
                    g_csv_t0_t6[axis] = now_t6;
                    g_csv_next_t6[axis] = now_t6;
                }
#if MOTION_CSV_AT_STEP && !MOTION_CSV_PRODUCE_IN_TIM6
                if (g_csv_active[axis]) {
                    uint32_t dt_t6 = g_tim6_ticks - g_csv_t0_t6[axis];
#if MOTION_CSV_TIME_IN_TICKS
                    uint32_t t_val = dt_t6;
#else
                    uint32_t t_val = dt_t6 / T6_TICKS_PER_MS;
#endif
                    int32_t rel = (int32_t)(g_encoder_position[axis] - g_encoder_origin[axis]);
                    if (g_csv_stepcount[axis] == 1u || (g_csv_stepcount[axis] % MOTION_CSV_STEP_DECIM) == 0u) {
                        uint32_t pm = motion_lock();
                        uint32_t id = ++g_csv_seq[axis];
                        motion_unlock(pm);
                        motion_csv_print(axis, id, t_val, rel, g_csv_stepcount[axis]);
                    }
                }
#endif
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
                    g_csv_stepcount[axis]++;
                    ax->target_steps = ax->emitted_steps;
                    /* Garanta ativação no primeiro STEP mesmo se o "arming" atrasar */
                    if ((g_csv_armed[axis] || g_csv_stepcount[axis] == 1u) && !g_csv_active[axis]) {
                        g_csv_active[axis] = 1u;
                        g_csv_armed[axis]  = 0u;
                        uint32_t now_t6 = g_tim6_ticks;
                        g_csv_t0_t6[axis] = now_t6;
                        g_csv_next_t6[axis] = now_t6;
                    }
#if MOTION_CSV_AT_STEP && !MOTION_CSV_PRODUCE_IN_TIM6
                    if (g_csv_active[axis]) {
                        uint32_t dt_t6 = g_tim6_ticks - g_csv_t0_t6[axis];
#if MOTION_CSV_TIME_IN_TICKS
                        uint32_t t_val = dt_t6;
#else
                        uint32_t t_val = dt_t6 / T6_TICKS_PER_MS;
#endif
                        int32_t rel = (int32_t)(g_encoder_position[axis] - g_encoder_origin[axis]);
                        if (g_csv_stepcount[axis] == 1u || (g_csv_stepcount[axis] % MOTION_CSV_STEP_DECIM) == 0u) {
                            uint32_t pm = motion_lock();
                            uint32_t id = ++g_csv_seq[axis];
                            motion_unlock(pm);
                            motion_csv_print(axis, id, t_val, rel, g_csv_stepcount[axis]);
                        }
                    }
#endif
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
    }

#if MOTION_CSV_PRODUCE_IN_TIM6
    /* 4) Produção rígida de amostras a cada 1 ms (50 ticks), sem imprimir
       - Apenas empilha amostras para drenagem posterior (TIM7).
       - Começa após o primeiro STEP (g_csv_active[axis]==1). */
    {
        uint32_t now_t6 = g_tim6_ticks;
        if ((int32_t)(now_t6 - g_csv_next_prod_t6) >= 0) {
            /* gera 1..K amostras de 1ms (se houver atraso ocasional) */
            do {
                for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
                    if (!g_csv_active[axis]) continue; /* iniciar no primeiro STEP */
                    uint32_t dt_t6 = now_t6 - g_csv_t0_t6[axis];
                    long rel = (long)(g_encoder_position[axis] - g_encoder_origin[axis]);
                    uint32_t pm = motion_lock();
                    uint32_t id = ++g_csv_seq[axis];
                    motion_unlock(pm);
                    csv_ring_push(axis, id, dt_t6, (int32_t)rel, g_csv_stepcount[axis]);
                }
                g_csv_next_prod_t6 += T6_TICKS_PER_MS;
            } while ((int32_t)(now_t6 - g_csv_next_prod_t6) >= 0);
        }
    }
#endif

    uint32_t primask = motion_lock();
    if (g_has_active_segment) {
        uint8_t confirm = 1u;
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            const motion_axis_state_t *ax = &g_axis_state[axis];
            if (ax->emitted_steps < ax->total_steps || ax->step_high) {
                confirm = 0u; break;
            }
        }
#if MOTION_PROGRESS_MODE
        /* Confirmar término apenas quando não houver trabalho (ativo+fila)
           em NENHUM eixo. Usa soma O(1) por eixo (ativo + fila acumulada). */
        if (!confirm) {
            uint32_t rem_all = 0u;
            for (uint8_t a = 0; a < MOTION_AXIS_COUNT; ++a) {
                const motion_axis_state_t *ax = &g_axis_state[a];
                uint32_t active = (ax->total_steps > ax->emitted_steps)
                                  ? (ax->total_steps - ax->emitted_steps) : 0u;
                rem_all += active + g_queue_rem_steps[a];
            }
            if (rem_all == 0u) {
                confirm = 1u;
            }
        }
#endif
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
                // Fim do movimento: encerrar sessão CSV e zerar contadores
        for (uint8_t a = 0; a < MOTION_AXIS_COUNT; ++a) {
            g_csv_active[a] = 0u;
            g_csv_armed[a]  = 0u;
            g_csv_stepcount[a] = 0u;
            g_csv_seq[a] = 0u;
        }
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

    /* Debug: imprime encoder a cada N pulsos (por eixo via máscara) */
#if MOTION_DEBUG_ENCODERS && MOTION_ENC_PRINT_EVERY_N_ENABLE
    {
        static int32_t last_print_rel[3] = { INT32_MIN, INT32_MIN, INT32_MIN };
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            if ((MOTION_ENC_PRINT_MASK & (1u << axis)) == 0u) continue;
            int32_t d = g_encoder_delta_tick[axis];
            if (d == 0) continue;
            int32_t rel = (int32_t)(g_encoder_position[axis] - g_encoder_origin[axis]);
            int32_t absrel = (rel < 0) ? -rel : rel;
            if (MOTION_ENC_PRINT_N > 0u && (absrel % (int32_t)MOTION_ENC_PRINT_N) == 0 && rel != last_print_rel[axis]) {
                last_print_rel[axis] = rel;
                char axch = (char)('X' + axis);
                printf("[ENC_%c+%u] rel=%ld delta=%ld\r\n", axch, (unsigned)MOTION_ENC_PRINT_N, (long)rel, (long)d);
            }
        }
    }
#endif

#if (MOTION_CSV_SAMPLE_MS + 0u) > 0u && !MOTION_CSV_PRODUCE_IN_TIM6
    // Emite CSV (axis,time,rel,steps) periodicamente usando base de tempo do TIM6
    uint32_t now_t6 = g_tim6_ticks;
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
        if (!g_csv_active[axis]) continue;
        if ((int32_t)(now_t6 - g_csv_next_t6[axis]) >= 0) {
            int32_t rel = (int32_t)(g_encoder_position[axis] - g_encoder_origin[axis]);
            uint32_t dt_t6 = now_t6 - g_csv_t0_t6[axis];
#if MOTION_CSV_TIME_IN_TICKS
            uint32_t t_val = dt_t6;
#else
            uint32_t t_val = dt_t6 / T6_TICKS_PER_MS;
#endif
            uint32_t pm = motion_lock();
            uint32_t id = ++g_csv_seq[axis];
            motion_unlock(pm);
            motion_csv_print(axis, id, t_val, rel, g_csv_stepcount[axis]);
            uint32_t inc_ticks = ((uint32_t)MOTION_CSV_SAMPLE_MS) * (uint32_t)T6_TICKS_PER_MS;
            do { g_csv_next_t6[axis] += inc_ticks; } while ((int32_t)(now_t6 - g_csv_next_t6[axis]) >= 0);
        }
    }
#endif

#if MOTION_CSV_PRODUCE_IN_TIM6
    /* Drena ring buffer (SPSC) produzido no TIM6, com limite por tick */
    uint32_t drained = 0u;
    for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT && drained < MOTION_CSV_DRAIN_PER_TICK; ++axis) {
        while (drained < MOTION_CSV_DRAIN_PER_TICK) {
            motion_csv_sample_t s;
            if (!csv_ring_pop(axis, &s)) break;
            uint32_t t_val;
#if MOTION_CSV_TIME_IN_TICKS
            t_val = s.t6;
#else
            t_val = s.t6 / T6_TICKS_PER_MS;
#endif
            motion_csv_print(axis, s.id, t_val, s.rel, s.steps);
            drained++;
        }
    }
#endif

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
        int8_t master_axis = -1;
        uint32_t rem_master = 0u;
#if MOTION_PROGRESS_MODE
        master_axis = motion_select_master_axis_progress();
        if (master_axis >= 0) {
            const motion_axis_state_t *am = &g_axis_state[(uint8_t)master_axis];
            uint32_t active_m = (am->total_steps > am->emitted_steps)
                                ? (am->total_steps - am->emitted_steps) : 0u;
            rem_master = active_m + g_queue_rem_steps[(uint8_t)master_axis];
        }
#endif
#if MOTION_DIAG_ENABLE
        g_dbg_master_axis = master_axis;
#endif
        for (uint8_t axis = 0; axis < MOTION_AXIS_COUNT; ++axis) {
            motion_axis_state_t *ax = &g_axis_state[axis];
            /* Mesmo que o segmento ativo para este eixo tenha zerado, podemos ter
               passos remanescentes na fila — mantemos a rampa global da lista. */

            uint32_t v_cmd_sps = ((uint32_t)ax->velocity_per_tick) * 1000u; /* alvo/cruzeiro */
#if MOTION_DIAG_ENABLE
            g_dbg_scale_pm[axis] = 1000u;
#endif

#if MOTION_PROGRESS_MODE && MOTION_ERR_THROTTLE_ENABLE
            /* Throttle por erro (apenas eixos não-mestres):
               - Usa o MESMO erro posicional do PI (steps físicos)
               - Ajusta v_cmd antes de aplicar correção PI */
            if (master_axis >= 0 && (int8_t)axis != master_axis) {
                int32_t desired  = (int32_t)ax->target_steps;
                int32_t actual   = motion_actual_steps_from_encoder(axis);
                int32_t err      = desired - actual;
                uint32_t err_abs = (err < 0) ? (uint32_t)(-err) : (uint32_t)err;

                uint32_t scale_pm;
                if (err_abs >= (uint32_t)MOTION_ERR_THROTTLE_THRESHOLD) {
                    scale_pm = (uint32_t)MOTION_ERR_THROTTLE_MIN_PERMILLE; /* piso */
                } else {
                    uint32_t range = 1000u - (uint32_t)MOTION_ERR_THROTTLE_MIN_PERMILLE; /* 0..750 */
                    uint32_t dec = (uint32_t)(((uint64_t)range * (uint64_t)err_abs) / (uint32_t)MOTION_ERR_THROTTLE_THRESHOLD);
                    scale_pm = 1000u - dec; /* 1000..min_permille */
                }
                v_cmd_sps = (uint32_t)(((uint64_t)v_cmd_sps * (uint64_t)scale_pm) / 1000u);
                if (v_cmd_sps > MOTION_MAX_SPS) v_cmd_sps = MOTION_MAX_SPS;
#if MOTION_DIAG_ENABLE
                g_dbg_scale_pm[axis] = scale_pm;
#endif
            }
#endif /* MOTION_PROGRESS_MODE && MOTION_ERR_THROTTLE_ENABLE */
            /* PI de posição: ajusta v_cmd_sps com base no erro posicional */
#if MOTION_PI_ENABLE
            if ((ax->kp | ax->ki | ax->kd) != 0u) {
                /* desired e actual em steps físicos do eixo */
                int32_t desired = (int32_t)ax->target_steps;
                int32_t actual  = motion_actual_steps_from_encoder(axis);
                int32_t err     = desired - actual;
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
#if MOTION_DIAG_ENABLE
                g_dbg_err[axis]   = err;
                g_dbg_pterm[axis] = pterm;
                g_dbg_iterm[axis] = iterm;
                g_dbg_dterm[axis] = dterm;
                g_dbg_corr[axis]  = corr;
                g_dbg_v_adj[axis] = v_adj;
#endif
                /* Anti-windup por saturação: só aceita a integral quando não saturou */
                if (!(v_adj == 0 || v_adj == (int32_t)MOTION_MAX_SPS)) {
                    g_pi_i_accum[axis] = iacc;
                }
            }
#endif
            /* Guarda v_cmd resultante para diagnóstico */
#if MOTION_DIAG_ENABLE
            g_dbg_v_cmd_sps[axis] = v_cmd_sps;
#endif
            uint32_t a_sps2    = (ax->accel_sps2 > 0u) ? ax->accel_sps2 : DEMO_ACCEL_SPS2;

            /* Distância restante total (ativo + fila) em passos (O(1)) */
            uint32_t active_rem = (ax->total_steps > ax->emitted_steps)
                                  ? (ax->total_steps - ax->emitted_steps) : 0u;
            uint32_t rem_steps = active_rem + g_queue_rem_steps[axis];
#if MOTION_PROGRESS_MODE
            /* Só impõe o restante do mestre se ele tiver trabalho pendente */
            if (master_axis >= 0 && rem_master > 0u) {
                rem_steps = rem_master;
            }
            /* caso contrário, mantém o rem_steps do próprio eixo
               para não “matar” a rampa dos demais */
#endif

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

    /* Diagnóstico: snapshots e detecção de stall em baixa taxa (TIM7) */
#if MOTION_DIAG_ENABLE
    motion_diag_tick();
#endif
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
        uint16_t depth = (uint16_t)(g_queue_count + (g_has_active_segment ? 1u : 0u));
        printf("[FLOW start_move request depth=%u active=%u ids=(",
               (unsigned)((depth > 255u) ? 255u : depth), (unsigned)g_active_frame_id);
        for (uint16_t i = 0; i < g_queue_count; ++i) {
            uint8_t idxq = (uint8_t)(((uint16_t)g_queue_head + i) % MOTION_QUEUE_CAPACITY);
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

    // Arma sessões CSV para iniciarem no primeiro STEP; zera contadores
    if (started) {
        uint32_t pm = motion_lock();
        for (uint8_t a = 0; a < MOTION_AXIS_COUNT; ++a) {
            g_csv_armed[a] = 1u;             // iniciar ao primeiro STEP
            g_csv_active[a] = 0u;            // ainda não iniciou
            g_csv_t0_ms[a]  = 0u;
            g_csv_stepcount[a] = 0u;         // zera contador de passos
            g_csv_next_ms[a] = 0u;           // será setado no primeiro STEP
            g_csv_t0_t6[a] = 0u;
            g_csv_next_t6[a] = 0u;
            g_csv_seq[a] = 0u;               // reinicia id incremental por start_move
        }
        motion_unlock(pm);
#if MOTION_CSV_PRODUCE_IN_TIM6
        csv_ring_reset();
        /* agenda primeira produção 1ms à frente do tempo atual */
        g_csv_next_prod_t6 = g_tim6_ticks + (uint32_t)T6_TICKS_PER_MS;
#endif
    }

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
    /* Interpreta 0 como 256 para cobrir todo o conjunto {1,2,4,...,256} em 8 bits */
    uint16_t ms = (req.microsteps == 0u) ? 256u : req.microsteps;
    if (ms > 256u) ms = 256u;
    for (uint8_t a = 0; a < MOTION_AXIS_COUNT; ++a) g_microstep_factor[a] = ms;
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "set_microsteps", "all_axes_ms=%u", (unsigned)ms);
    /* Log humano-legível do valor aplicado */
    printf("[MICROSTEPS] all_axes=%u\r\n", (unsigned)ms);
    /* ACK detalhado (6 bytes): [HDR, TYPE, frameId, ms_hi, ms_lo, TAIL] */
    {
        uint8_t raw[6] = { RESP_HEADER, RESP_SET_MICROSTEPS, (uint8_t)req.frameId,
                           (uint8_t)(ms >> 8), (uint8_t)(ms & 0xFF), RESP_TAIL };
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    }
}

void motion_on_set_microsteps_axes(const uint8_t *frame, uint32_t len) {
    set_microsteps_axes_req_t req;
    if (set_microsteps_axes_req_decoder(frame, len, &req) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "set_microsteps_ax", "decode_fail");
        return;
    }
    if (g_status.state == MOTION_RUNNING) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "set_microsteps_ax", "busy_running");
        return;
    }
    /* Interpreta 0 como 256 (sentinela) para cada eixo */
    uint16_t msx = (req.ms_x == 0u) ? 256u : req.ms_x; if (msx > 256u) msx = 256u;
    uint16_t msy = (req.ms_y == 0u) ? 256u : req.ms_y; if (msy > 256u) msy = 256u;
    uint16_t msz = (req.ms_z == 0u) ? 256u : req.ms_z; if (msz > 256u) msz = 256u;
    g_microstep_factor[AXIS_X] = msx;
    g_microstep_factor[AXIS_Y] = msy;
    g_microstep_factor[AXIS_Z] = msz;
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "set_microsteps_ax", "ms=(%u,%u,%u)", (unsigned)msx, (unsigned)msy, (unsigned)msz);
    /* Log humano-legível por eixo */
    printf("[MICROSTEPS_AX] X=%u Y=%u Z=%u\r\n", (unsigned)msx, (unsigned)msy, (unsigned)msz);
    /* ACK detalhado (10 bytes): [HDR, TYPE, frameId, msx_hi, msx_lo, msy_hi, msy_lo, msz_hi, msz_lo, TAIL] */
    {
        uint8_t raw[10] = { RESP_HEADER, RESP_SET_MICROSTEPS, (uint8_t)req.frameId,
                            (uint8_t)(msx >> 8), (uint8_t)(msx & 0xFF),
                            (uint8_t)(msy >> 8), (uint8_t)(msy & 0xFF),
                            (uint8_t)(msz >> 8), (uint8_t)(msz & 0xFF),
                            RESP_TAIL };
        (void)app_resp_push(raw, (uint32_t)sizeof raw);
    }
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
