// Serviço de movimento (FSM simplificado) — interface
#pragma once
#include <stdint.h>

typedef enum {
	MOTION_IDLE = 0,
	MOTION_QUEUED,
	MOTION_RUNNING,
	MOTION_PAUSED,
	MOTION_STOPPING,
	MOTION_DONE,
	MOTION_ERROR,
} motion_state_t;

typedef struct {
	volatile motion_state_t state;
	volatile uint8_t queue_depth;
	volatile uint8_t pctX, pctY, pctZ;
	volatile int8_t pidErrX, pidErrY, pidErrZ;
} motion_status_t;

// Inicialização e status
void motion_service_init(void);
const motion_status_t* motion_status_get(void);

// Hooks de tempo (chamar em TIM6/TIM7)
void motion_on_tim6_tick(void); // 100 kHz DDA (consome slots)
void motion_on_tim7_tick(void); // 1 kHz controle/status

// Handlers de frames
void motion_on_move_queue_add(const uint8_t *frame, uint32_t len);
void motion_on_move_queue_status(const uint8_t *frame, uint32_t len);
void motion_on_start_move(const uint8_t *frame, uint32_t len);
void motion_on_move_end(const uint8_t *frame, uint32_t len);

// Demo opcional: habilita um movimento de teste interno (sem host)
void motion_demo_set_enabled(uint8_t enable);
// Modo contínuo de teste: gera passos sem parar até desativar
void motion_demo_set_continuous(uint8_t enable);

// Parada imediata de segurança (E-STOP): desabilita drivers, limpa fila e
// retorna FSM para estado seguro (IDLE/DONE). Pode ser chamada de ISR.
void motion_emergency_stop(void);

// Utilidades do modo demo (teste):
// - Retorna 1 se o gerador contínuo está habilitado; 0 caso contrário
uint8_t motion_demo_is_active(void);
// - Avança para a próxima velocidade predefinida (4 passos) e aplica ao demo
void motion_demo_cycle_speed(void);

