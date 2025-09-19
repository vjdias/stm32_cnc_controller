// Serviço de nivelamento por probe (FSM simplificado)
#pragma once
#include <stdint.h>

typedef enum {
	PROBE_IDLE = 0,
	PROBE_SEEK,
	PROBE_LATCH,
	PROBE_REPORT,
	PROBE_DONE,
	PROBE_ERROR,
} probe_state_t;

typedef struct {
	volatile uint8_t axis_done_mask;
	volatile uint8_t error_flags;
	volatile uint32_t latched_pos_x;
	volatile uint32_t latched_pos_y;
	volatile uint32_t latched_pos_z;
} probe_status_t;

void probe_service_init(void);
const probe_status_t* probe_status_get(void);

// Handler do frame MOVE_PROBE_LEVEL
void probe_on_move_probe_level(const uint8_t *frame, uint32_t len);

