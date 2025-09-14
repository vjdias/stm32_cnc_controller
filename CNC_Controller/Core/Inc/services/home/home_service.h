// Home service (FSM por eixo)
#pragma once
#include <stdint.h>

typedef enum {
	HOME_IDLE = 0,
	HOME_SEEK,
	HOME_LATCH,
	HOME_BACKOFF,
	HOME_SET_ZERO,
	HOME_DONE,
	HOME_ERROR,
} home_state_t;

typedef struct {
	volatile uint8_t axis_done_mask;
	volatile uint8_t error_flags;
} home_status_t;

void home_service_init(void);
const home_status_t* home_status_get(void);

// Handler do frame MOVE_HOME
void home_on_move_home(const uint8_t *frame, uint32_t len);

