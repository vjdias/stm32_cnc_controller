// Safety service (E-STOP/FAULT)
#pragma once
#include <stdint.h>

typedef enum {
	SAFETY_NORMAL = 0, SAFETY_ESTOP, SAFETY_RECOVERY_WAIT,
} safety_state_t;

void safety_service_init(void);
void safety_estop_assert(void);
void safety_estop_release(void);
int safety_is_safe(void);

