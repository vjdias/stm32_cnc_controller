#include "Services/Safety/safety_service.h"

static volatile safety_state_t g_state = SAFETY_NORMAL;

void safety_service_init(void) {
	g_state = SAFETY_NORMAL;
}
void safety_estop_assert(void) {
	g_state = SAFETY_ESTOP;
}
void safety_estop_release(void) {
	if (g_state == SAFETY_ESTOP)
		g_state = SAFETY_RECOVERY_WAIT;
}
int safety_is_safe(void) {
	return g_state == SAFETY_NORMAL;
}
