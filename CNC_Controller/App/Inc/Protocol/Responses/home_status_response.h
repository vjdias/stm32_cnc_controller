// HOME_STATUS (18 bytes) — 0x21
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
	uint8_t axisMask;
	uint16_t posRelX, homeOffX, posRelY, homeOffY, posRelZ, homeOffZ;
} home_status_resp_t;

int home_status_resp_decoder(const uint8_t *raw, uint32_t len,
		home_status_resp_t *out);
int home_status_resp_encoder(const home_status_resp_t *in, uint8_t *raw,
		uint32_t len);
uint8_t home_status_resp_calc_parity(const home_status_resp_t *in);
int home_status_resp_check_parity(const uint8_t *raw, uint32_t len);
int home_status_resp_set_parity(uint8_t *raw, uint32_t len);
home_status_resp_t home_status_resp_make_default(void);

