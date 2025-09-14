// MOVE_HOME (8 bytes) — 0x04
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
	uint8_t status;
	uint8_t axisHomeMask;
	uint8_t errorFlags;
} move_home_resp_t;

int move_home_resp_decoder(const uint8_t *raw, uint32_t len,
		move_home_resp_t *out);
int move_home_resp_encoder(const move_home_resp_t *in, uint8_t *raw,
		uint32_t len);
uint8_t move_home_resp_calc_parity(const move_home_resp_t *in);
int move_home_resp_check_parity(const uint8_t *raw, uint32_t len);
int move_home_resp_set_parity(uint8_t *raw, uint32_t len);
move_home_resp_t move_home_resp_make_default(void);

