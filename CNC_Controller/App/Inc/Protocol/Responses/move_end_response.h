// MOVE_END (4 bytes) — 0x06
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
} move_end_resp_t;

int move_end_resp_decoder(const uint8_t *raw, uint32_t len,
		move_end_resp_t *out);
int move_end_resp_encoder(const move_end_resp_t *in, uint8_t *raw, uint32_t len);
uint8_t move_end_resp_calc_parity(const move_end_resp_t *in); // N/A
int move_end_resp_check_parity(const uint8_t *raw, uint32_t len); // N/A
int move_end_resp_set_parity(uint8_t *raw, uint32_t len); // N/A
move_end_resp_t move_end_resp_make_default(void);

