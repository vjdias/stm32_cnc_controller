// START_MOVE (4 bytes) — 0x03
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
} start_move_req_t;

int start_move_req_decoder(const uint8_t *raw, uint32_t len,
		start_move_req_t *out);
int start_move_req_encoder(const start_move_req_t *in, uint8_t *raw,
		uint32_t len);
uint8_t start_move_req_calc_parity(const start_move_req_t *in); // N/D
int start_move_req_check_parity(const uint8_t *raw, uint32_t len); // N/D
int start_move_req_set_parity(uint8_t *raw, uint32_t len); // N/D
start_move_req_t start_move_req_make_default(void);

