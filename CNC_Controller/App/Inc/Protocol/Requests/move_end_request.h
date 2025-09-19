// MOVE_END (4 bytes) — 0x06
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
} move_end_req_t;

int move_end_req_decoder(const uint8_t *raw, uint32_t len, move_end_req_t *out);
int move_end_req_encoder(const move_end_req_t *in, uint8_t *raw, uint32_t len);
uint8_t move_end_req_calc_parity(const move_end_req_t *in); // N/D
int move_end_req_check_parity(const uint8_t *raw, uint32_t len); // N/D
int move_end_req_set_parity(uint8_t *raw, uint32_t len); // N/D
move_end_req_t move_end_req_make_default(void);

