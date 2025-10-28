// START_MOVE (5 bytes) — 0x03
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
    uint8_t status;   // 0=started, 1=ignored/busy/unsafe
} start_move_resp_t;

int start_move_resp_decoder(const uint8_t *raw, uint32_t len,
		start_move_resp_t *out);
int start_move_resp_encoder(const start_move_resp_t *in, uint8_t *raw,
		uint32_t len);
uint8_t start_move_resp_calc_parity(const start_move_resp_t *in); // N/D
int start_move_resp_check_parity(const uint8_t *raw, uint32_t len); // N/D
int start_move_resp_set_parity(uint8_t *raw, uint32_t len); // N/D
start_move_resp_t start_move_resp_make_default(void);
