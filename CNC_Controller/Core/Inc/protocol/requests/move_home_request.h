// MOVE_HOME (9 bytes) — 0x04
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
	uint8_t axisMask;
	uint8_t dirMask;
	uint16_t vhome; // big-endian on wire
} move_home_req_t;

int move_home_req_decoder(const uint8_t *raw, uint32_t len,
		move_home_req_t *out);
int move_home_req_encoder(const move_home_req_t *in, uint8_t *raw, uint32_t len);
uint8_t move_home_req_calc_parity(const move_home_req_t *in);
int move_home_req_check_parity(const uint8_t *raw, uint32_t len);
int move_home_req_set_parity(uint8_t *raw, uint32_t len);
move_home_req_t move_home_req_make_default(void);

