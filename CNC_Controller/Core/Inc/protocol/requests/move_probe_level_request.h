// MOVE_PROBE_LEVEL (8 bytes) — 0x05
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
	uint8_t axisMask;
	uint16_t vprobe; // big-endian on wire
} move_probe_level_req_t;

int move_probe_level_req_decoder(const uint8_t *raw, uint32_t len,
		move_probe_level_req_t *out);
int move_probe_level_req_encoder(const move_probe_level_req_t *in, uint8_t *raw,
		uint32_t len);
uint8_t move_probe_level_req_calc_parity(const move_probe_level_req_t *in);
int move_probe_level_req_check_parity(const uint8_t *raw, uint32_t len);
int move_probe_level_req_set_parity(uint8_t *raw, uint32_t len);
move_probe_level_req_t move_probe_level_req_make_default(void);

