// MOVE_PROBE_LEVEL (20 bytes) — 0x05
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
	uint8_t status;
	uint8_t axisDoneMask;
	uint8_t errorFlags;
	uint32_t latchedPosX, latchedPosY, latchedPosZ;
} move_probe_level_resp_t;

int move_probe_level_resp_decoder(const uint8_t *raw, uint32_t len,
		move_probe_level_resp_t *out);
int move_probe_level_resp_encoder(const move_probe_level_resp_t *in,
		uint8_t *raw, uint32_t len);
uint8_t move_probe_level_resp_calc_parity(const move_probe_level_resp_t *in);
int move_probe_level_resp_check_parity(const uint8_t *raw, uint32_t len);
int move_probe_level_resp_set_parity(uint8_t *raw, uint32_t len);
move_probe_level_resp_t move_probe_level_resp_make_default(void);

