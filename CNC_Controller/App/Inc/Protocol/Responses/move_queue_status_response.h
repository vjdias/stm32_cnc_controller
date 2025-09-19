// MOVE_QUEUE_STATUS (12 bytes) — 0x02
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
	uint8_t status;
	uint8_t pidErrX, pidErrY, pidErrZ;
	uint8_t pctX, pctY, pctZ;
} move_queue_status_resp_t;

int move_queue_status_resp_decoder(const uint8_t *raw, uint32_t len,
		move_queue_status_resp_t *out);
int move_queue_status_resp_encoder(const move_queue_status_resp_t *in,
		uint8_t *raw, uint32_t len);
uint8_t move_queue_status_resp_calc_parity(const move_queue_status_resp_t *in); // redução aos bits dos bytes 1..9
int move_queue_status_resp_check_parity(const uint8_t *raw, uint32_t len);
int move_queue_status_resp_set_parity(uint8_t *raw, uint32_t len);
move_queue_status_resp_t move_queue_status_resp_make_default(void);

