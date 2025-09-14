// MOVE_QUEUE_STATUS (4 bytes) — 0x02
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
} move_queue_status_req_t;

int move_queue_status_req_decoder(const uint8_t *raw, uint32_t len,
		move_queue_status_req_t *out);
int move_queue_status_req_encoder(const move_queue_status_req_t *in,
		uint8_t *raw, uint32_t len);
uint8_t move_queue_status_req_calc_parity(const move_queue_status_req_t *in); // N/A
int move_queue_status_req_check_parity(const uint8_t *raw, uint32_t len); // N/A
int move_queue_status_req_set_parity(uint8_t *raw, uint32_t len);         // N/A
move_queue_status_req_t move_queue_status_req_make_default(void);

