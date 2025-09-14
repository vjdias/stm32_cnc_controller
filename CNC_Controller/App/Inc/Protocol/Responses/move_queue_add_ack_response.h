// MOVE_QUEUE_ADD_ACK (6 bytes) — 0x01
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
	uint8_t status;
} move_queue_add_ack_resp_t;

int move_queue_add_ack_resp_decoder(const uint8_t *raw, uint32_t len,
		move_queue_add_ack_resp_t *out);
int move_queue_add_ack_resp_encoder(const move_queue_add_ack_resp_t *in,
		uint8_t *raw, uint32_t len);
uint8_t move_queue_add_ack_resp_calc_parity(const move_queue_add_ack_resp_t *in); // bit reduce of bytes 1..3
int move_queue_add_ack_resp_check_parity(const uint8_t *raw, uint32_t len);
int move_queue_add_ack_resp_set_parity(uint8_t *raw, uint32_t len);
move_queue_add_ack_resp_t move_queue_add_ack_resp_make_default(void);

