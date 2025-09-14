// MOVE_QUEUE_ADD (42 bytes) — 0x01
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
	uint8_t dirMask;
	uint16_t vx;
	uint32_t sx;
	uint16_t vy;
	uint32_t sy;
	uint16_t vz;
	uint32_t sz;
	uint16_t kp_x, ki_x, kd_x;
	uint16_t kp_y, ki_y, kd_y;
	uint16_t kp_z, ki_z, kd_z;
} move_queue_add_req_t;

int move_queue_add_req_decoder(const uint8_t *raw, uint32_t len,
		move_queue_add_req_t *out);
int move_queue_add_req_encoder(const move_queue_add_req_t *in, uint8_t *raw,
		uint32_t len);
uint8_t move_queue_add_req_calc_parity(const move_queue_add_req_t *in); // returns bit in bit0
int move_queue_add_req_check_parity(const uint8_t *raw, uint32_t len);
int move_queue_add_req_set_parity(uint8_t *raw, uint32_t len);
move_queue_add_req_t move_queue_add_req_make_default(void);

