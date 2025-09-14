#include "protocol/requests/move_queue_add_request.h"

int move_queue_add_req_decoder(const uint8_t *raw, uint32_t len,
		move_queue_add_req_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_req(raw, len, REQ_MOVE_QUEUE_ADD, 42);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->dirMask = raw[3];
	out->vx = be16_read(&raw[4]);
	out->sx = be32_read(&raw[6]);
	out->vy = be16_read(&raw[10]);
	out->sy = be32_read(&raw[12]);
	out->vz = be16_read(&raw[16]);
	out->sz = be32_read(&raw[18]);
	out->kp_x = be16_read(&raw[22]);
	out->ki_x = be16_read(&raw[24]);
	out->kd_x = be16_read(&raw[26]);
	out->kp_y = be16_read(&raw[28]);
	out->ki_y = be16_read(&raw[30]);
	out->kd_y = be16_read(&raw[32]);
	out->kp_z = be16_read(&raw[34]);
	out->ki_z = be16_read(&raw[36]);
	out->kd_z = be16_read(&raw[38]);
	return PROTO_OK;
}

uint8_t move_queue_add_req_calc_parity(const move_queue_add_req_t *in) {
	uint8_t b[39];
	b[0] = REQ_MOVE_QUEUE_ADD;
	b[1] = in ? in->frameId : 0;
	b[2] = in ? in->dirMask : 0;
	uint8_t *p = &b[3];
	be16_write(p, in ? in->vx : 0);
	p += 2;
	be32_write(p, in ? in->sx : 0);
	p += 4;
	be16_write(p, in ? in->vy : 0);
	p += 2;
	be32_write(p, in ? in->sy : 0);
	p += 4;
	be16_write(p, in ? in->vz : 0);
	p += 2;
	be32_write(p, in ? in->sz : 0);
	p += 4;
	be16_write(p, in ? in->kp_x : 0);
	p += 2;
	be16_write(p, in ? in->ki_x : 0);
	p += 2;
	be16_write(p, in ? in->kd_x : 0);
	p += 2;
	be16_write(p, in ? in->kp_y : 0);
	p += 2;
	be16_write(p, in ? in->ki_y : 0);
	p += 2;
	be16_write(p, in ? in->kd_y : 0);
	p += 2;
	be16_write(p, in ? in->kp_z : 0);
	p += 2;
	be16_write(p, in ? in->ki_z : 0);
	p += 2;
	be16_write(p, in ? in->kd_z : 0);
	p += 2;
	return (uint8_t) (xor_bit_reduce_bytes(b, 39) & 0x1);
}

int move_queue_add_req_encoder(const move_queue_add_req_t *in, uint8_t *raw,
		uint32_t len) {
	if (!raw || !in || len < 42)
		return PROTO_ERR_ARG;
	req_init(raw, REQ_MOVE_QUEUE_ADD);
	raw[2] = in->frameId;
	raw[3] = in->dirMask;
	be16_write(&raw[4], in->vx);
	be32_write(&raw[6], in->sx);
	be16_write(&raw[10], in->vy);
	be32_write(&raw[12], in->sy);
	be16_write(&raw[16], in->vz);
	be32_write(&raw[18], in->sz);
	be16_write(&raw[22], in->kp_x);
	be16_write(&raw[24], in->ki_x);
	be16_write(&raw[26], in->kd_x);
	be16_write(&raw[28], in->kp_y);
	be16_write(&raw[30], in->ki_y);
	be16_write(&raw[32], in->kd_y);
	be16_write(&raw[34], in->kp_z);
	be16_write(&raw[36], in->ki_z);
	be16_write(&raw[38], in->kd_z);
	parity_set_bit_1N(raw, 39, 40);
	req_set_tail(raw, 41);
	return PROTO_OK;
}

int move_queue_add_req_check_parity(const uint8_t *raw, uint32_t len) {
	if (frame_expect_req(raw, len, REQ_MOVE_QUEUE_ADD, 42) != PROTO_OK)
		return 0;
	return parity_check_bit_1N(raw, 39, 40);
}
int move_queue_add_req_set_parity(uint8_t *raw, uint32_t len) {
	if (!raw || len < 42)
		return PROTO_ERR_ARG;
	return parity_set_bit_1N(raw, 39, 40);
}
move_queue_add_req_t move_queue_add_req_make_default(void) {
	move_queue_add_req_t d = { 0 };
	return d;
}
