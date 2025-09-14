#include "protocol/requests/move_home_request.h"

int move_home_req_decoder(const uint8_t *raw, uint32_t len,
		move_home_req_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_req(raw, len, REQ_MOVE_HOME, 9);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->axisMask = raw[3];
	out->dirMask = raw[4];
	out->vhome = be16_read(&raw[5]);
	return PROTO_OK;
}

uint8_t move_home_req_calc_parity(const move_home_req_t *in) {
	uint8_t b[6] = { REQ_MOVE_HOME, in ? in->frameId : 0, in ? in->axisMask : 0,
			in ? in->dirMask : 0, (uint8_t) (in ? (in->vhome >> 8) : 0),
			(uint8_t) (in ? (in->vhome & 0xFF) : 0) };
	return xor_reduce_bytes(b, 6);
}

int move_home_req_encoder(const move_home_req_t *in, uint8_t *raw, uint32_t len) {
	if (!raw || !in || len < 9)
		return PROTO_ERR_ARG;
	req_init(raw, REQ_MOVE_HOME);
	raw[2] = in->frameId;
	raw[3] = in->axisMask;
	raw[4] = in->dirMask;
	be16_write(&raw[5], in->vhome);
	parity_set_byte_1N(raw, 6, 7);
	req_set_tail(raw, 8);
	return PROTO_OK;
}

int move_home_req_check_parity(const uint8_t *raw, uint32_t len) {
	if (frame_expect_req(raw, len, REQ_MOVE_HOME, 9) != PROTO_OK)
		return 0;
	return parity_check_byte_1N(raw, 6, 7);
}
int move_home_req_set_parity(uint8_t *raw, uint32_t len) {
	if (!raw || len < 9)
		return PROTO_ERR_ARG;
	return parity_set_byte_1N(raw, 6, 7);
}
move_home_req_t move_home_req_make_default(void) {
	move_home_req_t d = { 0 };
	return d;
}
