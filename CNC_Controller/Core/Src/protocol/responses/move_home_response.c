#include "protocol/responses/move_home_response.h"

int move_home_resp_decoder(const uint8_t *raw, uint32_t len,
		move_home_resp_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_resp(raw, len, RESP_MOVE_HOME, 8);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->status = raw[3];
	out->axisHomeMask = raw[4];
	out->errorFlags = raw[5];
	return PROTO_OK;
}
uint8_t move_home_resp_calc_parity(const move_home_resp_t *in) {
	uint8_t b[5] = { RESP_MOVE_HOME, in ? in->frameId : 0, in ? in->status : 0,
			in ? in->axisHomeMask : 0, in ? in->errorFlags : 0 };
	return xor_reduce_bytes(b, 5);
}
int move_home_resp_encoder(const move_home_resp_t *in, uint8_t *raw,
		uint32_t len) {
	if (!raw || !in || len < 8)
		return PROTO_ERR_ARG;
	resp_init(raw, RESP_MOVE_HOME);
	raw[2] = in->frameId;
	raw[3] = in->status;
	raw[4] = in->axisHomeMask;
	raw[5] = in->errorFlags;
	parity_set_byte_1N(raw, 5, 6);
	resp_set_tail(raw, 7);
	return PROTO_OK;
}
int move_home_resp_check_parity(const uint8_t *raw, uint32_t len) {
	if (frame_expect_resp(raw, len, RESP_MOVE_HOME, 8) != PROTO_OK)
		return 0;
	return parity_check_byte_1N(raw, 5, 6);
}
int move_home_resp_set_parity(uint8_t *raw, uint32_t len) {
	if (!raw || len < 8)
		return PROTO_ERR_ARG;
	return parity_set_byte_1N(raw, 5, 6);
}
move_home_resp_t move_home_resp_make_default(void) {
	move_home_resp_t d = { 0 };
	return d;
}
