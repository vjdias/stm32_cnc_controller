#include "protocol/responses/move_probe_level_response.h"

int move_probe_level_resp_decoder(const uint8_t *raw, uint32_t len,
		move_probe_level_resp_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_resp(raw, len, RESP_MOVE_PROBE_LEVEL, 20);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->status = raw[3];
	out->axisDoneMask = raw[4];
	out->errorFlags = raw[5];
	out->latchedPosX = be32_read(&raw[6]);
	out->latchedPosY = be32_read(&raw[10]);
	out->latchedPosZ = be32_read(&raw[14]);
	return PROTO_OK;
}
uint8_t move_probe_level_resp_calc_parity(const move_probe_level_resp_t *in) {
	uint8_t b[17];
	b[0] = RESP_MOVE_PROBE_LEVEL;
	b[1] = in ? in->frameId : 0;
	b[2] = in ? in->status : 0;
	b[3] = in ? in->axisDoneMask : 0;
	b[4] = in ? in->errorFlags : 0;
	be32_write(&b[5], in ? in->latchedPosX : 0);
	be32_write(&b[9], in ? in->latchedPosY : 0);
	be32_write(&b[13], in ? in->latchedPosZ : 0);
	return xor_reduce_bytes(b, 17);
}
int move_probe_level_resp_encoder(const move_probe_level_resp_t *in,
		uint8_t *raw, uint32_t len) {
	if (!raw || !in || len < 20)
		return PROTO_ERR_ARG;
	resp_init(raw, RESP_MOVE_PROBE_LEVEL);
	raw[2] = in->frameId;
	raw[3] = in->status;
	raw[4] = in->axisDoneMask;
	raw[5] = in->errorFlags;
	be32_write(&raw[6], in->latchedPosX);
	be32_write(&raw[10], in->latchedPosY);
	be32_write(&raw[14], in->latchedPosZ);
	parity_set_byte_1N(raw, 17, 18);
	resp_set_tail(raw, 19);
	return PROTO_OK;
}
int move_probe_level_resp_check_parity(const uint8_t *raw, uint32_t len) {
	if (frame_expect_resp(raw, len, RESP_MOVE_PROBE_LEVEL, 20) != PROTO_OK)
		return 0;
	return parity_check_byte_1N(raw, 17, 18);
}
int move_probe_level_resp_set_parity(uint8_t *raw, uint32_t len) {
	if (!raw || len < 20)
		return PROTO_ERR_ARG;
	return parity_set_byte_1N(raw, 17, 18);
}
move_probe_level_resp_t move_probe_level_resp_make_default(void) {
	move_probe_level_resp_t d = { 0 };
	return d;
}
