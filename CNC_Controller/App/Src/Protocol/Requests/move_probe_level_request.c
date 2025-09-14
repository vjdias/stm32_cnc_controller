#include "Protocol/Requests/move_probe_level_request.h"

int move_probe_level_req_decoder(const uint8_t *raw, uint32_t len,
		move_probe_level_req_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_req(raw, len, REQ_MOVE_PROBE_LEVEL, 8);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->axisMask = raw[3];
	out->vprobe = be16_read(&raw[4]);
	return PROTO_OK;
}
uint8_t move_probe_level_req_calc_parity(const move_probe_level_req_t *in) {
	uint8_t b[5] = { REQ_MOVE_PROBE_LEVEL, in ? in->frameId : 0,
			in ? in->axisMask : 0, (uint8_t) (in ? in->vprobe >> 8 : 0),
			(uint8_t) (in ? in->vprobe : 0) };
	return xor_reduce_bytes(b, 5);
}
int move_probe_level_req_encoder(const move_probe_level_req_t *in, uint8_t *raw,
		uint32_t len) {
	if (!raw || !in || len < 8)
		return PROTO_ERR_ARG;
	req_init(raw, REQ_MOVE_PROBE_LEVEL);
	raw[2] = in->frameId;
	raw[3] = in->axisMask;
	be16_write(&raw[4], in->vprobe);
	parity_set_byte_1N(raw, 5, 6);
	req_set_tail(raw, 7);
	return PROTO_OK;
}
int move_probe_level_req_check_parity(const uint8_t *raw, uint32_t len) {
	if (frame_expect_req(raw, len, REQ_MOVE_PROBE_LEVEL, 8) != PROTO_OK)
		return 0;
	return parity_check_byte_1N(raw, 5, 6);
}
int move_probe_level_req_set_parity(uint8_t *raw, uint32_t len) {
	if (!raw || len < 8)
		return PROTO_ERR_ARG;
	return parity_set_byte_1N(raw, 5, 6);
}
move_probe_level_req_t move_probe_level_req_make_default(void) {
	move_probe_level_req_t d = { 0 };
	return d;
}
