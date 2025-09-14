#include "protocol/responses/move_queue_status_response.h"

int move_queue_status_resp_decoder(const uint8_t *raw, uint32_t len,
		move_queue_status_resp_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_resp(raw, len, RESP_MOVE_QUEUE_STATUS, 12);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->status = raw[3];
	out->pidErrX = raw[4];
	out->pidErrY = raw[5];
	out->pidErrZ = raw[6];
	out->pctX = raw[7];
	out->pctY = raw[8];
	out->pctZ = raw[9];
	return PROTO_OK;
}
uint8_t move_queue_status_resp_calc_parity(const move_queue_status_resp_t *in) {
	uint8_t b[9] = { RESP_MOVE_QUEUE_STATUS, in ? in->frameId : 0,
			in ? in->status : 0, in ? in->pidErrX : 0, in ? in->pidErrY : 0,
			in ? in->pidErrZ : 0, in ? in->pctX : 0, in ? in->pctY : 0,
			in ? in->pctZ : 0 };
	return xor_bit_reduce_bytes(b, 9);
}
int move_queue_status_resp_encoder(const move_queue_status_resp_t *in,
		uint8_t *raw, uint32_t len) {
	if (!raw || !in || len < 12)
		return PROTO_ERR_ARG;
	resp_init(raw, RESP_MOVE_QUEUE_STATUS);
	raw[2] = in->frameId;
	raw[3] = in->status;
	raw[4] = in->pidErrX;
	raw[5] = in->pidErrY;
	raw[6] = in->pidErrZ;
	raw[7] = in->pctX;
	raw[8] = in->pctY;
	raw[9] = in->pctZ;
	parity_set_bit_1N(raw, 9, 10);
	resp_set_tail(raw, 11);
	return PROTO_OK;
}
int move_queue_status_resp_check_parity(const uint8_t *raw, uint32_t len) {
	if (frame_expect_resp(raw, len, RESP_MOVE_QUEUE_STATUS, 12) != PROTO_OK)
		return 0;
	return parity_check_bit_1N(raw, 9, 10);
}
int move_queue_status_resp_set_parity(uint8_t *raw, uint32_t len) {
	if (!raw || len < 12)
		return PROTO_ERR_ARG;
	return parity_set_bit_1N(raw, 9, 10);
}
move_queue_status_resp_t move_queue_status_resp_make_default(void) {
	move_queue_status_resp_t d = { 0 };
	return d;
}
