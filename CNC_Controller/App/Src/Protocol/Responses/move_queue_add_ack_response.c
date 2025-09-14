#include "Protocol/Responses/move_queue_add_ack_response.h"

int move_queue_add_ack_resp_decoder(const uint8_t *raw, uint32_t len,
		move_queue_add_ack_resp_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_resp(raw, len, RESP_MOVE_QUEUE_ADD_ACK, 6);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->status = raw[3];
	return PROTO_OK;
}
uint8_t move_queue_add_ack_resp_calc_parity(const move_queue_add_ack_resp_t *in) {
	uint8_t b[3] = { RESP_MOVE_QUEUE_ADD_ACK, in ? in->frameId : 0,
			in ? in->status : 0 };
	return xor_bit_reduce_bytes(b, 3);
}
int move_queue_add_ack_resp_encoder(const move_queue_add_ack_resp_t *in,
		uint8_t *raw, uint32_t len) {
	if (!raw || !in || len < 6)
		return PROTO_ERR_ARG;
	resp_init(raw, RESP_MOVE_QUEUE_ADD_ACK);
	raw[2] = in->frameId;
	raw[3] = in->status;
	parity_set_bit_1N(raw, 3, 4);
	resp_set_tail(raw, 5);
	return PROTO_OK;
}
int move_queue_add_ack_resp_check_parity(const uint8_t *raw, uint32_t len) {
	if (frame_expect_resp(raw, len, RESP_MOVE_QUEUE_ADD_ACK, 6) != PROTO_OK)
		return 0;
	return parity_check_bit_1N(raw, 3, 4);
}
int move_queue_add_ack_resp_set_parity(uint8_t *raw, uint32_t len) {
	if (!raw || len < 6)
		return PROTO_ERR_ARG;
	return parity_set_bit_1N(raw, 3, 4);
}
move_queue_add_ack_resp_t move_queue_add_ack_resp_make_default(void) {
	move_queue_add_ack_resp_t d = { 0 };
	return d;
}
