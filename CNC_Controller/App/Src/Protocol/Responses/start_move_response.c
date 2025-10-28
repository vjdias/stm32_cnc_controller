#include "Protocol/Responses/start_move_response.h"

int start_move_resp_decoder(const uint8_t *raw, uint32_t len,
		start_move_resp_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_resp(raw, len, RESP_START_MOVE, 5);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->status = raw[3];
	return PROTO_OK;
}
int start_move_resp_encoder(const start_move_resp_t *in, uint8_t *raw,
		uint32_t len) {
	if (!raw || !in || len < 5)
		return PROTO_ERR_ARG;
	resp_init(raw, RESP_START_MOVE);
	raw[2] = in->frameId;
	raw[3] = in->status;
	resp_set_tail(raw, 4);
	return PROTO_OK;
}
uint8_t start_move_resp_calc_parity(const start_move_resp_t *in) {
	(void) in;
	return 0;
}
int start_move_resp_check_parity(const uint8_t *raw, uint32_t len) {
	(void) raw;
	(void) len;
	return 1;
}
int start_move_resp_set_parity(uint8_t *raw, uint32_t len) {
	(void) raw;
	(void) len;
	return 0;
}
start_move_resp_t start_move_resp_make_default(void) {
	start_move_resp_t d = { 0 };
	return d;
}
