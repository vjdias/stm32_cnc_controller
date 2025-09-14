#include "Protocol/Requests/start_move_request.h"

int start_move_req_decoder(const uint8_t *raw, uint32_t len,
		start_move_req_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_req(raw, len, REQ_START_MOVE, 4);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	return PROTO_OK;
}
int start_move_req_encoder(const start_move_req_t *in, uint8_t *raw,
		uint32_t len) {
	if (!raw || !in || len < 4)
		return PROTO_ERR_ARG;
	req_init(raw, REQ_START_MOVE);
	raw[2] = in->frameId;
	req_set_tail(raw, 3);
	return PROTO_OK;
}
uint8_t start_move_req_calc_parity(const start_move_req_t *in) {
	(void) in;
	return 0;
}
int start_move_req_check_parity(const uint8_t *raw, uint32_t len) {
	(void) raw;
	(void) len;
	return 1;
}
int start_move_req_set_parity(uint8_t *raw, uint32_t len) {
	(void) raw;
	(void) len;
	return 0;
}
start_move_req_t start_move_req_make_default(void) {
	start_move_req_t d = { 0 };
	return d;
}
