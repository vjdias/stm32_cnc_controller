#include "protocol/requests/move_end_request.h"

int move_end_req_decoder(const uint8_t *raw, uint32_t len, move_end_req_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_req(raw, len, REQ_MOVE_END, 4);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	return PROTO_OK;
}
int move_end_req_encoder(const move_end_req_t *in, uint8_t *raw, uint32_t len) {
	if (!raw || !in || len < 4)
		return PROTO_ERR_ARG;
	req_init(raw, REQ_MOVE_END);
	raw[2] = in->frameId;
	req_set_tail(raw, 3);
	return PROTO_OK;
}
uint8_t move_end_req_calc_parity(const move_end_req_t *in) {
	(void) in;
	return 0;
}
int move_end_req_check_parity(const uint8_t *raw, uint32_t len) {
	(void) raw;
	(void) len;
	return 1;
}
int move_end_req_set_parity(uint8_t *raw, uint32_t len) {
	(void) raw;
	(void) len;
	return 0;
}
move_end_req_t move_end_req_make_default(void) {
	move_end_req_t d = { 0 };
	return d;
}
