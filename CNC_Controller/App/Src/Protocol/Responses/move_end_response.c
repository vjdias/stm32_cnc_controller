#include "Protocol/Responses/move_end_response.h"

int move_end_resp_decoder(const uint8_t *raw, uint32_t len,
		move_end_resp_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_resp(raw, len, RESP_MOVE_END, 5);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->status  = raw[3];
	return PROTO_OK;
}
int move_end_resp_encoder(const move_end_resp_t *in, uint8_t *raw, uint32_t len) {
	if (!raw || !in || len < 5)
		return PROTO_ERR_ARG;
	resp_init(raw, RESP_MOVE_END);
	raw[2] = in->frameId;
	raw[3] = in->status;
	resp_set_tail(raw, 4);
	return PROTO_OK;
}
uint8_t move_end_resp_calc_parity(const move_end_resp_t *in) {
	(void) in;
	return 0;
}
int move_end_resp_check_parity(const uint8_t *raw, uint32_t len) {
	(void) raw;
	(void) len;
	return 1;
}
int move_end_resp_set_parity(uint8_t *raw, uint32_t len) {
	(void) raw;
	(void) len;
	return 0;
}
move_end_resp_t move_end_resp_make_default(void) {
	move_end_resp_t d = { 0 };
	return d;
}
