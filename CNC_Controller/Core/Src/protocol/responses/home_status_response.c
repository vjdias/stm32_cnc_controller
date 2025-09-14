#include "protocol/responses/home_status_response.h"

int home_status_resp_decoder(const uint8_t *raw, uint32_t len,
		home_status_resp_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_resp(raw, len, RESP_HOME_STATUS, 18);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->axisMask = raw[3];
	out->posRelX = be16_read(&raw[4]);
	out->homeOffX = be16_read(&raw[6]);
	out->posRelY = be16_read(&raw[8]);
	out->homeOffY = be16_read(&raw[10]);
	out->posRelZ = be16_read(&raw[12]);
	out->homeOffZ = be16_read(&raw[14]);
	return PROTO_OK;
}
uint8_t home_status_resp_calc_parity(const home_status_resp_t *in) {
	uint8_t b[15];
	b[0] = RESP_HOME_STATUS;
	b[1] = in ? in->frameId : 0;
	b[2] = in ? in->axisMask : 0;
	be16_write(&b[3], in ? in->posRelX : 0);
	be16_write(&b[5], in ? in->homeOffX : 0);
	be16_write(&b[7], in ? in->posRelY : 0);
	be16_write(&b[9], in ? in->homeOffY : 0);
	be16_write(&b[11], in ? in->posRelZ : 0);
	be16_write(&b[13], in ? in->homeOffZ : 0);
	return xor_reduce_bytes(b, 15);
}
int home_status_resp_encoder(const home_status_resp_t *in, uint8_t *raw,
		uint32_t len) {
	if (!raw || !in || len < 18)
		return PROTO_ERR_ARG;
	resp_init(raw, RESP_HOME_STATUS);
	raw[2] = in->frameId;
	raw[3] = in->axisMask;
	be16_write(&raw[4], in->posRelX);
	be16_write(&raw[6], in->homeOffX);
	be16_write(&raw[8], in->posRelY);
	be16_write(&raw[10], in->homeOffY);
	be16_write(&raw[12], in->posRelZ);
	be16_write(&raw[14], in->homeOffZ);
	parity_set_byte_1N(raw, 15, 16);
	resp_set_tail(raw, 17);
	return PROTO_OK;
}
int home_status_resp_check_parity(const uint8_t *raw, uint32_t len) {
	if (frame_expect_resp(raw, len, RESP_HOME_STATUS, 18) != PROTO_OK)
		return 0;
	return parity_check_byte_1N(raw, 15, 16);
}
int home_status_resp_set_parity(uint8_t *raw, uint32_t len) {
	if (!raw || len < 18)
		return PROTO_ERR_ARG;
	return parity_set_byte_1N(raw, 15, 16);
}
home_status_resp_t home_status_resp_make_default(void) {
	home_status_resp_t d = { 0 };
	return d;
}
