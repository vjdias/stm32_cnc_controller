#include "Protocol/Responses/led_control_response.h"

int led_ctrl_resp_decoder(const uint8_t *raw, uint32_t len,
		led_ctrl_resp_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_resp(raw, len, RESP_LED_CTRL, 7);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->ledMask = raw[3];
	out->status = raw[4];
	return PROTO_OK;
}
uint8_t led_ctrl_resp_calc_parity(const led_ctrl_resp_t *in) {
	uint8_t b[4] = { RESP_LED_CTRL, in ? in->frameId : 0, in ? in->ledMask : 0,
			in ? in->status : 0 };
	return xor_reduce_bytes(b, 4);
}
int led_ctrl_resp_encoder(const led_ctrl_resp_t *in, uint8_t *raw, uint32_t len) {
	if (!raw || !in || len < 7)
		return PROTO_ERR_ARG;
	resp_init(raw, RESP_LED_CTRL);
	raw[2] = in->frameId;
	raw[3] = in->ledMask;
	raw[4] = in->status;
	parity_set_byte_1N(raw, 4, 5);
	resp_set_tail(raw, 6);
	return PROTO_OK;
}
int led_ctrl_resp_check_parity(const uint8_t *raw, uint32_t len) {
	if (frame_expect_resp(raw, len, RESP_LED_CTRL, 7) != PROTO_OK)
		return 0;
	return parity_check_byte_1N(raw, 4, 5);
}
int led_ctrl_resp_set_parity(uint8_t *raw, uint32_t len) {
	if (!raw || len < 7)
		return PROTO_ERR_ARG;
	return parity_set_byte_1N(raw, 4, 5);
}
led_ctrl_resp_t led_ctrl_resp_make_default(void) {
	led_ctrl_resp_t d = { 0 };
	return d;
}
