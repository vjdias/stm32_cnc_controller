#include "protocol/requests/led_control_request.h"

int led_ctrl_req_decoder(const uint8_t *raw, uint32_t len, led_ctrl_req_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_req(raw, len, REQ_LED_CTRL, 7);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	out->ledMask = raw[3];
	out->ledValue = raw[4];
	return PROTO_OK;
}

uint8_t led_ctrl_req_calc_parity(const led_ctrl_req_t *in) {
	uint8_t bytes[4] = { REQ_LED_CTRL, in ? in->frameId : 0,
			in ? in->ledMask : 0, in ? in->ledValue : 0 };
	return xor_reduce_bytes(bytes, 4);
}

int led_ctrl_req_encoder(const led_ctrl_req_t *in, uint8_t *raw, uint32_t len) {
	if (!raw || !in || len < 7)
		return PROTO_ERR_ARG;
	req_init(raw, REQ_LED_CTRL);
	raw[2] = in->frameId;
	raw[3] = in->ledMask;
	raw[4] = in->ledValue;
	parity_set_byte_1N(raw, 4, 5);
	req_set_tail(raw, 6);
	return PROTO_OK;
}

int led_ctrl_req_check_parity(const uint8_t *raw, uint32_t len) {
	if (frame_expect_req(raw, len, REQ_LED_CTRL, 7) != PROTO_OK)
		return 0;
	return parity_check_byte_1N(raw, 4, 5);
}

int led_ctrl_req_set_parity(uint8_t *raw, uint32_t len) {
	if (!raw || len < 7)
		return PROTO_ERR_ARG;
	return parity_set_byte_1N(raw, 4, 5);
}

led_ctrl_req_t led_ctrl_req_make_default(void) {
	led_ctrl_req_t d = { 0 };
	return d;
}
