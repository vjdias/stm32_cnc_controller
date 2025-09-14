#include "Protocol/Requests/led_control_request.h"

// New LED_CTRL (RGB) request is 9 bytes total:
// [0]=0xAA, [1]=0x07, [2]=frameId, [3]=ledMask, [4]=R, [5]=G, [6]=B,
// [7]=parity(byte XOR over 1..6), [8]=0x55

int led_ctrl_req_decoder(const uint8_t *raw, uint32_t len, led_ctrl_req_t *out) {
    if (!raw || !out)
        return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_LED_CTRL, 9);
    if (st != PROTO_OK)
        return st;
    out->frameId = raw[2];
    out->ledMask = raw[3];
    out->r = raw[4];
    out->g = raw[5];
    out->b = raw[6];
    return PROTO_OK;
}

uint8_t led_ctrl_req_calc_parity(const led_ctrl_req_t *in) {
    uint8_t bytes[6] = { REQ_LED_CTRL, in ? in->frameId : 0,
                         in ? in->ledMask : 0, in ? in->r : 0,
                         in ? in->g : 0, in ? in->b : 0 };
    return xor_reduce_bytes(bytes, 6);
}

int led_ctrl_req_encoder(const led_ctrl_req_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 9)
        return PROTO_ERR_ARG;
    req_init(raw, REQ_LED_CTRL);
    raw[2] = in->frameId;
    raw[3] = in->ledMask;
    raw[4] = in->r;
    raw[5] = in->g;
    raw[6] = in->b;
    parity_set_byte_1N(raw, 6, 7);
    req_set_tail(raw, 8);
    return PROTO_OK;
}

int led_ctrl_req_check_parity(const uint8_t *raw, uint32_t len) {
    if (frame_expect_req(raw, len, REQ_LED_CTRL, 9) != PROTO_OK)
        return 0;
    return parity_check_byte_1N(raw, 6, 7);
}

int led_ctrl_req_set_parity(uint8_t *raw, uint32_t len) {
    if (!raw || len < 9)
        return PROTO_ERR_ARG;
    return parity_set_byte_1N(raw, 6, 7);
}

led_ctrl_req_t led_ctrl_req_make_default(void) {
    led_ctrl_req_t d = { 0 };
    return d;
}
