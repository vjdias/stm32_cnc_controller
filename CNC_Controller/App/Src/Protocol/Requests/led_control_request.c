#include "Protocol/Requests/led_control_request.h"

// A requisição LED_CTRL (único LED) possui 9 bytes no total:
// [0]=0xAA, [1]=0x07, [2]=frameId, [3]=ledMask,
// [4]=LED1.mode, [5..6]=LED1.frequencyHz (BE16, frequência em Hz),
// [7]=paridade (XOR dos bytes 1..6), [8]=0x55

#define LED_CTRL_REQ_TOTAL_LEN 9u
#define LED_CTRL_PARITY_LAST_INDEX 6u
#define LED_CTRL_PARITY_INDEX 7u

int led_ctrl_req_decoder(const uint8_t *raw, uint32_t len, led_ctrl_req_t *out) {
    if (!raw || !out)
        return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_LED_CTRL, LED_CTRL_REQ_TOTAL_LEN);
    if (st != PROTO_OK)
        return st;
    out->frameId = raw[2];
    out->ledMask = raw[3];
    out->channel[0].mode = raw[4];
    out->channel[0].frequency = be16_read(raw + 5);
    return PROTO_OK;
}

uint8_t led_ctrl_req_calc_parity(const led_ctrl_req_t *in) {
    uint8_t bytes[6] = {
        REQ_LED_CTRL,
        in ? in->frameId : 0,
        in ? in->ledMask : 0,
        in ? in->channel[0].mode : 0,
        (uint8_t)((in ? in->channel[0].frequency : 0) >> 8),
        (uint8_t)((in ? in->channel[0].frequency : 0) & 0xFFu)
    };
    return xor_reduce_bytes(bytes, 6);
}

int led_ctrl_req_encoder(const led_ctrl_req_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < LED_CTRL_REQ_TOTAL_LEN)
        return PROTO_ERR_ARG;
    req_init(raw, REQ_LED_CTRL);
    raw[2] = in->frameId;
    raw[3] = in->ledMask;
    raw[4] = in->channel[0].mode;
    be16_write(raw + 5, in->channel[0].frequency);
    parity_set_byte_1N(raw, LED_CTRL_PARITY_LAST_INDEX, LED_CTRL_PARITY_INDEX);
    req_set_tail(raw, LED_CTRL_REQ_TOTAL_LEN - 1u);
    return PROTO_OK;
}

int led_ctrl_req_check_parity(const uint8_t *raw, uint32_t len) {
    if (frame_expect_req(raw, len, REQ_LED_CTRL, LED_CTRL_REQ_TOTAL_LEN) != PROTO_OK)
        return 0;
    return parity_check_byte_1N(raw, LED_CTRL_PARITY_LAST_INDEX, LED_CTRL_PARITY_INDEX);
}

int led_ctrl_req_set_parity(uint8_t *raw, uint32_t len) {
    if (!raw || len < LED_CTRL_REQ_TOTAL_LEN)
        return PROTO_ERR_ARG;
    return parity_set_byte_1N(raw, LED_CTRL_PARITY_LAST_INDEX, LED_CTRL_PARITY_INDEX);
}

led_ctrl_req_t led_ctrl_req_make_default(void) {
    led_ctrl_req_t d = { 0 };
    return d;
}
