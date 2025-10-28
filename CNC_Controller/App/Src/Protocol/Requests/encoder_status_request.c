#include "Protocol/Requests/encoder_status_request.h"

int encoder_status_req_decoder(const uint8_t *raw, uint32_t len, encoder_status_req_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_ENCODER_STATUS, 4);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    return PROTO_OK;
}
int encoder_status_req_encoder(const encoder_status_req_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 4) return PROTO_ERR_ARG;
    req_init(raw, REQ_ENCODER_STATUS);
    raw[2] = in->frameId;
    req_set_tail(raw, 3);
    return PROTO_OK;
}
uint8_t encoder_status_req_calc_parity(const encoder_status_req_t *in) {
    (void)in; return 0;
}
int encoder_status_req_check_parity(const uint8_t *raw, uint32_t len) {
    (void)raw; (void)len; return 1;
}
int encoder_status_req_set_parity(uint8_t *raw, uint32_t len) {
    (void)raw; (void)len; return 0;
}
encoder_status_req_t encoder_status_req_make_default(void) {
    encoder_status_req_t d = {0}; return d;
}

