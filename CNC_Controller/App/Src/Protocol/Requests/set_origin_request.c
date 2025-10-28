#include "Protocol/Requests/set_origin_request.h"

int set_origin_req_decoder(const uint8_t *raw, uint32_t len, set_origin_req_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_SET_ORIGIN, 6);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->mask = raw[3] & 0x07u;
    out->mode = raw[4];
    return PROTO_OK;
}
int set_origin_req_encoder(const set_origin_req_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 6) return PROTO_ERR_ARG;
    req_init(raw, REQ_SET_ORIGIN);
    raw[2] = in->frameId;
    raw[3] = in->mask & 0x07u;
    raw[4] = in->mode;
    req_set_tail(raw, 5);
    return PROTO_OK;
}
uint8_t set_origin_req_calc_parity(const set_origin_req_t *in) {
    (void)in; return 0;
}
int set_origin_req_check_parity(const uint8_t *raw, uint32_t len) {
    (void)raw; (void)len; return 1;
}
int set_origin_req_set_parity(uint8_t *raw, uint32_t len) {
    (void)raw; (void)len; return 0;
}
set_origin_req_t set_origin_req_make_default(void) {
    set_origin_req_t d = {0}; return d;
}

