#include "Protocol/Requests/set_microsteps_request.h"

int set_microsteps_req_decoder(const uint8_t *raw, uint32_t len, set_microsteps_req_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_SET_MICROSTEPS, 5);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->microsteps = (uint16_t)raw[3];
    return PROTO_OK;
}
int set_microsteps_req_encoder(const set_microsteps_req_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 5) return PROTO_ERR_ARG;
    req_init(raw, REQ_SET_MICROSTEPS);
    raw[2] = in->frameId;
    raw[3] = (uint8_t)(in->microsteps & 0xFFu);
    req_set_tail(raw, 4);
    return PROTO_OK;
}
uint8_t set_microsteps_req_calc_parity(const set_microsteps_req_t *in) {
    (void)in; return 0;
}
int set_microsteps_req_check_parity(const uint8_t *raw, uint32_t len) {
    (void)raw; (void)len; return 1;
}
int set_microsteps_req_set_parity(uint8_t *raw, uint32_t len) {
    (void)raw; (void)len; return 0;
}
set_microsteps_req_t set_microsteps_req_make_default(void) {
    set_microsteps_req_t d = {0}; return d;
}

