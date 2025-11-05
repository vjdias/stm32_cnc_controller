#include "Protocol/Requests/motion_estimate_request.h"

int motion_estimate_req_decoder(const uint8_t *raw, uint32_t len, motion_estimate_req_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_MOTION_ESTIMATE, 4);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    return PROTO_OK;
}

int motion_estimate_req_encoder(const motion_estimate_req_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 4) return PROTO_ERR_ARG;
    req_init(raw, REQ_MOTION_ESTIMATE);
    raw[2] = in->frameId;
    req_set_tail(raw, 3);
    return PROTO_OK;
}

motion_estimate_req_t motion_estimate_req_make_default(void) {
    motion_estimate_req_t d = {0}; return d;
}

