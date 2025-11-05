#include "Protocol/Responses/motion_estimate_response.h"

int motion_estimate_resp_encoder(const motion_estimate_resp_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 16) return PROTO_ERR_ARG;
    resp_init(raw, RESP_MOTION_ESTIMATE);
    raw[2] = in->frameId;
    be32_write(&raw[3],  (uint32_t)in->avgAccel);
    be32_write(&raw[7],  (uint32_t)in->avgCruise);
    be32_write(&raw[11], (uint32_t)in->avgDecel);
    resp_set_tail(raw, 15);
    return PROTO_OK;
}

int motion_estimate_resp_decoder(const uint8_t *raw, uint32_t len, motion_estimate_resp_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_resp(raw, len, RESP_MOTION_ESTIMATE, 16);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->avgAccel  = (int32_t)be32_read(&raw[3]);
    out->avgCruise = (int32_t)be32_read(&raw[7]);
    out->avgDecel  = (int32_t)be32_read(&raw[11]);
    return PROTO_OK;
}

motion_estimate_resp_t motion_estimate_resp_make_default(void) {
    motion_estimate_resp_t d = {0}; return d;
}

