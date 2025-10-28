#include "Protocol/Responses/set_origin_response.h"

int set_origin_resp_encoder(const set_origin_resp_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 16) return PROTO_ERR_ARG;
    resp_init(raw, RESP_SET_ORIGIN);
    raw[2] = in->frameId;
    be32_write(&raw[3], (uint32_t)in->x0);
    be32_write(&raw[7], (uint32_t)in->y0);
    be32_write(&raw[11], (uint32_t)in->z0);
    resp_set_tail(raw, 15);
    return PROTO_OK;
}

int set_origin_resp_decoder(const uint8_t *raw, uint32_t len, set_origin_resp_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_resp(raw, len, RESP_SET_ORIGIN, 16);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->x0 = (int32_t)be32_read(&raw[3]);
    out->y0 = (int32_t)be32_read(&raw[7]);
    out->z0 = (int32_t)be32_read(&raw[11]);
    return PROTO_OK;
}

set_origin_resp_t set_origin_resp_make_default(void) {
    set_origin_resp_t d = {0}; return d;
}

