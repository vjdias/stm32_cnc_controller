#include "Protocol/Responses/encoder_status_response.h"

int encoder_status_resp_encoder(const encoder_status_resp_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 20) return PROTO_ERR_ARG;
    resp_init(raw, RESP_ENCODER_STATUS);
    raw[2] = in->frameId;
    raw[3] = in->pidErrX;
    raw[4] = in->pidErrY;
    raw[5] = in->pidErrZ;
    raw[6] = in->delta;
    be32_write(&raw[7],  (uint32_t)in->absX);
    be32_write(&raw[11], (uint32_t)in->absY);
    be32_write(&raw[15], (uint32_t)in->absZ);
    resp_set_tail(raw, 19);
    return PROTO_OK;
}

int encoder_status_resp_decoder(const uint8_t *raw, uint32_t len, encoder_status_resp_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_resp(raw, len, RESP_ENCODER_STATUS, 20);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->pidErrX = raw[3];
    out->pidErrY = raw[4];
    out->pidErrZ = raw[5];
    out->delta   = raw[6];
    out->absX = (int32_t)be32_read(&raw[7]);
    out->absY = (int32_t)be32_read(&raw[11]);
    out->absZ = (int32_t)be32_read(&raw[15]);
    return PROTO_OK;
}

encoder_status_resp_t encoder_status_resp_make_default(void) {
    encoder_status_resp_t d = {0}; return d;
}

