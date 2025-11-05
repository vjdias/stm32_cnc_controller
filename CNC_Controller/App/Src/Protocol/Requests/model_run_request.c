#include "Protocol/Requests/model_run_request.h"

int model_run_req_decoder(const uint8_t *raw, uint32_t len, model_run_req_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_MODEL_RUN, 12);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->axis = raw[3];
    out->dir = raw[4];
    out->freq_sps = be32_read(&raw[5]);
    out->turns = (uint16_t)be16_read(&raw[9]);
    return PROTO_OK;
}

int model_run_req_encoder(const model_run_req_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 12) return PROTO_ERR_ARG;
    req_init(raw, REQ_MODEL_RUN);
    raw[2] = in->frameId;
    raw[3] = in->axis;
    raw[4] = in->dir;
    be32_write(&raw[5], in->freq_sps);
    be16_write(&raw[9], in->turns);
    req_set_tail(raw, 11);
    return PROTO_OK;
}

model_run_req_t model_run_req_make_default(void) { model_run_req_t d = {0}; return d; }
