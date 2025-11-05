#include "Protocol/Responses/model_run_response.h"

int model_run_resp_encoder(const model_run_resp_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 5) return PROTO_ERR_ARG;
    resp_init(raw, RESP_MODEL_RUN);
    raw[2] = in->frameId;
    raw[3] = in->status;
    resp_set_tail(raw, 4);
    return PROTO_OK;
}

int model_run_resp_decoder(const uint8_t *raw, uint32_t len, model_run_resp_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_resp(raw, len, RESP_MODEL_RUN, 5);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->status = raw[3];
    return PROTO_OK;
}

model_run_resp_t model_run_resp_make_default(void) { model_run_resp_t d = {0}; return d; }

