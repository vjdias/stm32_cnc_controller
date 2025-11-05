#include "Protocol/Responses/set_enc_ppr_response.h"

int set_enc_ppr_resp_encoder(const set_enc_ppr_resp_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 9) return PROTO_ERR_ARG;
    resp_init(raw, RESP_SET_ENC_PPR);
    raw[2] = in->frameId;
    raw[3] = in->axis;
    be32_write(&raw[4], in->ppr);
    resp_set_tail(raw, 8);
    return PROTO_OK;
}

int set_enc_ppr_resp_decoder(const uint8_t *raw, uint32_t len, set_enc_ppr_resp_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_resp(raw, len, RESP_SET_ENC_PPR, 9);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->axis = raw[3];
    out->ppr = be32_read(&raw[4]);
    return PROTO_OK;
}

set_enc_ppr_resp_t set_enc_ppr_resp_make_default(void) { set_enc_ppr_resp_t d = {0}; return d; }

