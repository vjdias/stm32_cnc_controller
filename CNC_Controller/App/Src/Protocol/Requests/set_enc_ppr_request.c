#include "Protocol/Requests/set_enc_ppr_request.h"

int set_enc_ppr_req_decoder(const uint8_t *raw, uint32_t len, set_enc_ppr_req_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_SET_ENC_PPR, 9);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->axis = raw[3];
    out->ppr = be32_read(&raw[4]);
    return PROTO_OK;
}

int set_enc_ppr_req_encoder(const set_enc_ppr_req_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 9) return PROTO_ERR_ARG;
    req_init(raw, REQ_SET_ENC_PPR);
    raw[2] = in->frameId;
    raw[3] = in->axis;
    be32_write(&raw[4], in->ppr);
    req_set_tail(raw, 8);
    return PROTO_OK;
}

set_enc_ppr_req_t set_enc_ppr_req_make_default(void) { set_enc_ppr_req_t d = {0}; return d; }

