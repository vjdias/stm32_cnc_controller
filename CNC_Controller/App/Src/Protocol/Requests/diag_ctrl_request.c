#include "Protocol/Requests/diag_ctrl_request.h"

int diag_ctrl_req_decoder(const uint8_t *raw, uint32_t len, diag_ctrl_req_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_DIAG_CTRL, 5);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->flags = raw[3];
    return PROTO_OK;
}

int diag_ctrl_req_encoder(const diag_ctrl_req_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 5) return PROTO_ERR_ARG;
    req_init(raw, REQ_DIAG_CTRL);
    raw[2] = in->frameId;
    raw[3] = in->flags;
    req_set_tail(raw, 4);
    return PROTO_OK;
}

diag_ctrl_req_t diag_ctrl_req_make_default(void) { diag_ctrl_req_t d = {0}; return d; }

