#include "Protocol/Responses/diag_ctrl_response.h"

int diag_ctrl_resp_encoder(const diag_ctrl_resp_t *in, uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 5) return PROTO_ERR_ARG;
    resp_init(raw, RESP_DIAG_CTRL);
    raw[2] = in->frameId;
    raw[3] = in->flags;
    resp_set_tail(raw, 4);
    return PROTO_OK;
}

int diag_ctrl_resp_decoder(const uint8_t *raw, uint32_t len, diag_ctrl_resp_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_resp(raw, len, RESP_DIAG_CTRL, 5);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->flags = raw[3];
    return PROTO_OK;
}

diag_ctrl_resp_t diag_ctrl_resp_make_default(void) { diag_ctrl_resp_t d = {0}; return d; }

