#include "Protocol/Requests/set_microsteps_axes_request.h"

int set_microsteps_axes_req_decoder(const uint8_t *raw, uint32_t len,
                                    set_microsteps_axes_req_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_SET_MICROSTEPS_AX, 7);
    if (st != PROTO_OK) return st;
    out->frameId = raw[2];
    out->ms_x = raw[3];
    out->ms_y = raw[4];
    out->ms_z = raw[5];
    return PROTO_OK;
}

int set_microsteps_axes_req_encoder(const set_microsteps_axes_req_t *in,
                                    uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 7) return PROTO_ERR_ARG;
    req_init(raw, REQ_SET_MICROSTEPS_AX);
    raw[2] = in->frameId;
    raw[3] = in->ms_x;
    raw[4] = in->ms_y;
    raw[5] = in->ms_z;
    req_set_tail(raw, 6);
    return PROTO_OK;
}

set_microsteps_axes_req_t set_microsteps_axes_req_make_default(void) {
    set_microsteps_axes_req_t d = {0};
    return d;
}

