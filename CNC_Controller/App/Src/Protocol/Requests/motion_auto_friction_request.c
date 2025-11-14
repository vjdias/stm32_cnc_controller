#include "Protocol/Requests/motion_auto_friction_request.h"

int motion_auto_friction_req_decoder(const uint8_t *raw, uint32_t len,
                                     motion_auto_friction_req_t *out) {
    if (!raw || !out) return PROTO_ERR_ARG;
    int st = frame_expect_req(raw, len, REQ_MOTION_AUTO_FRICTION, 8);
    if (st != PROTO_OK) return st;
    out->frameId          = raw[2];
    out->revolutions      = raw[3];
    out->friction_segment = raw[4];
    out->sample_limit     = be16_read(&raw[5]);
    return PROTO_OK;
}

int motion_auto_friction_req_encoder(const motion_auto_friction_req_t *in,
                                     uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 8) return PROTO_ERR_ARG;
    req_init(raw, REQ_MOTION_AUTO_FRICTION);
    raw[2] = in->frameId;
    raw[3] = in->revolutions;
    raw[4] = in->friction_segment;
    be16_write(&raw[5], in->sample_limit);
    req_set_tail(raw, 7);
    return PROTO_OK;
}

int motion_auto_friction_req_check_parity(const uint8_t *raw, uint32_t len) {
    (void)raw; (void)len;
    return 1;
}

int motion_auto_friction_req_set_parity(uint8_t *raw, uint32_t len) {
    (void)raw; (void)len;
    return 0;
}

motion_auto_friction_req_t motion_auto_friction_req_make_default(void) {
    motion_auto_friction_req_t d = {0};
    return d;
}
