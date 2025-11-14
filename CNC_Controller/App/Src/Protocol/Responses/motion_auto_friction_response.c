#include "Protocol/Responses/motion_auto_friction_response.h"

int motion_auto_friction_resp_encoder(const motion_auto_friction_resp_t *in,
                                      uint8_t *raw, uint32_t len) {
    if (!raw || !in || len < 9) return PROTO_ERR_ARG;
    resp_init(raw, RESP_MOTION_AUTO_FRICTION);
    raw[2] = in->frameId;
    raw[3] = in->status;
    raw[4] = in->revolutions;
    raw[5] = in->friction_segment;
    be16_write(&raw[6], in->sample_limit);
    resp_set_tail(raw, 8);
    return PROTO_OK;
}
