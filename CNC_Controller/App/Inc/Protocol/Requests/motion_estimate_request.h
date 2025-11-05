// MOTION_ESTIMATE (4 bytes) — 0x27
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
} motion_estimate_req_t;

int motion_estimate_req_decoder(const uint8_t *raw, uint32_t len, motion_estimate_req_t *out);
int motion_estimate_req_encoder(const motion_estimate_req_t *in, uint8_t *raw, uint32_t len);
motion_estimate_req_t motion_estimate_req_make_default(void);

