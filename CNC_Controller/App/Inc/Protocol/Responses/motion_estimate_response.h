// MOTION_ESTIMATE response (16 bytes) — 0x27
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    int32_t avgAccel;      // steps/s^2
    int32_t avgCruise;     // steps/s
    int32_t avgDecel;      // steps/s^2 (magnitude positiva)
} motion_estimate_resp_t;

int motion_estimate_resp_encoder(const motion_estimate_resp_t *in, uint8_t *raw, uint32_t len);
int motion_estimate_resp_decoder(const uint8_t *raw, uint32_t len, motion_estimate_resp_t *out);
motion_estimate_resp_t motion_estimate_resp_make_default(void);

