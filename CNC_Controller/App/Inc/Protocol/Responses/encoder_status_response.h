// ENCODER_STATUS response (20 bytes) — 0x25
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint8_t pidErrX, pidErrY, pidErrZ;
    uint8_t delta;
    int32_t absX, absY, absZ;
} encoder_status_resp_t;

int encoder_status_resp_encoder(const encoder_status_resp_t *in, uint8_t *raw, uint32_t len);
int encoder_status_resp_decoder(const uint8_t *raw, uint32_t len, encoder_status_resp_t *out);
encoder_status_resp_t encoder_status_resp_make_default(void);

