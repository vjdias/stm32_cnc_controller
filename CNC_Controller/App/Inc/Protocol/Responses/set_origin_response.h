// SET_ORIGIN response (16 bytes) — 0x24
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    int32_t x0;
    int32_t y0;
    int32_t z0;
} set_origin_resp_t;

int set_origin_resp_encoder(const set_origin_resp_t *in, uint8_t *raw, uint32_t len);
int set_origin_resp_decoder(const uint8_t *raw, uint32_t len, set_origin_resp_t *out);
set_origin_resp_t set_origin_resp_make_default(void);

