// SET_MICROSTEPS_AX (7 bytes) — 0x27
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint8_t ms_x;
    uint8_t ms_y;
    uint8_t ms_z;
} set_microsteps_axes_req_t;

int set_microsteps_axes_req_decoder(const uint8_t *raw, uint32_t len,
                                    set_microsteps_axes_req_t *out);
int set_microsteps_axes_req_encoder(const set_microsteps_axes_req_t *in,
                                    uint8_t *raw, uint32_t len);
set_microsteps_axes_req_t set_microsteps_axes_req_make_default(void);

