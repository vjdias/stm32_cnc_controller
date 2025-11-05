// DIAG_CTRL (5 bytes) — 0x28
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint8_t flags; // bit0: SPD SWO enable
} diag_ctrl_req_t;

int diag_ctrl_req_decoder(const uint8_t *raw, uint32_t len, diag_ctrl_req_t *out);
int diag_ctrl_req_encoder(const diag_ctrl_req_t *in, uint8_t *raw, uint32_t len);
diag_ctrl_req_t diag_ctrl_req_make_default(void);

