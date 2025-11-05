// DIAG_CTRL response (5 bytes) — 0x28
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint8_t flags; // eco dos flags aplicados
} diag_ctrl_resp_t;

int diag_ctrl_resp_encoder(const diag_ctrl_resp_t *in, uint8_t *raw, uint32_t len);
int diag_ctrl_resp_decoder(const uint8_t *raw, uint32_t len, diag_ctrl_resp_t *out);
diag_ctrl_resp_t diag_ctrl_resp_make_default(void);

