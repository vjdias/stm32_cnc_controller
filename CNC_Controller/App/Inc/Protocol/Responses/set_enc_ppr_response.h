// SET_ENC_PPR response (9 bytes) — 0x29
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint8_t axis;
    uint32_t ppr;
} set_enc_ppr_resp_t;

int set_enc_ppr_resp_encoder(const set_enc_ppr_resp_t *in, uint8_t *raw, uint32_t len);
int set_enc_ppr_resp_decoder(const uint8_t *raw, uint32_t len, set_enc_ppr_resp_t *out);
set_enc_ppr_resp_t set_enc_ppr_resp_make_default(void);

