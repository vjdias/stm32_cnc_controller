// SET_ENC_PPR (9 bytes) — 0x29
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint8_t axis;   // 0=X,1=Y,2=Z
    uint32_t ppr;   // Pulsos do encoder por revolução (quadratura já considerada)
} set_enc_ppr_req_t;

int set_enc_ppr_req_decoder(const uint8_t *raw, uint32_t len, set_enc_ppr_req_t *out);
int set_enc_ppr_req_encoder(const set_enc_ppr_req_t *in, uint8_t *raw, uint32_t len);
set_enc_ppr_req_t set_enc_ppr_req_make_default(void);

