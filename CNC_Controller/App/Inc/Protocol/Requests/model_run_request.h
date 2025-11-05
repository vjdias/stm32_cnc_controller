// MODEL_RUN (10 bytes) — 0x2A
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint8_t axis;   // 0=X,1=Y,2=Z
    uint8_t dir;    // 0=pos, 1=neg
    uint32_t freq_sps; // steps/s (fixo)
    uint16_t turns; // voltas completas (1..20)
} model_run_req_t;

int model_run_req_decoder(const uint8_t *raw, uint32_t len, model_run_req_t *out);
int model_run_req_encoder(const model_run_req_t *in, uint8_t *raw, uint32_t len);
model_run_req_t model_run_req_make_default(void);

