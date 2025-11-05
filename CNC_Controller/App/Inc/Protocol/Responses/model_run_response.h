// MODEL_RUN response (5 bytes) — 0x2A
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint8_t status; // 0=OK, !0=erro
} model_run_resp_t;

int model_run_resp_encoder(const model_run_resp_t *in, uint8_t *raw, uint32_t len);
int model_run_resp_decoder(const uint8_t *raw, uint32_t len, model_run_resp_t *out);
model_run_resp_t model_run_resp_make_default(void);

