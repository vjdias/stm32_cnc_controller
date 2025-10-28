// ENCODER_STATUS (4 bytes) — 0x25
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
} encoder_status_req_t;

int encoder_status_req_decoder(const uint8_t *raw, uint32_t len, encoder_status_req_t *out);
int encoder_status_req_encoder(const encoder_status_req_t *in, uint8_t *raw, uint32_t len);
uint8_t encoder_status_req_calc_parity(const encoder_status_req_t *in); // N/D
int encoder_status_req_check_parity(const uint8_t *raw, uint32_t len); // N/D
int encoder_status_req_set_parity(uint8_t *raw, uint32_t len); // N/D
encoder_status_req_t encoder_status_req_make_default(void);

