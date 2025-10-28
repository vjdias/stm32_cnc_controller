// SET_ORIGIN (6 bytes) — 0x24
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint8_t mask;   // bit0=X, bit1=Y, bit2=Z
    uint8_t mode;   // 0=start, 1=initial
} set_origin_req_t;

int set_origin_req_decoder(const uint8_t *raw, uint32_t len, set_origin_req_t *out);
int set_origin_req_encoder(const set_origin_req_t *in, uint8_t *raw, uint32_t len);
uint8_t set_origin_req_calc_parity(const set_origin_req_t *in); // N/D
int set_origin_req_check_parity(const uint8_t *raw, uint32_t len); // N/D
int set_origin_req_set_parity(uint8_t *raw, uint32_t len); // N/D
set_origin_req_t set_origin_req_make_default(void);

