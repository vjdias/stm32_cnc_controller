// SET_MICROSTEPS (5 bytes) — 0x26
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint16_t microsteps; // aplica globalmente; 0 => 256; caso contrário 1..255
} set_microsteps_req_t;

int set_microsteps_req_decoder(const uint8_t *raw, uint32_t len, set_microsteps_req_t *out);
int set_microsteps_req_encoder(const set_microsteps_req_t *in, uint8_t *raw, uint32_t len);
uint8_t set_microsteps_req_calc_parity(const set_microsteps_req_t *in); // N/D
int set_microsteps_req_check_parity(const uint8_t *raw, uint32_t len); // N/D
int set_microsteps_req_set_parity(uint8_t *raw, uint32_t len); // N/D
set_microsteps_req_t set_microsteps_req_make_default(void);

