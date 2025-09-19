// FPGA_STATUS (4 bytes) — 0x20
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
} fpga_status_req_t;

int fpga_status_req_decoder(const uint8_t *raw, uint32_t len,
		fpga_status_req_t *out);
int fpga_status_req_encoder(const fpga_status_req_t *in, uint8_t *raw,
		uint32_t len);
uint8_t fpga_status_req_calc_parity(const fpga_status_req_t *in); // N/D
int fpga_status_req_check_parity(const uint8_t *raw, uint32_t len); // N/D
int fpga_status_req_set_parity(uint8_t *raw, uint32_t len); // N/D
fpga_status_req_t fpga_status_req_make_default(void);

