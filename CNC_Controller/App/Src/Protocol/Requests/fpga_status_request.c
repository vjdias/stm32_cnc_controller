#include "Protocol/Requests/fpga_status_request.h"

int fpga_status_req_decoder(const uint8_t *raw, uint32_t len,
		fpga_status_req_t *out) {
	if (!raw || !out)
		return PROTO_ERR_ARG;
	int st = frame_expect_req(raw, len, REQ_FPGA_STATUS, 4);
	if (st != PROTO_OK)
		return st;
	out->frameId = raw[2];
	return PROTO_OK;
}

int fpga_status_req_encoder(const fpga_status_req_t *in, uint8_t *raw,
		uint32_t len) {
	if (!raw || !in || len < 4)
		return PROTO_ERR_ARG;
	req_init(raw, REQ_FPGA_STATUS);
	raw[2] = in->frameId;
	req_set_tail(raw, 3);
	return PROTO_OK;
}

uint8_t fpga_status_req_calc_parity(const fpga_status_req_t *in) {
	(void) in;
	return 0;
}
int fpga_status_req_check_parity(const uint8_t *raw, uint32_t len) {
	(void) raw;
	(void) len;
	return 1;
}
int fpga_status_req_set_parity(uint8_t *raw, uint32_t len) {
	(void) raw;
	(void) len;
	return 0;
}
fpga_status_req_t fpga_status_req_make_default(void) {
	fpga_status_req_t d = { 0 };
	return d;
}
