// LED_CTRL request (7 bytes) — 0x07
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
	uint8_t frameId;
	uint8_t ledMask;
	uint8_t ledValue;
} led_ctrl_req_t;

int led_ctrl_req_decoder(const uint8_t *raw, uint32_t len, led_ctrl_req_t *out);
int led_ctrl_req_encoder(const led_ctrl_req_t *in, uint8_t *raw, uint32_t len);
uint8_t led_ctrl_req_calc_parity(const led_ctrl_req_t *in);
int led_ctrl_req_check_parity(const uint8_t *raw, uint32_t len);
int led_ctrl_req_set_parity(uint8_t *raw, uint32_t len);
led_ctrl_req_t led_ctrl_req_make_default(void);

