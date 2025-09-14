// LED_CTRL request (7 bytes) — 0x07
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t frameId;
    uint8_t ledMask; // bit0=R, bit1=G, bit2=B
    uint8_t r;
    uint8_t g;
    uint8_t b;
} led_ctrl_req_t;

// Mask bits mapping (current board)
//  - bit0: on-board user LED (monochrome)
#define LED_MASK_USER 0x01u

// Value semantics for monochrome LED
#define LED_VAL_OFF   0x00u
#define LED_VAL_ON    0x01u

int led_ctrl_req_decoder(const uint8_t *raw, uint32_t len, led_ctrl_req_t *out);
int led_ctrl_req_encoder(const led_ctrl_req_t *in, uint8_t *raw, uint32_t len);
uint8_t led_ctrl_req_calc_parity(const led_ctrl_req_t *in);
int led_ctrl_req_check_parity(const uint8_t *raw, uint32_t len);
int led_ctrl_req_set_parity(uint8_t *raw, uint32_t len);
led_ctrl_req_t led_ctrl_req_make_default(void);

// Mask bits mapping (RGB channels)
#define LED_MASK_R 0x01u
#define LED_MASK_G 0x02u
#define LED_MASK_B 0x04u
