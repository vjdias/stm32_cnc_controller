// Requisição LED_CTRL (9 bytes úteis, 42 bytes com padding) — 0x07
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

#define LED_CTRL_REQ_TOTAL_LEN 9u
#define LED_CTRL_REQ_PADDED_TOTAL_LEN 42u

#define LED_CTRL_CHANNEL_COUNT 1u

typedef struct {
    uint8_t mode;        // Valor conforme LED_MODE_*
    uint16_t frequency;  // Frequência de pisca em centi-Hz (ignorada salvo modo=BLINK)
} led_ctrl_channel_cfg_t;

typedef struct {
    uint8_t frameId;
    uint8_t ledMask; // bit0=LED1
    led_ctrl_channel_cfg_t channel[LED_CTRL_CHANNEL_COUNT];
} led_ctrl_req_t;

// Bits da máscara
#define LED_MASK_LED1 0x01u

// Modos aceitos pelo firmware
enum {
    LED_MODE_OFF = 0u,
    LED_MODE_ON = 1u,
    LED_MODE_BLINK = 2u,
};

int led_ctrl_req_decoder(const uint8_t *raw, uint32_t len, led_ctrl_req_t *out);
int led_ctrl_req_encoder(const led_ctrl_req_t *in, uint8_t *raw, uint32_t len);
uint8_t led_ctrl_req_calc_parity(const led_ctrl_req_t *in);
int led_ctrl_req_check_parity(const uint8_t *raw, uint32_t len);
int led_ctrl_req_set_parity(uint8_t *raw, uint32_t len);
led_ctrl_req_t led_ctrl_req_make_default(void);
