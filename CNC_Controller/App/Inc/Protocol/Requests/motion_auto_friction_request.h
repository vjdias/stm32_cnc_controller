// MOTION_AUTO_FRICTION (0x69) — dispara autoanálise de atrito
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

typedef struct {
    uint8_t  frameId;
    uint8_t  revolutions;      /* número de segmentos consecutivos (voltas) */
    uint8_t  friction_segment; /* índice (1-based) a partir do qual atrito liga */
    uint16_t sample_limit;     /* amostras por fase (antes/depois) */
} motion_auto_friction_req_t;

int motion_auto_friction_req_decoder(const uint8_t *raw, uint32_t len,
                                     motion_auto_friction_req_t *out);
int motion_auto_friction_req_encoder(const motion_auto_friction_req_t *in,
                                     uint8_t *raw, uint32_t len);
int motion_auto_friction_req_check_parity(const uint8_t *raw, uint32_t len);
int motion_auto_friction_req_set_parity(uint8_t *raw, uint32_t len);
motion_auto_friction_req_t motion_auto_friction_req_make_default(void);
