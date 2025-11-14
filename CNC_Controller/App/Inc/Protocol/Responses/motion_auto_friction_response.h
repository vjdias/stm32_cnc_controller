// RESP_MOTION_AUTO_FRICTION (0x69) — ACK do comando de autoanálise
#pragma once
#include <stdint.h>
#include "../frame_defs.h"

enum {
    MOTION_AUTO_FRICTION_STATUS_OK = 0u,
    MOTION_AUTO_FRICTION_STATUS_BUSY = 1u,
    MOTION_AUTO_FRICTION_STATUS_INVALID = 2u,
    MOTION_AUTO_FRICTION_STATUS_QUEUE_FULL = 3u,
    MOTION_AUTO_FRICTION_STATUS_UNAVAILABLE = 4u,
    MOTION_AUTO_FRICTION_STATUS_ERROR = 255u,
};

typedef struct {
    uint8_t  frameId;
    uint8_t  status;
    uint8_t  revolutions;
    uint8_t  friction_segment;
    uint16_t sample_limit;
} motion_auto_friction_resp_t;

int motion_auto_friction_resp_encoder(const motion_auto_friction_resp_t *in,
                                      uint8_t *raw, uint32_t len);
