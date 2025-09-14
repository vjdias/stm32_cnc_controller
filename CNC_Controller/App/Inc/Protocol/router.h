// SPI frame router (AA..55 -> services, AB..54 <- responses)
#pragma once
#include <stdint.h>
#include "frame_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Simple response FIFO API (opaque to callers)
typedef struct response_fifo_s response_fifo_t;

// Router state
typedef struct {
	uint8_t acc[64];   // small accumulator buffer
	uint8_t idx;
	response_fifo_t *resp;
} router_t;

// Callback types for dispatch
typedef void (*req_handler_fn)(router_t *r, const uint8_t *frame, uint32_t len);

typedef struct {
	req_handler_fn on_move_queue_add;
	req_handler_fn on_move_queue_status;
	req_handler_fn on_start_move;
	req_handler_fn on_move_home;
	req_handler_fn on_move_probe_level;
	req_handler_fn on_move_end;
	req_handler_fn on_led_ctrl;
	req_handler_fn on_fpga_status;
} router_handlers_t;

void router_init(router_t *r, response_fifo_t *resp_fifo,
		const router_handlers_t *h);
// Feed bytes from SPI RX DMA callbacks (half/full complete)
void router_feed_bytes(router_t *r, const uint8_t *data, uint32_t len);

// Response FIFO minimal API (producer side)
response_fifo_t* resp_fifo_create(void);
void resp_fifo_destroy(response_fifo_t *q);
// Push a fully formed response frame (AB..54)
int resp_fifo_push(response_fifo_t *q, const uint8_t *frame, uint32_t len);
// Pop for transmission (caller owns buffer lifetime)
int resp_fifo_pop(response_fifo_t *q, uint8_t *out, uint32_t max_len);
int resp_fifo_count(const response_fifo_t *q);

#ifdef __cplusplus
}
#endif

