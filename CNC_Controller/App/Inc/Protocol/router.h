// Roteador de frames SPI (AA..55 -> serviços, AB..54 <- respostas)
#pragma once
#include <stdint.h>
#include "frame_defs.h"

#ifdef __cplusplus
extern "C" {
#endif

// API simples do FIFO de respostas (opaca para quem chama)
typedef struct response_fifo_s response_fifo_t;

// Estado do roteador
typedef struct {
        uint8_t acc[64];   // buffer acumulador pequeno
	uint8_t idx;
	response_fifo_t *resp;
} router_t;

// Tipos de callback para despacho
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
    req_handler_fn on_set_origin;
    req_handler_fn on_encoder_status;
    req_handler_fn on_set_microsteps;
    req_handler_fn on_set_microsteps_axes;
    req_handler_fn on_test_hello;
} router_handlers_t;

void router_init(router_t *r, response_fifo_t *resp_fifo,
		const router_handlers_t *h);
// Alimenta bytes vindos dos callbacks de SPI RX DMA (meia/transferência completa)
void router_feed_bytes(router_t *r, const uint8_t *data, uint32_t len);

// API mínima do FIFO de respostas (lado produtor)
response_fifo_t* resp_fifo_create(void);
void resp_fifo_destroy(response_fifo_t *q);
// Empilha um frame de resposta completo (AB..54)
int resp_fifo_push(response_fifo_t *q, const uint8_t *frame, uint32_t len);
// Retira para transmissão (quem chama controla o buffer)
int resp_fifo_pop(response_fifo_t *q, uint8_t *out, uint32_t max_len);
int resp_fifo_count(const response_fifo_t *q);

#ifdef __cplusplus
}
#endif

