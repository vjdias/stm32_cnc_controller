#pragma once
#include <stdint.h>
#include "spi.h"
#include "Protocol/router.h"
#include "Protocol/frame_defs.h"

// ---- Parâmetros de protocolo/DMA (mínimo) ----
#ifndef APP_SPI_DMA_BUF_LEN
#define APP_SPI_DMA_BUF_LEN 42u
#endif

#ifndef REQ_HEADER
#define REQ_HEADER 0xAAu
#endif

#ifndef REQ_TAIL
#define REQ_TAIL   0x55u
#endif

// 42 × 0x3C => cliente apenas consulta respostas (não envia requisição)
#ifndef SPI_POLL_BYTE
#define SPI_POLL_BYTE 0x3Cu
#endif

// Filler quando não há resposta pronta
#ifndef SPI_FILL_BYTE
#define SPI_FILL_BYTE 0xA5u
#endif

// Permite trocar a instância sem tocar no código
#ifndef APP_SPI_INSTANCE
#define APP_SPI_INSTANCE SPI2
#endif

// ---- Estados mínimos do enlace SPI ----
typedef enum {
    APP_SPI_READY   = 0, // pronto, sem resposta enfileirada
    APP_SPI_BUSY    = 1, // DMA em andamento
    APP_SPI_PENDING = 2, // há resposta preparada para o próximo round
} app_spi_state_t;

// ---- FIFO de respostas (forward decl. para não puxar dependências pesadas) ----
//typedef struct response_fifo response_fifo_t;
//response_fifo_t* resp_fifo_create(void);
//int  resp_fifo_pop(response_fifo_t *q, uint8_t *out, int maxlen);

// ---- API do app ----
void app_init(void);
void app_poll(void);

// Status/diag
app_spi_state_t app_spi_get_state(void);
uint8_t         app_spi_get_error(void);

// ---- Chamadas a partir dos callbacks do HAL no main.c ----
// Dentro do seu HAL_SPI_TxRxCpltCallback() chame isto:
void app_spi_isr_txrx_done(SPI_HandleTypeDef *hspi);

int app_resp_push(const uint8_t *frame, uint32_t len);
//static inline void spi_post_dma_rx_fifo_sanity(SPI_HandleTypeDef *hspi);


