#include <string.h>
#include "spi.h"
#include "app.h"
#include "Protocol/frame_defs.h"
#include "Protocol/router.h"
#include "Services/service_adapters.h"
#include "Services/Led/led_service.h"
#include "Services/Log/log_service.h"
#include "Services/Test/test_spi_service.h"

#define APP_SPI_MAX_REQUEST_LEN    42u
#define APP_SPI_HANDSHAKE_BITS     8u
#define APP_SPI_HANDSHAKE_BYTES    (APP_SPI_HANDSHAKE_BITS / 8u)
#define APP_SPI_DMA_BUF_LEN        (APP_SPI_MAX_REQUEST_LEN + APP_SPI_HANDSHAKE_BYTES)
#define APP_SPI_RX_QUEUE_DEPTH     4u
#define APP_SPI_STATUS_READY       0x00u
#define APP_SPI_STATUS_BUSY        0xFFu
#define APP_SPI_IDLE_FILL          0x00u

#if (APP_SPI_HANDSHAKE_BITS % 8u) != 0
#error "APP_SPI_HANDSHAKE_BITS must be a multiple of 8"
#endif
#if APP_SPI_HANDSHAKE_BYTES == 0
#error "Handshake area must be at least one byte"
#endif

typedef struct {
    uint8_t data[APP_SPI_MAX_REQUEST_LEN];
    uint16_t len;
} app_spi_frame_t;

static router_t g_router;
static router_handlers_t g_handlers;
static response_fifo_t *g_resp_fifo;

static uint8_t g_spi_rx_dma_buf[APP_SPI_DMA_BUF_LEN];
static uint8_t g_spi_tx_dma_buf[APP_SPI_DMA_BUF_LEN];
static volatile uint8_t g_spi_need_restart = 0;

static app_spi_frame_t g_spi_rx_queue[APP_SPI_RX_QUEUE_DEPTH];
static volatile uint8_t g_spi_rx_queue_head = 0;
static volatile uint8_t g_spi_rx_queue_tail = 0;
static volatile uint8_t g_spi_rx_queue_count = 0;
static volatile uint8_t g_spi_rx_overflow = 0;

static volatile int g_spi_tx_busy = 0;

LOG_SVC_DEFINE(LOG_SVC_APP, "app");

static void app_spi_queue_reset(void);
static void app_spi_prime_tx_buffer(uint8_t status);
static uint8_t app_spi_compute_status(void);
static void app_spi_restart_dma(uint8_t status);
static void app_spi_try_restart_dma(void);
static int app_spi_locate_frame(const uint8_t *buf, uint16_t *offset, uint16_t *len);
static int app_spi_queue_push_isr(const uint8_t *frame, uint16_t len);
static int app_spi_queue_pop(app_spi_frame_t *out);
static void app_spi_handle_txrx_complete(void);

void app_init(void) {
    led_service_init();
    log_service_init();
    test_spi_service_init();
    LOGT_THIS(LOG_STATE_START, PROTO_OK, "start", "ready");

    g_resp_fifo = resp_fifo_create();
    memset(&g_handlers, 0, sizeof g_handlers);
    services_register_handlers(&g_handlers);
    router_init(&g_router, g_resp_fifo, &g_handlers);

    app_spi_queue_reset();
    app_spi_prime_tx_buffer(APP_SPI_STATUS_READY);
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, g_spi_tx_dma_buf, g_spi_rx_dma_buf,
                                    (uint16_t)APP_SPI_DMA_BUF_LEN) != HAL_OK) {
        g_spi_need_restart = 1u;
    }

    (void)test_spi_send_hello();
}

void app_poll(void) {
    app_spi_try_restart_dma();

    app_spi_frame_t frame;
    while (app_spi_queue_pop(&frame) == 0) {
        router_feed_bytes(&g_router, frame.data, frame.len);
    }

    app_spi_try_restart_dma();

    if (!g_spi_tx_busy && g_resp_fifo) {
        uint8_t out[64];
        int n = resp_fifo_pop(g_resp_fifo, out, sizeof out);
        if (n > 0) {
            if (HAL_SPI_Transmit_IT(&hspi1, out, (uint16_t)n) == HAL_OK) {
                g_spi_tx_busy = 1;
            }
        }
    }

    if (g_spi_rx_overflow) {
        g_spi_rx_overflow = 0u;
        LOGT_THIS(LOG_STATE_ERROR, PROTO_WARN, "spi_rx", "overflow");
    }
}

void app_on_spi_txrx_half_complete(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        /* Nenhuma ação: handshake já foi enviado no primeiro byte */
    }
}

void app_on_spi_txrx_complete(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        app_spi_handle_txrx_complete();
    }
}

void app_on_spi_tx_complete(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        g_spi_tx_busy = 0;
    }
}

int app_resp_push(const uint8_t *frame, uint32_t len) {
    if (!g_resp_fifo || !frame || len == 0) {
        return -1;
    }
    return resp_fifo_push(g_resp_fifo, frame, len);
}

static void app_spi_queue_reset(void) {
    __disable_irq();
    g_spi_rx_queue_head = 0u;
    g_spi_rx_queue_tail = 0u;
    g_spi_rx_queue_count = 0u;
    g_spi_rx_overflow = 0u;
    __enable_irq();
}

static void app_spi_prime_tx_buffer(uint8_t status) {
    g_spi_tx_dma_buf[0] = status;
    if (APP_SPI_DMA_BUF_LEN > 1u) {
        memset(&g_spi_tx_dma_buf[1], APP_SPI_IDLE_FILL, APP_SPI_DMA_BUF_LEN - 1u);
    }
}

static uint8_t app_spi_compute_status(void) {
    return (g_spi_rx_queue_count >= APP_SPI_RX_QUEUE_DEPTH)
               ? APP_SPI_STATUS_BUSY
               : APP_SPI_STATUS_READY;
}

static void app_spi_restart_dma(uint8_t status) {
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        g_spi_need_restart = 1u;
        return;
    }

    app_spi_prime_tx_buffer(status);
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, g_spi_tx_dma_buf, g_spi_rx_dma_buf,
                                    (uint16_t)APP_SPI_DMA_BUF_LEN) == HAL_OK) {
        g_spi_need_restart = 0u;
    } else {
        g_spi_need_restart = 1u;
    }
}

static void app_spi_try_restart_dma(void) {
    if (!g_spi_need_restart) {
        return;
    }
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        return;
    }
    app_spi_restart_dma(app_spi_compute_status());
}

static int app_spi_locate_frame(const uint8_t *buf, uint16_t *offset, uint16_t *len) {
    if (!buf || !offset || !len || APP_SPI_DMA_BUF_LEN < 2u) {
        return -1;
    }

    uint16_t start = 1u;
    while (start < APP_SPI_DMA_BUF_LEN && buf[start] == APP_SPI_IDLE_FILL) {
        ++start;
    }

    if (start >= APP_SPI_DMA_BUF_LEN || buf[start] != REQ_HEADER) {
        return -1;
    }

    for (uint16_t i = (uint16_t)(start + 1u); i < APP_SPI_DMA_BUF_LEN; ++i) {
        if (buf[i] == REQ_TAIL) {
            uint16_t frame_len = (uint16_t)(i - start + 1u);
            if (frame_len > APP_SPI_MAX_REQUEST_LEN) {
                return -1;
            }
            *offset = start;
            *len = frame_len;
            return 0;
        }
    }

    return -1;
}

static int app_spi_queue_push_isr(const uint8_t *frame, uint16_t len) {
    if (!frame || len == 0u || len > APP_SPI_MAX_REQUEST_LEN) {
        return -1;
    }
    if (g_spi_rx_queue_count >= APP_SPI_RX_QUEUE_DEPTH) {
        return -1;
    }
    app_spi_frame_t *slot = &g_spi_rx_queue[g_spi_rx_queue_head];
    memcpy(slot->data, frame, len);
    slot->len = len;
    g_spi_rx_queue_head = (uint8_t)((g_spi_rx_queue_head + 1u) % APP_SPI_RX_QUEUE_DEPTH);
    g_spi_rx_queue_count++;
    return 0;
}

static int app_spi_queue_pop(app_spi_frame_t *out) {
    int rc = -1;
    __disable_irq();
    if (g_spi_rx_queue_count > 0u) {
        if (out) {
            *out = g_spi_rx_queue[g_spi_rx_queue_tail];
        }
        g_spi_rx_queue_tail = (uint8_t)((g_spi_rx_queue_tail + 1u) % APP_SPI_RX_QUEUE_DEPTH);
        g_spi_rx_queue_count--;
        rc = 0;
    }
    __enable_irq();
    return rc;
}

static void app_spi_handle_txrx_complete(void) {
    uint16_t offset = 0u;
    uint16_t len = 0u;

    if (app_spi_locate_frame(g_spi_rx_dma_buf, &offset, &len) == 0) {
        if (app_spi_queue_push_isr(&g_spi_rx_dma_buf[offset], len) != 0) {
            g_spi_rx_overflow = 1u;
        }
    } else {
        g_spi_rx_overflow = 1u;
    }

    app_spi_restart_dma(app_spi_compute_status());
}
