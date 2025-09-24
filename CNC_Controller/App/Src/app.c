#include <string.h>
#include <stdint.h>
#include "spi.h"
#include "app.h"
#include "Protocol/frame_defs.h"
#include "Protocol/router.h"
#include "Services/service_adapters.h"
#include "Services/Led/led_service.h"
#include "Services/Log/log_service.h"
#include "Services/Test/test_spi_service.h"

#define APP_SPI_MAX_REQUEST_LEN    42u
#define APP_SPI_DMA_BUF_LEN        APP_SPI_MAX_REQUEST_LEN
#define APP_SPI_RX_QUEUE_DEPTH     APP_SPI_DMA_BUF_LEN
#define APP_SPI_STATUS_READY       0xA5u
#define APP_SPI_STATUS_BUSY        0x5Au
/* Estados de handshake usam padrões alternados para evitar colisão com 0x00/0xFF. */

#if APP_SPI_DMA_BUF_LEN != 42u
#error "SPI DMA buffer must remain 42 bytes (payload + handshake por byte)"
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
static volatile uint8_t g_spi_next_status = APP_SPI_STATUS_READY;

#if defined(SCB_CleanDCache_by_Addr) || defined(SCB_InvalidateDCache_by_Addr)
#define APP_SPI_DCACHE_LINE_SIZE    32u
static void app_spi_clean_dcache(void *addr, uint32_t len);
static void app_spi_invalidate_dcache(void *addr, uint32_t len);
#endif

static app_spi_frame_t g_spi_rx_queue[APP_SPI_RX_QUEUE_DEPTH];
static volatile uint8_t g_spi_rx_queue_head = 0;
static volatile uint8_t g_spi_rx_queue_tail = 0;
static volatile uint8_t g_spi_rx_queue_count = 0;
static volatile uint8_t g_spi_rx_overflow = 0;

static uint8_t g_spi_tx_pending_buf[APP_SPI_DMA_BUF_LEN];
static volatile uint16_t g_spi_tx_pending_len = 0u;
static volatile uint8_t g_spi_tx_pending_ready = 0u;

LOG_SVC_DEFINE(LOG_SVC_APP, "app");

static void app_spi_queue_reset(void);
static uint8_t app_spi_prime_tx_buffer(uint8_t status);
static uint8_t app_spi_compute_status(void);
static void app_spi_restart_dma(uint8_t status);
static void app_spi_try_restart_dma(void);
static int app_spi_locate_frame(const uint8_t *buf, uint16_t *offset, uint16_t *len);
static int app_spi_queue_push_isr(const uint8_t *frame, uint16_t len);
static int app_spi_queue_pop(app_spi_frame_t *out);
static void app_spi_handle_txrx_complete(void);

#if defined(SCB_CleanDCache_by_Addr) || defined(SCB_InvalidateDCache_by_Addr)
static uintptr_t app_spi_cache_align_down(uintptr_t addr);
static uintptr_t app_spi_cache_align_up(uintptr_t addr);
#endif

#if defined(SCB_CleanDCache_by_Addr)
static void app_spi_clean_dcache(void *addr, uint32_t len) {
    if (((SCB->CCR & SCB_CCR_DC_Msk) == 0u) || len == 0u) {
        return;
    }

    uintptr_t start = app_spi_cache_align_down((uintptr_t)addr);
    uintptr_t end = app_spi_cache_align_up((uintptr_t)addr + (uintptr_t)len);
    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}
#else
static void app_spi_clean_dcache(void *addr, uint32_t len) {
    (void)addr;
    (void)len;
}
#endif

#if defined(SCB_InvalidateDCache_by_Addr)
static void app_spi_invalidate_dcache(void *addr, uint32_t len) {
    if (((SCB->CCR & SCB_CCR_DC_Msk) == 0u) || len == 0u) {
        return;
    }

    uintptr_t start = app_spi_cache_align_down((uintptr_t)addr);
    uintptr_t end = app_spi_cache_align_up((uintptr_t)addr + (uintptr_t)len);
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}
#else
static void app_spi_invalidate_dcache(void *addr, uint32_t len) {
    (void)addr;
    (void)len;
}
#endif

#if defined(SCB_CleanDCache_by_Addr) || defined(SCB_InvalidateDCache_by_Addr)
static uintptr_t app_spi_cache_align_down(uintptr_t addr) {
    return addr & ~((uintptr_t)APP_SPI_DCACHE_LINE_SIZE - 1u);
}

static uintptr_t app_spi_cache_align_up(uintptr_t addr) {
    return (addr + ((uintptr_t)APP_SPI_DCACHE_LINE_SIZE - 1u)) &
           ~((uintptr_t)APP_SPI_DCACHE_LINE_SIZE - 1u);
}
#endif

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
    g_spi_next_status = APP_SPI_STATUS_READY;
    (void)app_spi_prime_tx_buffer(g_spi_next_status);
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

    if (g_resp_fifo && !g_spi_tx_pending_ready) {
        uint8_t out[APP_SPI_DMA_BUF_LEN];
        int n = resp_fifo_pop(g_resp_fifo, out, sizeof out);
        if (n > 0 && n <= (int)APP_SPI_DMA_BUF_LEN) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            memcpy(g_spi_tx_pending_buf, out, (uint32_t)n);
            g_spi_tx_pending_len = (uint16_t)n;
            g_spi_tx_pending_ready = 1u;
            if (primask == 0u) {
                __enable_irq();
            }
        } else if (n == PROTO_ERR_RANGE) {
            LOGT_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "spi_tx", "resp too large for dma frame");
        }
    }

    if (g_spi_rx_overflow) {
        g_spi_rx_overflow = 0u;
        LOGT_THIS(LOG_STATE_ERROR, PROTO_WARN, "spi_rx", "overflow");
    }
}

void app_on_spi_txrx_half_complete(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        /* Reserva o handshake para sinalizar BUSY até concluir o tratamento atual */
        g_spi_next_status = APP_SPI_STATUS_BUSY;
    }
}

void app_on_spi_txrx_complete(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        app_spi_handle_txrx_complete();
    }
}

void app_on_spi_tx_complete(SPI_HandleTypeDef *h) {
    (void)h;
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

static uint8_t app_spi_prime_tx_buffer(uint8_t status) {
    uint8_t used_pending = 0u;

    memset(g_spi_tx_dma_buf, status, APP_SPI_DMA_BUF_LEN);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t has_pending = g_spi_tx_pending_ready;
    uint16_t pending_len = g_spi_tx_pending_len;
    if (has_pending && pending_len > 0u && pending_len <= APP_SPI_DMA_BUF_LEN) {
        memcpy(g_spi_tx_dma_buf, g_spi_tx_pending_buf, pending_len);
        used_pending = 1u;
    }
    if (primask == 0u) {
        __enable_irq();
    }

    app_spi_clean_dcache(g_spi_tx_dma_buf, APP_SPI_DMA_BUF_LEN);
    return used_pending;
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

    g_spi_next_status = status;
    uint8_t used_pending = app_spi_prime_tx_buffer(status);
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, g_spi_tx_dma_buf, g_spi_rx_dma_buf,
                                    (uint16_t)APP_SPI_DMA_BUF_LEN) == HAL_OK) {
        g_spi_need_restart = 0u;
        if (used_pending) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            g_spi_tx_pending_ready = 0u;
            g_spi_tx_pending_len = 0u;
            if (primask == 0u) {
                __enable_irq();
            }
        }
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

    app_spi_restart_dma(g_spi_next_status);
}

static int app_spi_locate_frame(const uint8_t *buf, uint16_t *offset, uint16_t *len) {
    if (!buf || !offset || !len || APP_SPI_DMA_BUF_LEN < 2u) {
        return -1;
    }

    uint16_t start = 0u;
    while (start < APP_SPI_DMA_BUF_LEN && buf[start] != REQ_HEADER) {
        ++start;
    }

    if (start >= APP_SPI_DMA_BUF_LEN) {
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
    uint8_t armazenado = 0u;

    app_spi_invalidate_dcache(g_spi_rx_dma_buf, APP_SPI_DMA_BUF_LEN);

    if (app_spi_locate_frame(g_spi_rx_dma_buf, &offset, &len) == 0) {
        if (app_spi_queue_push_isr(&g_spi_rx_dma_buf[offset], len) == 0) {
            armazenado = 1u;
        } else {
            g_spi_rx_overflow = 1u;
        }
    } else {
        g_spi_rx_overflow = 1u;
    }

    if (armazenado) {
        g_spi_next_status = app_spi_compute_status();
    } else {
        g_spi_next_status = APP_SPI_STATUS_BUSY;
    }

    app_spi_restart_dma(g_spi_next_status);
}
