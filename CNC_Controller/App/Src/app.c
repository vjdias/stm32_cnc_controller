#include <string.h>
#include "spi.h"
#include "app.h"
#include "Protocol/router.h"
#include "Services/service_adapters.h"
#include "Services/Led/led_service.h"
#include "Services/Log/log_service.h"
#include "Services/Test/test_spi_service.h"

// Router + response queue
static router_t g_router;
static router_handlers_t g_handlers;
static response_fifo_t *g_resp_fifo;

// SPI RX/TX buffers and state
#ifndef APP_SPI_RX_BUF_SZ
#define APP_SPI_RX_BUF_SZ 256u
#endif
static uint8_t g_spi_rx_buf[APP_SPI_RX_BUF_SZ];
static volatile uint16_t g_spi_rx_tail = 0;
static volatile uint32_t g_spi_error_flags = 0;
static volatile uint8_t g_spi_restart_pending = 0;
static volatile int g_spi_tx_busy = 0;

LOG_SVC_DEFINE(LOG_SVC_APP, "app");

static void app_spi_drain_rx(void);
static void app_spi_handle_pending_error(void);
static void app_spi_restart_rx(SPI_HandleTypeDef *h);

void app_init(void) {
    // Init services (GPIO for LED etc.)
    led_service_init();
    log_service_init();
    test_spi_service_init();
    // Boot log (visible on USART1 VCP terminal)
    LOGT_THIS(LOG_STATE_START, PROTO_OK, "start", "ready");

    // Prepare router and response FIFO
    g_resp_fifo = resp_fifo_create();
    memset(&g_handlers, 0, sizeof g_handlers);
    services_register_handlers(&g_handlers);
    router_init(&g_router, g_resp_fifo, &g_handlers);

    // Start SPI RX DMA in circular mode to feed router from callbacks
    (void)HAL_SPI_Receive_DMA(&hspi1, g_spi_rx_buf, (uint16_t)APP_SPI_RX_BUF_SZ);

    // Enfileira um frame de teste: AB "hello" 54
    (void)test_spi_send_hello();
}

void app_poll(void) {

    // Handle any latched SPI fault before looking at new data
    app_spi_handle_pending_error();

    // Drain any bytes that arrived since the last iteration
    app_spi_drain_rx();

    // If TX is idle, try to pop one response frame from FIFO and transmit
    if (!g_spi_tx_busy && g_resp_fifo) {
        uint8_t out[64];
        int n = resp_fifo_pop(g_resp_fifo, out, sizeof out);
        if (n > 0) {
            // Use interrupt-driven TX to avoid DMA mode constraints
            if (HAL_SPI_Transmit_IT(&hspi1, out, (uint16_t)n) == HAL_OK) {
                g_spi_tx_busy = 1;
            }
        }
    }

    // Lowest priority: drain log output (non-blocking, only if USART idle)
    //log_poll();
}

// Funções auxiliares invocadas por main.c a partir dos callbacks do HAL
void app_on_spi_rx_half_complete(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        app_spi_drain_rx();
    }
}

void app_on_spi_rx_complete(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        app_spi_drain_rx();
    }
}

void app_on_spi_tx_complete(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        g_spi_tx_busy = 0;
    }
}

void app_on_spi_error(SPI_HandleTypeDef *h) {
    if (!h || h->Instance != SPI1) {
        return;
    }

    uint32_t err = HAL_SPI_GetError(h);
    if (err == HAL_SPI_ERROR_NONE) {
        return;
    }

    g_spi_tx_busy = 0;
    g_spi_error_flags |= err;
    g_spi_restart_pending = 1u;
}

int app_resp_push(const uint8_t *frame, uint32_t len) {
    if (!g_resp_fifo || !frame || len == 0) return -1;
    return resp_fifo_push(g_resp_fifo, frame, len);
}

static void app_spi_drain_rx(void) {
    if (!hspi1.hdmarx) {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint16_t cur = (uint16_t)(APP_SPI_RX_BUF_SZ - __HAL_DMA_GET_COUNTER(hspi1.hdmarx));
    uint16_t tail = g_spi_rx_tail;

    if (cur == tail) {
        __set_PRIMASK(primask);
        return; // nothing new
    }

    g_spi_rx_tail = cur;

    __set_PRIMASK(primask);

    if (cur > tail) {
        router_feed_bytes(&g_router, g_spi_rx_buf + tail, cur - tail);
    } else {
        if (tail < APP_SPI_RX_BUF_SZ) {
            router_feed_bytes(&g_router, g_spi_rx_buf + tail, APP_SPI_RX_BUF_SZ - tail);
        }
        if (cur > 0) {
            router_feed_bytes(&g_router, g_spi_rx_buf, cur);
        }
    }
}

static void app_spi_handle_pending_error(void) {
    if (!g_spi_restart_pending) {
        return;
    }

    uint32_t flags;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    flags = g_spi_error_flags;
    g_spi_error_flags = 0u;
    g_spi_restart_pending = 0u;
    __set_PRIMASK(primask);

    if (flags == HAL_SPI_ERROR_NONE) {
        return;
    }

    LOGA_THIS(LOG_STATE_WARNING, (int)flags, "spi", "fault flags=0x%02lX state=%lu",
              (unsigned long)flags, (unsigned long)HAL_SPI_GetState(&hspi1));

    if ((flags & HAL_SPI_ERROR_OVR) != 0u) {
        __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    }
    if ((flags & HAL_SPI_ERROR_MODF) != 0u) {
        __HAL_SPI_CLEAR_MODFFLAG(&hspi1);
    }
    if ((flags & HAL_SPI_ERROR_FRE) != 0u) {
        __HAL_SPI_CLEAR_FREFLAG(&hspi1);
    }

    app_spi_restart_rx(&hspi1);
}

static void app_spi_restart_rx(SPI_HandleTypeDef *h) {
    if (!h || !h->hdmarx) {
        return;
    }

    if (HAL_SPI_DMAStop(h) != HAL_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "spi", "failed to stop dma for restart");
        g_spi_restart_pending = 1u;
    }

    g_spi_rx_tail = 0u;

    if (HAL_SPI_Receive_DMA(h, g_spi_rx_buf, (uint16_t)APP_SPI_RX_BUF_SZ) != HAL_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "spi", "failed to rearm rx dma");
        g_spi_restart_pending = 1u;
    }
}
