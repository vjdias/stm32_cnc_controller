#include <string.h>
#include "spi.h"
#include "app.h"
#include "Protocol/router.h"
#include "Services/service_adapters.h"
#include "Services/Led/led_service.h"
#include "Services/Log/log_service.h"

// Router + response queue
static router_t g_router;
static router_handlers_t g_handlers;
static response_fifo_t *g_resp_fifo;

// SPI RX/TX buffers and state
#ifndef APP_SPI_RX_BUF_SZ
#define APP_SPI_RX_BUF_SZ 256u
#endif
static uint8_t g_spi_rx_buf[APP_SPI_RX_BUF_SZ];
static volatile int g_spi_tx_busy = 0;

LOG_SVC_DEFINE(LOG_SVC_APP, "app");

void app_init(void) {
    // Init services (GPIO for LED etc.)
    led_service_init();
    log_service_init();
    // Boot log (visible on USART1 VCP terminal)
    LOGT_THIS(LOG_STATE_START, PROTO_OK, "start", "ready");

    // Prepare router and response FIFO
    g_resp_fifo = resp_fifo_create();
    memset(&g_handlers, 0, sizeof g_handlers);
    services_register_handlers(&g_handlers);
    router_init(&g_router, g_resp_fifo, &g_handlers);

    // Start SPI RX DMA in circular mode to feed router from callbacks
    (void)HAL_SPI_Receive_DMA(&hspi1, g_spi_rx_buf, (uint16_t)APP_SPI_RX_BUF_SZ);
}

void app_poll(void) {
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
    log_poll();
}

// HAL callbacks (override weak definitions) to feed the router
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        router_feed_bytes(&g_router, g_spi_rx_buf, APP_SPI_RX_BUF_SZ / 2);
    }
}
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        router_feed_bytes(&g_router, g_spi_rx_buf + (APP_SPI_RX_BUF_SZ / 2), APP_SPI_RX_BUF_SZ / 2);
    }
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        g_spi_tx_busy = 0;
    }
}
