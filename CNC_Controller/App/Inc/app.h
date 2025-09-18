// App bootstrap: wires SPI DMA callbacks to protocol router and services
#pragma once

// Forward declaration to avoid including HAL headers here
typedef struct __SPI_HandleTypeDef SPI_HandleTypeDef;

// Initialize app subsystems (router, services, SPI DMA)
void app_init(void);

// Poll for pending responses to transmit over SPI
void app_poll(void);

// Allow services to push a raw response frame (AB..54) into TX FIFO
int app_resp_push(const uint8_t *frame, uint32_t len);

// Hooks invoked from HAL callbacks (defined in main.c)
void app_on_spi_rx_half_complete(SPI_HandleTypeDef *h);
void app_on_spi_rx_complete(SPI_HandleTypeDef *h);
void app_on_spi_tx_complete(SPI_HandleTypeDef *h);
void app_on_spi_error(SPI_HandleTypeDef *h);
