// App bootstrap: wires SPI DMA callbacks to protocol router and services
#pragma once

// Initialize app subsystems (router, services, SPI DMA)
void app_init(void);

// Poll for pending responses to transmit over SPI
void app_poll(void);

// Allow services to push a raw response frame (AB..54) into TX FIFO
int app_resp_push(const uint8_t *frame, uint32_t len);
