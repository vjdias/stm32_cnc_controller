// App bootstrap: wires SPI DMA callbacks to protocol router and services
#pragma once

// Initialize app subsystems (router, services, SPI DMA)
void app_init(void);

// Poll for pending responses to transmit over SPI
void app_poll(void);

