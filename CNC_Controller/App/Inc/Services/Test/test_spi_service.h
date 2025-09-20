#pragma once
#include <stdint.h>

// Inicializa o serviço de teste (placeholder)
void test_spi_service_init(void);

// Empilha no FIFO um frame de resposta mínimo: AB 'hello' 54
int test_spi_send_hello(void);

// Trata requisições 'hello' vindas por SPI (AA 'hello' 55)
void test_spi_on_hello(const uint8_t *frame, uint32_t len);

