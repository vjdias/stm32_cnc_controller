#pragma once
#include <stdint.h>

// Inicializa o serviço de teste (placeholder)
void test_spi_service_init(void);

// Empilha no FIFO um frame de resposta mínimo: AB 'hello' 54
int test_spi_send_hello(void);

