// App bootstrap: integra SPI + serviços de aplicação
#pragma once

// Inicializa serviços, roteador e ativa DMA circular do SPI
void app_init(void);

// Executa tarefas em laço (envio de respostas pendentes)
void app_poll(void);

// Permite que serviços enfileirem um frame bruto (AB..54) para TX
int app_resp_push(const uint8_t *frame, uint32_t len);

// Callbacks internos acionados pelo HAL_SPI_* ao concluir cada etapa do DMA
void app_on_spi_rx_half_complete(void);
void app_on_spi_rx_complete(void);
void app_on_spi_tx_complete(void);
