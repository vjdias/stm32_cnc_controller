// Inicialização da aplicação: integra SPI DMA, roteador de protocolo e serviços
#pragma once

// Declaração antecipada para evitar incluir os headers do HAL aqui
typedef struct __SPI_HandleTypeDef SPI_HandleTypeDef;

/**
 * @brief Inicializa os subsistemas da aplicação (roteador, serviços e SPI DMA).
 */
void app_init(void);

/**
 * @brief Processa rotinas periódicas, transmitindo respostas pendentes via SPI.
 */
void app_poll(void);

/**
 * @brief Permite que os serviços empilhem um frame de resposta bruto (AB..54).
 *
 * @param frame Ponteiro para o frame completo.
 * @param len   Tamanho do frame em bytes.
 * @return 0 em caso de sucesso, código de erro negativo em falha.
 */
int app_resp_push(const uint8_t *frame, uint32_t len);

/**
 * @brief Hook para o callback de meia transferência do SPI DMA.
 */
void app_on_spi_txrx_half_complete(SPI_HandleTypeDef *h);
/**
 * @brief Hook para o callback de transferência completa do SPI DMA.
 */
void app_on_spi_txrx_complete(SPI_HandleTypeDef *h);
/**
 * @brief Hook para o callback de transmissão concluída do SPI (modo IT).
 */
void app_on_spi_tx_complete(SPI_HandleTypeDef *h);
