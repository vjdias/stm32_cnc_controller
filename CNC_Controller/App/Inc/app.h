// Inicialização da aplicação: integra SPI DMA, roteador de protocolo e serviços

/**
 * @brief Visão geral do ciclo SPI → fila → serviços.
 *
 * 1) Fluxo do SPI com DMA — `app_init` arma o `HAL_SPI_TransmitReceive_DMA`
 *    com um buffer inteiro preenchido pelo handshake atual (0xA5 pronto ou
 *    0x5A ocupado). Os hooks `app_on_spi_txrx_half_complete` e
 *    `app_on_spi_txrx_complete` mantêm esse preenchimento uniforme em cada
 *    transação, garantindo que o mestre sempre receba um estado consistente
 *    enquanto o DMA coleta o frame AA..55.
 *
 * 2) Durante a cópia para a fila interna — `app_on_spi_txrx_complete` roda
 *    depois que o HAL derruba `RXDMAEN/TXDMAEN`. Somente nesse ponto os bytes
 *    recém-recebidos são invalidados no cache e copiados para a fila circular,
 *    protegendo o frame porque o DMA já não pode mais sobrescrever o buffer. Se
 *    o mestre insistir em clockar outro pedido nesse intervalo, não há DMA
 *    ligado para recebê-lo; esses bytes ficam do lado do mestre e serão
 *    descartados na aplicação Raspberry assim que o handshake indicar BUSY.
 *
 * 3) Preparação da próxima recepção — assim que a cópia termina,
 *    `app_on_spi_txrx_complete` decide o próximo handshake (READY ou BUSY) e
 *    `app_on_spi_txrx_half_complete` garante que qualquer novo ciclo iniciado
 *    antes da conclusão completa do processamento volte a sair com 0x5A. Isso
 *    impede o STM32 de reler o mesmo buffer enquanto ainda há dados a mover e
 *    sinaliza explicitamente ao Raspberry Pi que qualquer frame enviado nessa
 *    janela será ignorado e deverá ser reenviado mais tarde.
 */
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
 * @brief Hook legado do callback de transmissão concluída do SPI (modo IT).
 *
 * Mantido apenas por compatibilidade com o código gerado pelo CubeIDE; a
 * transmissão das respostas utiliza o próprio buffer do DMA e não depende mais
 * desse callback.
 */
void app_on_spi_tx_complete(SPI_HandleTypeDef *h);
