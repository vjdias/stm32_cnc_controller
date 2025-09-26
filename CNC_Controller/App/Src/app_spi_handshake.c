#include "app_spi_handshake.h"

#include <string.h>

uint8_t app_spi_handshake_compute_status(uint8_t queue_count,
                                         uint8_t queue_capacity) {
    return (queue_count >= queue_capacity) ? APP_SPI_STATUS_BUSY
                                           : APP_SPI_STATUS_READY;
}

/*
 * A rotina abaixo encapsula a preparação do buffer DMA utilizado no
 * handshaking SPI. Sem payload pendente ela preenche todos os 42 bytes com o
 * status de READY/BUSY, preservando o comportamento clássico do protocolo.
 * Quando uma resposta é disponibilizada pelos serviços, o conteúdo é copiado
 * a partir da posição zero do quadro, mantendo o header 0xAB no primeiro byte
 * observado pelo mestre. O restante do buffer permanece preenchido com o
 * padrão de status informado (tipicamente READY uma vez que existe payload).
 * Dessa forma evitamos que 0x5A (BUSY) anteceda o header durante o polling,
 * ao mesmo tempo em que preservamos o comportamento histórico de preencher o
 * espaço não utilizado com o handshake vigente.
 */
app_spi_handshake_prime_result_t
app_spi_handshake_prime(const app_spi_handshake_prime_args_t *args) {
    app_spi_handshake_prime_result_t result = {
        .state = APP_SPI_HANDSHAKE_STATE_UNRECOGNIZED,
        .consumed_response = 0u,
    };

    if (!args || !args->tx_buf || args->tx_len == 0u) {
        return result;
    }

    memset(args->tx_buf, args->status_byte, args->tx_len);

    if (args->status_byte == APP_SPI_STATUS_READY) {
        result.state = APP_SPI_HANDSHAKE_STATE_READY;
    } else if (args->status_byte == APP_SPI_STATUS_BUSY) {
        result.state = APP_SPI_HANDSHAKE_STATE_BUSY;
    } else {
        result.state = APP_SPI_HANDSHAKE_STATE_UNRECOGNIZED;
    }

    if (args->response_buf && args->response_len > 0u) {
        if (args->response_len <= args->tx_len &&
            (result.state == APP_SPI_HANDSHAKE_STATE_READY ||
             result.state == APP_SPI_HANDSHAKE_STATE_BUSY)) {
            memcpy(args->tx_buf, args->response_buf, args->response_len);
            result.consumed_response = 1u;
            result.state = APP_SPI_HANDSHAKE_STATE_RESPONSE;
        } else {
            result.state = APP_SPI_HANDSHAKE_STATE_UNRECOGNIZED;
            result.consumed_response = 0u;
        }
    }

    return result;
}
