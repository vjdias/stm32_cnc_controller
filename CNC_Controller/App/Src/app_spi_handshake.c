#include "app_spi_handshake.h"

#include <string.h>

uint8_t app_spi_handshake_compute_status(uint8_t queue_count,
                                         uint8_t queue_capacity) {
    return (queue_count >= queue_capacity) ? APP_SPI_STATUS_BUSY
                                           : APP_SPI_STATUS_READY;
}

/*
 * A rotina abaixo encapsula a preparação do buffer DMA utilizado no
 * handshaking SPI. Quando não há payload pendente ela preenche todos os 42
 * bytes com o status de READY/BUSY, preservando o comportamento clássico do
 * protocolo. Assim que um serviço enfileira uma resposta, o conteúdo é
 * alinhado à direita e preenchido com zeros à esquerda antes do DMA ser
 * iniciado. Esse alinhamento garante que os bytes de "READY" (0xA5) ou
 * "BUSY" (0x5A) não vazem para o início da mensagem — exatamente o efeito
 * observado pelo usuário quando o primeiro byte chegava como 0x5A. O mestre
 * Raspberry Pi envia 0x3C ao pesquisar por respostas pendentes e espera que
 * qualquer payload sobrescreva completamente o preenchimento de status. Com a
 * reformulação, assim que a fila fica vazia voltamos a transmitir apenas o
 * padrão READY, enquanto respostas reais ocupam o final do quadro com zeros à
 * esquerda, como requerido.
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
            uint32_t pad = args->tx_len - args->response_len;
            if (pad > 0u) {
                memset(args->tx_buf, 0, pad);
            }
            memcpy(&args->tx_buf[pad], args->response_buf, args->response_len);
            result.consumed_response = 1u;
            result.state = APP_SPI_HANDSHAKE_STATE_RESPONSE;
        } else {
            result.state = APP_SPI_HANDSHAKE_STATE_UNRECOGNIZED;
            result.consumed_response = 0u;
        }
    }

    return result;
}
