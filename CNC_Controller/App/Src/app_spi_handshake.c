#include "app_spi_handshake.h"

#include <string.h>

uint8_t app_spi_handshake_compute_status(uint8_t queue_count,
                                         uint8_t queue_capacity) {
    return (queue_count >= queue_capacity) ? APP_SPI_STATUS_BUSY
                                           : APP_SPI_STATUS_READY;
}

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
