#ifndef APP_SPI_HANDSHAKE_H
#define APP_SPI_HANDSHAKE_H

#include <stdint.h>

#define APP_SPI_MAX_REQUEST_LEN 42u
#define APP_SPI_DMA_BUF_LEN   APP_SPI_MAX_REQUEST_LEN

#define APP_SPI_STATUS_READY 0xA5u
#define APP_SPI_STATUS_BUSY  0x5Au
/*
 * Byte enviado pelo Raspberry Pi enquanto aguarda uma resposta do STM32.
 * Diferencia o polling do handshaking READY/BUSY sem conflitar com header/tail.
 */
#define APP_SPI_CLIENT_POLL_BYTE 0x3Cu

typedef enum {
    APP_SPI_HANDSHAKE_STATE_READY = 0,
    APP_SPI_HANDSHAKE_STATE_BUSY,
    APP_SPI_HANDSHAKE_STATE_RESPONSE,
    APP_SPI_HANDSHAKE_STATE_UNRECOGNIZED,
} app_spi_handshake_state_t;

typedef struct {
    uint8_t status_byte;
    uint8_t *tx_buf;
    uint16_t tx_len;
    const uint8_t *response_buf;
    uint16_t response_len;
} app_spi_handshake_prime_args_t;

typedef struct {
    app_spi_handshake_state_t state;
    uint8_t consumed_response;
} app_spi_handshake_prime_result_t;

uint8_t app_spi_handshake_compute_status(uint8_t queue_count,
                                         uint8_t queue_capacity);

app_spi_handshake_prime_result_t
app_spi_handshake_prime(const app_spi_handshake_prime_args_t *args);

#endif /* APP_SPI_HANDSHAKE_H */
