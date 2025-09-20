#include <string.h>
#include "Services/Test/test_spi_service.h"
#include "Protocol/frame_defs.h"
#include "app.h"

#define TEST_SPI_HELLO_SUFFIX_LEN 4u
#define TEST_SPI_HELLO_MIN_REQ_LEN 7u

static const uint8_t k_test_spi_hello_suffix[TEST_SPI_HELLO_SUFFIX_LEN] = {
    'e', 'l', 'l', 'o'
};

void test_spi_service_init(void) {
    // Nada a inicializar por ora
}

int test_spi_send_hello(void) {
    uint8_t frame[TEST_SPI_HELLO_MIN_REQ_LEN];
    frame[0] = RESP_HEADER;          // 0xAB
    frame[1] = RESP_TEST_HELLO;      // 'h'
    memcpy(&frame[2], k_test_spi_hello_suffix, TEST_SPI_HELLO_SUFFIX_LEN);
    frame[TEST_SPI_HELLO_MIN_REQ_LEN - 1u] = RESP_TAIL;   // 0x54
    return app_resp_push(frame, (uint32_t)sizeof frame);
}

void test_spi_on_hello(const uint8_t *frame, uint32_t len) {
    if (!frame) {
        return;
    }

    if (frame_expect_req(frame, len, REQ_TEST_HELLO, TEST_SPI_HELLO_MIN_REQ_LEN)
        != PROTO_OK) {
        return;
    }

    if (memcmp(&frame[2], k_test_spi_hello_suffix, TEST_SPI_HELLO_SUFFIX_LEN)
        != 0) {
        return;
    }

    (void)test_spi_send_hello();
}

