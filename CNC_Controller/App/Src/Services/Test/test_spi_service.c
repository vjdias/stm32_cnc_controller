#include "Services/Test/test_spi_service.h"
#include "Protocol/frame_defs.h"
#include "app.h"

void test_spi_service_init(void) {
    // Nada a inicializar por ora
}

int test_spi_send_hello(void) {
    uint8_t frame[7];
    frame[0] = RESP_HEADER; // 0xAB
    frame[1] = 'h';
    frame[2] = 'e';
    frame[3] = 'l';
    frame[4] = 'l';
    frame[5] = 'o';
    frame[6] = RESP_TAIL;   // 0x54
    return app_resp_push(frame, sizeof frame);
}

