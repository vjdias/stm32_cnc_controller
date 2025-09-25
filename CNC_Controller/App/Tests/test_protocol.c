#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Protocol/frame_defs.h"
#include "Protocol/Requests/move_home_request.h"
#include "Protocol/Requests/move_probe_level_request.h"
#include "Protocol/Requests/move_queue_add_request.h"
#include "Protocol/Responses/move_queue_add_ack_response.h"
#include "Protocol/router.h"
#include "app_spi_handshake.h"

static void test_move_home_req(void){
    move_home_req_t in = { .frameId=0xAA, .axisMask=0x03, .dirMask=0x01, .vhome=0x1234 };
    uint8_t raw[9];
    int rc = move_home_req_encoder(&in, raw, sizeof raw);
    assert(rc == 0);
    assert(raw[0] == REQ_HEADER && raw[1] == REQ_MOVE_HOME && raw[8] == REQ_TAIL);
    assert(move_home_req_check_parity(raw, sizeof raw) == 1);
    move_home_req_t out = {0};
    rc = move_home_req_decoder(raw, sizeof raw, &out);
    assert(rc == 0);
    assert(out.frameId == in.frameId);
    assert(out.axisMask == in.axisMask);
    assert(out.dirMask == in.dirMask);
    assert(out.vhome == in.vhome);
}

static void test_move_probe_level_req(void){
    move_probe_level_req_t in = { .frameId=0x10, .axisMask=0x04, .vprobe=0x0F0F };
    uint8_t raw[8];
    int rc = move_probe_level_req_encoder(&in, raw, sizeof raw);
    assert(rc == 0);
    assert(raw[0] == REQ_HEADER && raw[1] == REQ_MOVE_PROBE_LEVEL && raw[7] == REQ_TAIL);
    assert(move_probe_level_req_check_parity(raw, sizeof raw) == 1);
    move_probe_level_req_t out = {0};
    rc = move_probe_level_req_decoder(raw, sizeof raw, &out);
    assert(rc == 0);
    assert(out.frameId == in.frameId);
    assert(out.axisMask == in.axisMask);
    assert(out.vprobe == in.vprobe);
}

static void test_move_queue_add_req_parity_and_roundtrip(void){
    move_queue_add_req_t in = {0};
    in.frameId=0x55; in.dirMask=0x05;
    in.vx=100; in.sx=10000; in.vy=200; in.sy=20000; in.vz=300; in.sz=30000;
    in.kp_x=1; in.ki_x=2; in.kd_x=3; in.kp_y=4; in.ki_y=5; in.kd_y=6; in.kp_z=7; in.ki_z=8; in.kd_z=9;
    uint8_t raw[42];
    int rc = move_queue_add_req_encoder(&in, raw, sizeof raw);
    assert(rc == 0);
    assert(raw[0] == REQ_HEADER && raw[1] == REQ_MOVE_QUEUE_ADD && raw[41] == REQ_TAIL);
    assert(move_queue_add_req_check_parity(raw, sizeof raw) == 1);
    move_queue_add_req_t out = {0};
    rc = move_queue_add_req_decoder(raw, sizeof raw, &out);
    assert(rc == 0);
    assert(out.frameId==in.frameId && out.dirMask==in.dirMask);
    assert(out.vx==in.vx && out.sx==in.sx);
    assert(out.vy==in.vy && out.sy==in.sy);
    assert(out.vz==in.vz && out.sz==in.sz);
    assert(out.kp_x==in.kp_x && out.ki_x==in.ki_x && out.kd_x==in.kd_x);
    assert(out.kp_y==in.kp_y && out.ki_y==in.ki_y && out.kd_y==in.kd_y);
    assert(out.kp_z==in.kp_z && out.ki_z==in.ki_z && out.kd_z==in.kd_z);
}

static void test_response_fifo_basic(void){
    response_fifo_t* q = resp_fifo_create();
    assert(q != NULL);
    uint8_t frame[6] = { RESP_HEADER, RESP_MOVE_QUEUE_ADD_ACK, 0x22, 0x00, 0x00, RESP_TAIL };
    move_queue_add_ack_resp_t ack = { .frameId=0x22, .status=0x00 };
    frame[4] = move_queue_add_ack_resp_calc_parity(&ack);
    int rc = resp_fifo_push(q, frame, sizeof frame);
    assert(rc == 0);
    assert(resp_fifo_count(q) == 1);

    uint8_t out[6] = {0};
    int n = resp_fifo_pop(q, out, sizeof out);
    assert(n == 6);
    assert(memcmp(out, frame, 6) == 0);
    assert(resp_fifo_count(q) == 0);

    // Caminho de erro de faixa
    rc = resp_fifo_push(q, frame, sizeof frame);
    assert(rc == 0);
    uint8_t small[4];
    n = resp_fifo_pop(q, small, sizeof small);
    assert(n < 0); // PROTO_ERR_RANGE

    // limpeza
    resp_fifo_destroy(q);
}

static void test_handshake_ready_state(void){
    uint8_t tx[APP_SPI_MAX_REQUEST_LEN] = {0};
    app_spi_handshake_prime_args_t args = {
        .status_byte = APP_SPI_STATUS_READY,
        .tx_buf = tx,
        .tx_len = sizeof tx,
        .response_buf = NULL,
        .response_len = 0u,
    };

    app_spi_handshake_prime_result_t res = app_spi_handshake_prime(&args);
    assert(res.state == APP_SPI_HANDSHAKE_STATE_READY);
    assert(res.consumed_response == 0u);
    for (size_t i = 0; i < sizeof tx; ++i) {
        assert(tx[i] == APP_SPI_STATUS_READY);
    }
}

static void test_handshake_busy_state(void){
    uint8_t tx[APP_SPI_MAX_REQUEST_LEN] = {0};
    app_spi_handshake_prime_args_t args = {
        .status_byte = APP_SPI_STATUS_BUSY,
        .tx_buf = tx,
        .tx_len = sizeof tx,
        .response_buf = NULL,
        .response_len = 0u,
    };

    app_spi_handshake_prime_result_t res = app_spi_handshake_prime(&args);
    assert(res.state == APP_SPI_HANDSHAKE_STATE_BUSY);
    assert(res.consumed_response == 0u);
    for (size_t i = 0; i < sizeof tx; ++i) {
        assert(tx[i] == APP_SPI_STATUS_BUSY);
    }
}

static void test_handshake_response_state(void){
    uint8_t tx[APP_SPI_MAX_REQUEST_LEN] = {0};
    const uint8_t resp[] = { RESP_HEADER, RESP_MOVE_QUEUE_ADD_ACK, 0x10, 0x00, 0x00, RESP_TAIL };
    app_spi_handshake_prime_args_t args = {
        .status_byte = APP_SPI_STATUS_READY,
        .tx_buf = tx,
        .tx_len = sizeof tx,
        .response_buf = resp,
        .response_len = sizeof resp,
    };

    app_spi_handshake_prime_result_t res = app_spi_handshake_prime(&args);
    assert(res.state == APP_SPI_HANDSHAKE_STATE_RESPONSE);
    assert(res.consumed_response == 1u);
    assert(memcmp(tx, resp, sizeof resp) == 0);
    for (size_t i = sizeof resp; i < sizeof tx; ++i) {
        assert(tx[i] == APP_SPI_STATUS_READY);
    }
}

static void test_handshake_unrecognized_status(void){
    uint8_t tx[APP_SPI_MAX_REQUEST_LEN] = {0};
    app_spi_handshake_prime_args_t args = {
        .status_byte = 0x77u,
        .tx_buf = tx,
        .tx_len = sizeof tx,
        .response_buf = NULL,
        .response_len = 0u,
    };

    app_spi_handshake_prime_result_t res = app_spi_handshake_prime(&args);
    assert(res.state == APP_SPI_HANDSHAKE_STATE_UNRECOGNIZED);
    assert(res.consumed_response == 0u);
    for (size_t i = 0; i < sizeof tx; ++i) {
        assert(tx[i] == 0x77u);
    }
}

static void test_handshake_client_poll_byte_uniqueness(void){
    assert(APP_SPI_CLIENT_POLL_BYTE != APP_SPI_STATUS_READY);
    assert(APP_SPI_CLIENT_POLL_BYTE != APP_SPI_STATUS_BUSY);
    assert(APP_SPI_CLIENT_POLL_BYTE != REQ_HEADER);
    assert(APP_SPI_CLIENT_POLL_BYTE != REQ_TAIL);
}

static void test_handshake_rejects_poll_as_status(void){
    uint8_t tx[APP_SPI_MAX_REQUEST_LEN] = {0};
    app_spi_handshake_prime_args_t args = {
        .status_byte = APP_SPI_CLIENT_POLL_BYTE,
        .tx_buf = tx,
        .tx_len = sizeof tx,
        .response_buf = NULL,
        .response_len = 0u,
    };

    app_spi_handshake_prime_result_t res = app_spi_handshake_prime(&args);
    assert(res.state == APP_SPI_HANDSHAKE_STATE_UNRECOGNIZED);
    assert(res.consumed_response == 0u);
    for (size_t i = 0; i < sizeof tx; ++i) {
        assert(tx[i] == APP_SPI_CLIENT_POLL_BYTE);
    }
}

static void test_handshake_invalid_response_len(void){
    uint8_t tx[APP_SPI_MAX_REQUEST_LEN] = {0};
    uint8_t resp[APP_SPI_MAX_REQUEST_LEN + 4];
    memset(resp, 0xAA, sizeof resp);
    app_spi_handshake_prime_args_t args = {
        .status_byte = APP_SPI_STATUS_READY,
        .tx_buf = tx,
        .tx_len = sizeof tx,
        .response_buf = resp,
        .response_len = (uint16_t)sizeof resp,
    };

    app_spi_handshake_prime_result_t res = app_spi_handshake_prime(&args);
    assert(res.state == APP_SPI_HANDSHAKE_STATE_UNRECOGNIZED);
    assert(res.consumed_response == 0u);
    for (size_t i = 0; i < sizeof tx; ++i) {
        assert(tx[i] == APP_SPI_STATUS_READY);
    }
}

static void test_handshake_compute_status(void){
    assert(app_spi_handshake_compute_status(0u, APP_SPI_MAX_REQUEST_LEN) == APP_SPI_STATUS_READY);
    assert(app_spi_handshake_compute_status(APP_SPI_MAX_REQUEST_LEN, APP_SPI_MAX_REQUEST_LEN) == APP_SPI_STATUS_BUSY);
    assert(app_spi_handshake_compute_status(APP_SPI_MAX_REQUEST_LEN + 1u, APP_SPI_MAX_REQUEST_LEN) == APP_SPI_STATUS_BUSY);
}

int main(void){
    test_move_home_req();
    test_move_probe_level_req();
    test_move_queue_add_req_parity_and_roundtrip();
    test_response_fifo_basic();
    test_handshake_ready_state();
    test_handshake_busy_state();
    test_handshake_response_state();
    test_handshake_unrecognized_status();
    test_handshake_client_poll_byte_uniqueness();
    test_handshake_rejects_poll_as_status();
    test_handshake_invalid_response_len();
    test_handshake_compute_status();
    printf("All tests passed.\n");
    return 0;
}
