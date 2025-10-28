// Protocol/router.c — versão mínima e direta
#include <string.h>
#include <stdlib.h>
#include "Protocol/router.h"
#include "Protocol/frame_defs.h"  // REQ_HEADER, REQ_TAIL, PROTO_* e REQ_* IDs

// ---------- FIFO mínima de respostas ----------
typedef struct node_s {
    uint8_t *buf;
    uint32_t len;
    struct node_s *next;
} node_t;

struct response_fifo_s {
    node_t *head, *tail;
    int count;
};

response_fifo_t* resp_fifo_create(void) {
    return (response_fifo_t*)calloc(1, sizeof(response_fifo_t));
}

void resp_fifo_destroy(response_fifo_t *q) {
    if (!q) return;
    while (q->head) {
        node_t *n = q->head;
        q->head = n->next;
        free(n->buf);
        free(n);
    }
    free(q);
}

int resp_fifo_push(response_fifo_t *q, const uint8_t *frame, uint32_t len) {
    if (!q || !frame || len == 0) return PROTO_ERR_ARG;
    node_t *n = (node_t*)malloc(sizeof(*n));
    if (!n) return PROTO_ERR_ALLOC;
    n->buf = (uint8_t*)malloc(len);
    if (!n->buf) { free(n); return PROTO_ERR_ALLOC; }
    memcpy(n->buf, frame, len);
    n->len = len;
    n->next = NULL;
    if (q->tail) q->tail->next = n; else q->head = n;
    q->tail = n;
    q->count++;
    return PROTO_OK;
}

int resp_fifo_pop(response_fifo_t *q, uint8_t *out, uint32_t max_len) {
    if (!q || !q->head || !out) return 0;
    node_t *n = q->head;
    if (n->len > max_len) return PROTO_ERR_RANGE;
    memcpy(out, n->buf, n->len);
    int ret = (int)n->len;
    q->head = n->next;
    if (!q->head) q->tail = NULL;
    q->count--;
    free(n->buf);
    free(n);
    return ret;
}

int resp_fifo_count(const response_fifo_t *q) { return q ? q->count : 0; }

// ---------- Router mínimo ----------
static router_handlers_t g_handlers;  // cópia local dos handlers

void router_init(router_t *r, response_fifo_t *resp_fifo, const router_handlers_t *h) {
    if (!r) return;
    memset(r, 0, sizeof(*r));
    r->resp = resp_fifo;
    memset(&g_handlers, 0, sizeof g_handlers);
    if (h) g_handlers = *h;
}

static void dispatch(router_t *r, const uint8_t *f, uint32_t len) {
    if (!r || !f || len < 4) return;
    uint8_t type = f[1];

    // Helper pra reduzir ruído
    #define CALL(h) do{ if (g_handlers.h) g_handlers.h(r, f, len); }while(0)

    switch (type) {
        case REQ_MOVE_QUEUE_ADD:     CALL(on_move_queue_add);     break;
        case REQ_MOVE_QUEUE_STATUS:  CALL(on_move_queue_status);  break;
        case REQ_START_MOVE:         CALL(on_start_move);         break;
        case REQ_MOVE_HOME:          CALL(on_move_home);          break;
        case REQ_MOVE_PROBE_LEVEL:   CALL(on_move_probe_level);   break;
        case REQ_MOVE_END:           CALL(on_move_end);           break;
        case REQ_LED_CTRL:           CALL(on_led_ctrl);           break;
        case REQ_STM32_STATUS:       CALL(on_fpga_status);        break;
        case REQ_SET_ORIGIN:         CALL(on_set_origin);         break;
        case REQ_ENCODER_STATUS:     CALL(on_encoder_status);     break;
        case REQ_SET_MICROSTEPS:     CALL(on_set_microsteps);     break;
        case REQ_TEST_HELLO:         CALL(on_test_hello);         break;
        default: /* desconhecido */  break;
    }
    #undef CALL
}

// Como o app já entrega um frame completo, basta validar header/tail e despachar.
void router_feed_bytes(router_t *r, const uint8_t *data, uint32_t len) {
    if (!r || !data || len < 4) return;
    if (data[0] != REQ_HEADER) return;
    if (data[len - 1] != REQ_TAIL) return;
    dispatch(r, data, len);
}
