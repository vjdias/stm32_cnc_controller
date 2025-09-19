// Roteador mínimo e autocontido (não é vinculado por padrão no CubeIDE)
#include <string.h>
#include <stdlib.h>
#include "Protocol/router.h"

typedef struct node_s {
	uint8_t *buf;
	uint32_t len;
	struct node_s *next;
} node_t;

struct response_fifo_s {
	node_t *head;
	node_t *tail;
	int count;
};

static router_handlers_t handlers;

void router_init(router_t *r, response_fifo_t *resp_fifo,
		const router_handlers_t *h) {
	memset(r, 0, sizeof(*r));
	r->resp = resp_fifo;
	if (h) {
		handlers = *h;
	}
}

static int is_req_complete(const uint8_t *a, uint32_t n) {
	if (n < 4)
		return 0; // mínimo
	if (a[0] != REQ_HEADER)
		return -1; // inválido
	// fim quando encontrar REQ_TAIL
	for (uint32_t i = 3; i < n; i++) {
		if (a[i] == REQ_TAIL)
			return (int) (i + 1);
	}
	return 0;
}

static void dispatch(router_t *r, const uint8_t *f, uint32_t len) {
	if (len < 4)
		return;
	uint8_t type = f[1];
	switch (type) {
	case REQ_MOVE_QUEUE_ADD:
		if (handlers.on_move_queue_add)
			handlers.on_move_queue_add(r, f, len);
		break;
	case REQ_MOVE_QUEUE_STATUS:
		if (handlers.on_move_queue_status)
			handlers.on_move_queue_status(r, f, len);
		break;
	case REQ_START_MOVE:
		if (handlers.on_start_move)
			handlers.on_start_move(r, f, len);
		break;
	case REQ_MOVE_HOME:
		if (handlers.on_move_home)
			handlers.on_move_home(r, f, len);
		break;
	case REQ_MOVE_PROBE_LEVEL:
		if (handlers.on_move_probe_level)
			handlers.on_move_probe_level(r, f, len);
		break;
	case REQ_MOVE_END:
		if (handlers.on_move_end)
			handlers.on_move_end(r, f, len);
		break;
	case REQ_LED_CTRL:
		if (handlers.on_led_ctrl)
			handlers.on_led_ctrl(r, f, len);
		break;
	case REQ_FPGA_STATUS:
		if (handlers.on_fpga_status)
			handlers.on_fpga_status(r, f, len);
		break;
	default:
		break; // desconhecido
	}
}

void router_feed_bytes(router_t *r, const uint8_t *data, uint32_t len) {
	for (uint32_t i = 0; i < len; i++) {
		if (r->idx >= sizeof(r->acc))
			r->idx = 0; // evita overflow simples
		r->acc[r->idx++] = data[i];
		int comp = is_req_complete(r->acc, r->idx);
		if (comp < 0) {
			r->idx = 0;
			continue;
		} // descarta até header
		if (comp > 0) {
			dispatch(r, r->acc, (uint32_t) comp);
			r->idx = 0;
		}
	}
}

response_fifo_t* resp_fifo_create(void) {
	response_fifo_t *q = (response_fifo_t*) calloc(1, sizeof(*q));
	return q;
}
void resp_fifo_destroy(response_fifo_t *q) {
	while (q && q->head) {
		node_t *n = q->head;
		q->head = n->next;
		free(n->buf);
		free(n);
	}
	free(q);
}
int resp_fifo_push(response_fifo_t *q, const uint8_t *frame, uint32_t len) {
	if (!q || !frame || len == 0)
		return PROTO_ERR_ARG;
	node_t *n = (node_t*) malloc(sizeof(*n));
	if (!n)
		return PROTO_ERR_ALLOC;
	n->buf = (uint8_t*) malloc(len);
	if (!n->buf) {
		free(n);
		return PROTO_ERR_ALLOC;
	}
	memcpy(n->buf, frame, len);
	n->len = len;
	n->next = NULL;
	if (q->tail)
		q->tail->next = n;
	else
		q->head = n;
	q->tail = n;
	q->count++;
	return PROTO_OK;
}
int resp_fifo_pop(response_fifo_t *q, uint8_t *out, uint32_t max_len) {
	if (!q || !q->head || !out)
		return 0;
	node_t *n = q->head;
	if (n->len > max_len)
		return PROTO_ERR_RANGE;
	memcpy(out, n->buf, n->len);
	int l = (int) n->len;
	q->head = n->next;
	if (!q->head)
		q->tail = NULL;
	q->count--;
	free(n->buf);
	free(n);
	return l;
}
int resp_fifo_count(const response_fifo_t *q) {
	return q ? q->count : 0;
}
