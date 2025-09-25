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
        case REQ_TEST_HELLO:
                if (handlers.on_test_hello)
                        handlers.on_test_hello(r, f, len);
                break;
        default:
                break; // desconhecido
        }
}

/*
 * Resumo: consome um bloco de bytes recém-chegados do barramento e alimenta o
 *         acumulador do roteador até identificar um frame completo, momento em
 *         que o encaminha para o despachante adequado com base no tipo
 *         codificado.
 * Parâmetros:
 *  - r: instância do roteador que mantém o buffer acumulador, índice atual e
 *       ponteiro para a FIFO de respostas usada pelas rotinas de dispatch.
 *  - data: sequência de bytes recebidos do SPI/UART que devem ser analisados e
 *          agregados ao acumulador.
 *  - len: quantidade de bytes válidos em data que precisam ser processados
 *         nesta chamada.
 * Variáveis e validações internas:
 *  - i: índice do laço que percorre byte a byte o bloco recebido.
 *  - r->idx: quando alcança o tamanho do acumulador, é zerado para evitar
 *            overflow e reiniciar a captura a partir do próximo header.
 *  - comp: resultado de is_req_complete; negativo sinaliza frame inválido
 *          (reseta o índice), zero mantém a captura e positivo indica o número
 *          de bytes que formam um frame completo pronto para dispatch.
 */
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
/*
 * Resumo: enfileira um quadro de resposta copiando seus bytes para um nó
 *         alocado dinamicamente ao final da FIFO encadeada.
 * Parâmetros:
 *  - q: fila de respostas que receberá o quadro e terá ponteiros/contador
 *       atualizados para refletir a nova entrada.
 *  - frame: buffer de origem com o conteúdo a ser duplicado, preservando o
 *           armazenamento original enquanto o roteador opera com uma cópia.
 *  - len: quantidade de bytes válidos no quadro, usada tanto para validar a
 *         chamada quanto para dimensionar a alocação do buffer privado.
 * Validações internas:
 *  - Garante que a fila, o ponteiro de dados e o tamanho sejam válidos antes
 *    de iniciar qualquer alocação, retornando PROTO_ERR_ARG quando um requisito
 *    básico não é atendido.
 *  - Verifica o resultado de cada chamada a malloc e libera recursos
 *    intermediários em caso de falha, sinalizando PROTO_ERR_ALLOC para evitar
 *    vazamento e deixar claro o motivo do erro.
 * Estrutura associada:
 *  - response_fifo_s: mantém ponteiros head/tail para o primeiro e último nós
 *    encadeados (node_t) e um contador de elementos. Durante o push a função
 *    atualiza tail->next para apontar para o novo nó, ajusta head quando a fila
 *    estava vazia e incrementa count para refletir a nova ocupação.
 * Variáveis locais:
 *  - n: nó recém-criado que mantém a cópia do quadro e os ponteiros de
 *       encadeamento necessários para anexá-lo à lista.
 * Exemplo prático:
 *  - Fila recém-criada: head/tail = NULL e count = 0.
 *  - Primeiro push ("ACK", len = 3): aloca n1, copia os bytes e como tail é
 *    NULL o nó passa a ser head e tail. O contador sobe para 1.
 *  - Segundo push ("ERR", len = 3): aloca n2 e encadeia em tail->next. A
 *    cabeça permanece em n1, tail passa a apontar para n2 e count torna-se 2.
 *  - Um pop subsequente retira n1, promovendo head para n2; quando todos os
 *    nós forem removidos, head/tail voltam a NULL e count zera novamente.
 * Fluxo geral:
 *  1. Valida os argumentos recebidos.
 *  2. Aloca o nó e o buffer dedicado para armazenar a cópia do quadro,
 *     tratando eventuais falhas de memória.
 *  3. Copia os dados, atualiza encadeamento e contador da fila e devolve
 *     PROTO_OK indicando inserção bem-sucedida.
 */
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
