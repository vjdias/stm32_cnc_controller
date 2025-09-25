#include <string.h>
#include <stdint.h>
#if LOG_ENABLE
#include <stdio.h>
#endif
#include "spi.h"
#include "app.h"
#include "Protocol/frame_defs.h"
#include "Protocol/router.h"
#include "Services/service_adapters.h"
#include "Services/Led/led_service.h"
#include "Services/Log/log_service.h"
#include "Services/Test/test_spi_service.h"

#define APP_SPI_MAX_REQUEST_LEN    42u
#define APP_SPI_DMA_BUF_LEN        APP_SPI_MAX_REQUEST_LEN
#define APP_SPI_RX_QUEUE_DEPTH     APP_SPI_DMA_BUF_LEN
#define APP_SPI_STATUS_READY       0xA5u
#define APP_SPI_STATUS_BUSY        0x5Au
#define APP_SPI_POLL_BYTE          0xF7u

#define APP_SPI_RX_OVERFLOW_QUEUE_FULL   0x01u
#define APP_SPI_RX_OVERFLOW_INVALID_FRAME 0x02u
/* Estados de handshake usam padrões alternados para evitar colisão com 0x00/0xFF. */

#if APP_SPI_DMA_BUF_LEN != 42u
#error "SPI DMA buffer must remain 42 bytes (payload + handshake por byte)"
#endif

typedef struct {
    uint8_t data[APP_SPI_MAX_REQUEST_LEN];
    uint16_t len;
} app_spi_frame_t;

static router_t g_router;
static router_handlers_t g_handlers;
static response_fifo_t *g_resp_fifo;

#if LOG_ENABLE
struct node_s {
    uint8_t *buf;
    uint32_t len;
    struct node_s *next;
};

struct response_fifo_s {
    struct node_s *head;
    struct node_s *tail;
    int count;
};

static int app_resp_fifo_peek(const response_fifo_t *q, uint8_t *out, uint32_t max_len) {
    if (!q || !out) {
        return 0;
    }

    if (!q->head) {
        return 0;
    }

    if (q->head->len > max_len) {
        return PROTO_ERR_RANGE;
    }

    memcpy(out, q->head->buf, q->head->len);
    return (int)q->head->len;
}
#endif

static uint8_t g_spi_rx_dma_buf[APP_SPI_DMA_BUF_LEN];
static uint8_t g_spi_tx_dma_buf[APP_SPI_DMA_BUF_LEN];
static volatile uint8_t g_spi_need_restart = 0;
static volatile uint8_t g_spi_next_status = APP_SPI_STATUS_READY;

#if defined(SCB_CleanDCache_by_Addr) || defined(SCB_InvalidateDCache_by_Addr)
#define APP_SPI_DCACHE_LINE_SIZE    32u
static void app_spi_clean_dcache(void *addr, uint32_t len);
static void app_spi_invalidate_dcache(void *addr, uint32_t len);
#endif

static app_spi_frame_t g_spi_rx_queue[APP_SPI_RX_QUEUE_DEPTH];
static volatile uint8_t g_spi_rx_queue_head = 0;
static volatile uint8_t g_spi_rx_queue_tail = 0;
static volatile uint8_t g_spi_rx_queue_count = 0;
static volatile uint8_t g_spi_rx_overflow = 0;

static uint8_t g_spi_tx_pending_buf[APP_SPI_DMA_BUF_LEN];
static volatile uint16_t g_spi_tx_pending_len = 0u;
static volatile uint8_t g_spi_tx_pending_ready = 0u;

LOG_SVC_DEFINE(LOG_SVC_APP, "app");

#if LOG_ENABLE
static void app_spi_log_tx_snapshot(uint16_t pending_len);
#endif

static void app_spi_queue_reset(void);
static uint8_t app_spi_prime_tx_buffer(uint8_t status);
static uint8_t app_spi_compute_status(void);
static void app_spi_restart_dma(uint8_t status);
static void app_spi_try_restart_dma(void);
static int app_spi_locate_frame(const uint8_t *buf, uint16_t *offset, uint16_t *len);
static int app_spi_queue_push_isr(const uint8_t *frame, uint16_t len);
static int app_spi_queue_pop(app_spi_frame_t *out);
/*
 * Resumo: trata a conclusão de uma transferência SPI DMA verificando se o
 *         buffer recebido contém um quadro válido, enfileirando-o para o
 *         processamento da camada superior e preparando o status a ser
 *         usado na próxima rodada de handshaking com o mestre.
 */
static void app_spi_handle_txrx_complete(void);

#if defined(SCB_CleanDCache_by_Addr) || defined(SCB_InvalidateDCache_by_Addr)
static uintptr_t app_spi_cache_align_down(uintptr_t addr);
static uintptr_t app_spi_cache_align_up(uintptr_t addr);
#endif

#if defined(SCB_CleanDCache_by_Addr)
static void app_spi_clean_dcache(void *addr, uint32_t len) {
    if (((SCB->CCR & SCB_CCR_DC_Msk) == 0u) || len == 0u) {
        return;
    }

    uintptr_t start = app_spi_cache_align_down((uintptr_t)addr);
    uintptr_t end = app_spi_cache_align_up((uintptr_t)addr + (uintptr_t)len);
    SCB_CleanDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}
#else
static void app_spi_clean_dcache(void *addr, uint32_t len) {
    (void)addr;
    (void)len;
}
#endif

#if defined(SCB_InvalidateDCache_by_Addr)
static void app_spi_invalidate_dcache(void *addr, uint32_t len) {
    if (((SCB->CCR & SCB_CCR_DC_Msk) == 0u) || len == 0u) {
        return;
    }

    uintptr_t start = app_spi_cache_align_down((uintptr_t)addr);
    uintptr_t end = app_spi_cache_align_up((uintptr_t)addr + (uintptr_t)len);
    SCB_InvalidateDCache_by_Addr((uint32_t *)start, (int32_t)(end - start));
}
#else
static void app_spi_invalidate_dcache(void *addr, uint32_t len) {
    (void)addr;
    (void)len;
}
#endif

#if defined(SCB_CleanDCache_by_Addr) || defined(SCB_InvalidateDCache_by_Addr)
static uintptr_t app_spi_cache_align_down(uintptr_t addr) {
    return addr & ~((uintptr_t)APP_SPI_DCACHE_LINE_SIZE - 1u);
}

static uintptr_t app_spi_cache_align_up(uintptr_t addr) {
    return (addr + ((uintptr_t)APP_SPI_DCACHE_LINE_SIZE - 1u)) &
           ~((uintptr_t)APP_SPI_DCACHE_LINE_SIZE - 1u);
}
#endif

void app_init(void) {
    led_service_init();
    log_service_init();
    test_spi_service_init();
    LOGT_THIS(LOG_STATE_START, PROTO_OK, "start", "ready");

    g_resp_fifo = resp_fifo_create();
    memset(&g_handlers, 0, sizeof g_handlers);
    services_register_handlers(&g_handlers);
    router_init(&g_router, g_resp_fifo, &g_handlers);

    app_spi_queue_reset();
    g_spi_next_status = APP_SPI_STATUS_READY;
    (void)app_spi_prime_tx_buffer(g_spi_next_status);
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, g_spi_tx_dma_buf, g_spi_rx_dma_buf,
                                    (uint16_t)APP_SPI_DMA_BUF_LEN) != HAL_OK) {
        g_spi_need_restart = 1u;
    }

    (void)test_spi_send_hello();
}

void app_poll(void) {
    app_spi_try_restart_dma();

    app_spi_frame_t frame;
    while (app_spi_queue_pop(&frame) == 0) {
        router_feed_bytes(&g_router, frame.data, frame.len);
    }

    app_spi_try_restart_dma();

    if (g_resp_fifo && !g_spi_tx_pending_ready) {
        uint8_t out[APP_SPI_DMA_BUF_LEN];
        int n = resp_fifo_pop(g_resp_fifo, out, sizeof out);
        if (n > 0 && n <= (int)APP_SPI_DMA_BUF_LEN) {
            uint16_t tx_len = (uint16_t)n;
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            memcpy(g_spi_tx_pending_buf, out, (uint32_t)tx_len);
            g_spi_tx_pending_len = tx_len;
            g_spi_tx_pending_ready = 1u;
            if (primask == 0u) {
                __enable_irq();
            }
#if LOG_ENABLE
            app_spi_log_tx_snapshot(tx_len);
#endif
        } else if (n == PROTO_ERR_RANGE) {
            LOGT_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "spi_tx", "resp too large for dma frame");
        }
    }

    if (g_spi_rx_overflow) {
        uint8_t overflow_reason = g_spi_rx_overflow;
        g_spi_rx_overflow = 0u;

        const char *reason_label = "unknown";
        if (overflow_reason == APP_SPI_RX_OVERFLOW_QUEUE_FULL) {
            reason_label = "queue_full";
        } else if (overflow_reason == APP_SPI_RX_OVERFLOW_INVALID_FRAME) {
            reason_label = "invalid_frame";
        }

        LOGA_THIS(LOG_STATE_ERROR, PROTO_WARN, "spi_rx", "overflow reason=%s", reason_label);
    }
}

void app_on_spi_txrx_half_complete(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        /* Reserva o handshake para sinalizar BUSY até concluir o tratamento atual */
        g_spi_next_status = APP_SPI_STATUS_BUSY;
    }
}

void app_on_spi_txrx_complete(SPI_HandleTypeDef *h) {
    if (h && h->Instance == SPI1) {
        app_spi_handle_txrx_complete();
    }
}

void app_on_spi_tx_complete(SPI_HandleTypeDef *h) {
    (void)h;
}

int app_resp_push(const uint8_t *frame, uint32_t len) {
    if (!g_resp_fifo || !frame || len == 0) {
        return -1;
    }
    return resp_fifo_push(g_resp_fifo, frame, len);
}

#if LOG_ENABLE
static void app_spi_format_hex(const uint8_t *buf, uint32_t len,
                               char *dst, size_t dst_len) {
    size_t pos = 0u;

    if (dst_len == 0u) {
        return;
    }

    dst[0] = '\0';

    for (uint32_t i = 0u; i < len && (pos + 2u) < dst_len; ++i) {
        int written = snprintf(&dst[pos], dst_len - pos, "%02X", (unsigned int)buf[i]);
        if (written < 0) {
            dst[0] = '\0';
            return;
        }
        if ((size_t)written >= dst_len - pos) {
            dst[dst_len - 1u] = '\0';
            return;
        }
        pos += (size_t)written;
        if ((i + 1u) < len && (pos + 1u) < dst_len) {
            dst[pos++] = ' ';
            dst[pos] = '\0';
        }
    }
}

static void app_spi_log_tx_snapshot(uint16_t pending_len) {
    if (pending_len > APP_SPI_DMA_BUF_LEN) {
        pending_len = APP_SPI_DMA_BUF_LEN;
    }

    uint8_t current_buf[APP_SPI_DMA_BUF_LEN];
    memset(current_buf, APP_SPI_STATUS_READY, sizeof current_buf);
    memcpy(current_buf, g_spi_tx_pending_buf, pending_len);

    uint8_t next_buf[APP_SPI_DMA_BUF_LEN];
    memset(next_buf, APP_SPI_STATUS_READY, sizeof next_buf);

    int next_len = 0;
    if (g_resp_fifo) {
        next_len = app_resp_fifo_peek(g_resp_fifo, next_buf, sizeof next_buf);
        if (next_len == PROTO_ERR_RANGE) {
            LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "spi_tx", "next resp exceeds dma frame");
            next_len = 0;
        }
    }

    if (next_len > 0 && next_len < (int)APP_SPI_DMA_BUF_LEN) {
        memset(&next_buf[next_len], APP_SPI_STATUS_READY,
               (size_t)(APP_SPI_DMA_BUF_LEN - (uint32_t)next_len));
    }

    char current_hex[((size_t)APP_SPI_DMA_BUF_LEN * 3u) + 1u];
    char next_hex[((size_t)APP_SPI_DMA_BUF_LEN * 3u) + 1u];
    app_spi_format_hex(current_buf, APP_SPI_DMA_BUF_LEN, current_hex, sizeof current_hex);
    app_spi_format_hex(next_buf, APP_SPI_DMA_BUF_LEN, next_hex, sizeof next_hex);

    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "spi_tx", "now=%s next=%s", current_hex, next_hex);
}
#endif

static void app_spi_queue_reset(void) {
    __disable_irq();
    g_spi_rx_queue_head = 0u;
    g_spi_rx_queue_tail = 0u;
    g_spi_rx_queue_count = 0u;
    g_spi_rx_overflow = 0u;
    __enable_irq();
}

/*
 * Resume: prepara o quadro de transmissão do DMA preenchendo-o com o status
 *         desejado e, quando existe uma resposta pendente válida, copia o
 *         conteúdo dessa resposta para o buffer antes de iniciar a nova
 *         transação SPI.
 * Variáveis locais:
 *  - used_pending: indica se uma resposta pendente foi aplicada ao buffer
 *                  de transmissão nesta preparação.
 *  - primask: guarda o valor atual do registrador PRIMASK para restaurar o
 *             estado das interrupções ao final da seção crítica.
 *  - has_pending: flag que sinaliza se há dados pendentes disponíveis para
 *                 transmissão.
 *  - pending_len: quantidade de bytes válidos armazenados no buffer pendente
 *                 que podem ser copiados para o DMA.
 */
static uint8_t app_spi_prime_tx_buffer(uint8_t status) {
    uint8_t used_pending = 0u;

    memset(g_spi_tx_dma_buf, status, APP_SPI_DMA_BUF_LEN);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint8_t has_pending = g_spi_tx_pending_ready;
    uint16_t pending_len = g_spi_tx_pending_len;
    if (has_pending && pending_len > 0u && pending_len <= APP_SPI_DMA_BUF_LEN) {
        memcpy(g_spi_tx_dma_buf, g_spi_tx_pending_buf, pending_len);
        used_pending = 1u;
    }
    if (primask == 0u) {
        __enable_irq();
    }

    app_spi_clean_dcache(g_spi_tx_dma_buf, APP_SPI_DMA_BUF_LEN);
    return used_pending;
}

/*
 * Resumo: avalia a ocupação da fila de recepção SPI para decidir qual
 *         padrão de status deve ser devolvido ao mestre durante o próximo
 *         ciclo de handshaking.
 * Variáveis globais utilizadas:
 *  - g_spi_rx_queue_count: total de quadros aguardando processamento.
 * Constantes envolvidas:
 *  - APP_SPI_RX_QUEUE_DEPTH: capacidade máxima da fila circular usada como
 *    limite para sinalizar condição de lotação.
 *  - APP_SPI_STATUS_BUSY / APP_SPI_STATUS_READY: códigos de status trocados
 *    byte a byte com o mestre SPI.
 * Fluxo geral:
 *  1. Compara a quantidade de quadros pendentes com a profundidade máxima da
 *     fila.
 *  2. Retorna BUSY quando a fila está cheia para impedir novos envios do
 *     mestre até que os dados sejam processados; caso contrário retorna READY.
 */
static uint8_t app_spi_compute_status(void) {
    return (g_spi_rx_queue_count >= APP_SPI_RX_QUEUE_DEPTH)
               ? APP_SPI_STATUS_BUSY
               : APP_SPI_STATUS_READY;
}

/*
 * Resumo: reinicializa a transferência SPI por DMA configurando o buffer de
 *         transmissão com o status desejado e, se possível, consumindo a
 *         resposta pendente pronta para envio.
 * Parâmetros:
 *  - status: byte de handshake que deve preencher o buffer de TX quando não
 *            houver payload disponível. Indica READY/BUSY ao mestre SPI.
 * Variáveis locais:
 *  - used_pending: indica se um frame armazenado previamente foi copiado para
 *                  o buffer DMA nesta tentativa de reinício.
 *  - primask: armazena o estado das interrupções antes de manipular flags
 *             compartilhadas com o contexto principal.
 * Validações executadas:
 *  1. Confere se o periférico SPI está em estado READY antes de requisitar um
 *     novo ciclo DMA; caso contrário, sinaliza a necessidade de tentar mais
 *     tarde.
 *  2. Após preparar o buffer, verifica o resultado de HAL_SPI_TransmitReceive_DMA
 *     para distinguir sucesso (limpando indicadores pendentes) de falha
 *     (marcando que outro restart será necessário).
 */
static void app_spi_restart_dma(uint8_t status) {
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        g_spi_need_restart = 1u;
        return;
    }

    g_spi_next_status = status;
    uint8_t used_pending = app_spi_prime_tx_buffer(status);
    if (HAL_SPI_TransmitReceive_DMA(&hspi1, g_spi_tx_dma_buf, g_spi_rx_dma_buf,
                                    (uint16_t)APP_SPI_DMA_BUF_LEN) == HAL_OK) {
        g_spi_need_restart = 0u;
        if (used_pending) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            g_spi_tx_pending_ready = 0u;
            g_spi_tx_pending_len = 0u;
            if (primask == 0u) {
                __enable_irq();
            }
        }
    } else {
        g_spi_need_restart = 1u;
    }
}

static void app_spi_try_restart_dma(void) {
    if (!g_spi_need_restart) {
        return;
    }
    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        return;
    }

    app_spi_restart_dma(g_spi_next_status);
}

typedef enum {
    APP_SPI_FRAME_NONE = -1,
    APP_SPI_FRAME_OK = 0,
    APP_SPI_FRAME_POLL = 1,
} app_spi_locate_result_t;

static int app_spi_locate_frame(const uint8_t *buf, uint16_t *offset, uint16_t *len) {
    if (!buf || !offset || !len || APP_SPI_DMA_BUF_LEN < 2u) {
        return APP_SPI_FRAME_NONE;
    }

    *offset = 0u;
    *len = 0u;

    uint16_t poll_matches = 0u;
    for (uint16_t i = 0u; i < APP_SPI_DMA_BUF_LEN; ++i) {
        if (buf[i] == APP_SPI_POLL_BYTE) {
            poll_matches++;
        }
    }

    if (poll_matches == APP_SPI_DMA_BUF_LEN) {
        return APP_SPI_FRAME_POLL;
    }

    uint16_t start = 0u;
    while (start < APP_SPI_DMA_BUF_LEN && buf[start] != REQ_HEADER) {
        ++start;
    }

    if (start >= APP_SPI_DMA_BUF_LEN) {
        return APP_SPI_FRAME_NONE;
    }

    for (uint16_t i = (uint16_t)(start + 1u); i < APP_SPI_DMA_BUF_LEN; ++i) {
        if (buf[i] == REQ_TAIL) {
            uint16_t frame_len = (uint16_t)(i - start + 1u);
            if (frame_len > APP_SPI_MAX_REQUEST_LEN) {
                return APP_SPI_FRAME_NONE;
            }
            *offset = start;
            *len = frame_len;
            return APP_SPI_FRAME_OK;
        }
    }

    return APP_SPI_FRAME_NONE;
}

/*
 * Resumo: copia um quadro recebido pelo DMA para a fila circular de
 *         processamento enquanto estiver no contexto de interrupção,
 *         desde que haja espaço disponível para armazená-lo.
 * Parâmetros:
 *  - frame: ponteiro para o início do quadro já localizado no buffer de
 *           recepção, permitindo que o conteúdo seja duplicado na fila.
 *  - len: tamanho efetivo do quadro a ser copiado, usado para validar o
 *         limite e registrar o comprimento na fila.
 * Variáveis locais:
 *  - slot: aponta para a posição corrente de escrita na fila circular,
 *          onde os dados e o tamanho do quadro são armazenados.
 * Fluxo geral:
 *  1. Valida argumentos de entrada e garante que o quadro caiba no limite
 *     máximo aceito pela aplicação.
 *  2. Verifica se a fila possui espaço livre; caso contrário, aborta com
 *     erro para sinalizar overflow.
 *  3. Copia os bytes do quadro para o slot atual da fila, registra o
 *     comprimento recebido e atualiza os ponteiros circulares e o contador
 *     de itens enfileirados.
 */
static int app_spi_queue_push_isr(const uint8_t *frame, uint16_t len) {
    if (!frame || len == 0u || len > APP_SPI_MAX_REQUEST_LEN) {
        return -1;
    }
    if (g_spi_rx_queue_count >= APP_SPI_RX_QUEUE_DEPTH) {
        return -1;
    }
    app_spi_frame_t *slot = &g_spi_rx_queue[g_spi_rx_queue_head];
    memcpy(slot->data, frame, len);
    slot->len = len;
    g_spi_rx_queue_head = (uint8_t)((g_spi_rx_queue_head + 1u) % APP_SPI_RX_QUEUE_DEPTH);
    g_spi_rx_queue_count++;
    return 0;
}

static int app_spi_queue_pop(app_spi_frame_t *out) {
    int rc = -1;
    __disable_irq();
    if (g_spi_rx_queue_count > 0u) {
        if (out) {
            *out = g_spi_rx_queue[g_spi_rx_queue_tail];
        }
        g_spi_rx_queue_tail = (uint8_t)((g_spi_rx_queue_tail + 1u) % APP_SPI_RX_QUEUE_DEPTH);
        g_spi_rx_queue_count--;
        rc = 0;
    }
    __enable_irq();
    return rc;
}

/*
 * Fluxo detalhado:
 *  1. Invalida a linha de dados do cache referente ao buffer DMA para garantir
 *     que a CPU leia os bytes recém recebidos pelo periférico.
 *  2. Localiza um quadro válido dentro do buffer recebido. Esta validação
 *     assegura que o framing esperado (handshake + tamanho) esteja presente
 *     antes de continuar.
 *  3. Caso o framing seja válido, tenta enfileirar o quadro para processamento
 *     posterior. O retorno de erro desta etapa cobre a validação de limites da
 *     fila circular (profundidade e comprimento máximo do quadro). Se a fila
 *     estiver cheia ou o quadro for inválido, a flag de overflow é acionada.
 *  4. Quando nenhum quadro válido é identificado, diferencia entre um ciclo de
 *     handshake (ausência de header) — que apenas mantém o status calculado a
 *     partir da ocupação atual da fila — e um frame inválido (header presente
 *     sem tail), o qual é sinalizado como overflow para análise.
 *  5. Atualiza o status do handshake: permanece em BUSY em caso de overflow ou
 *     calcula dinamicamente (READY/BUSY) com base na ocupação da fila quando o
 *     fluxo está consistente.
 *  6. Reinicia o DMA com o status apropriado para dar sequência ao ciclo de
 *     comunicação SPI.
 * Variáveis locais:
 *  - offset: posição inicial do quadro válido dentro do buffer DMA.
 *  - len: comprimento do quadro localizado; auxilia na validação de tamanho.
 *  - overflow_reason: registra o motivo do overflow (fila cheia ou frame
 *                     inválido) para repasse ao laço principal de logging.
 */
static void app_spi_handle_txrx_complete(void) {
    uint16_t offset = 0u;
    uint16_t len = 0u;
    uint8_t overflow_reason = 0u;

    app_spi_invalidate_dcache(g_spi_rx_dma_buf, APP_SPI_DMA_BUF_LEN);

    int locate_rc = app_spi_locate_frame(g_spi_rx_dma_buf, &offset, &len);
    if (locate_rc == APP_SPI_FRAME_OK) {
        if (app_spi_queue_push_isr(&g_spi_rx_dma_buf[offset], len) != 0) {
            overflow_reason = APP_SPI_RX_OVERFLOW_QUEUE_FULL;
        }
    } else if (locate_rc == APP_SPI_FRAME_NONE &&
               memchr(g_spi_rx_dma_buf, REQ_HEADER, APP_SPI_DMA_BUF_LEN) != NULL) {
        overflow_reason = APP_SPI_RX_OVERFLOW_INVALID_FRAME;
    }

    if (overflow_reason != 0u) {
        g_spi_rx_overflow = overflow_reason;
        g_spi_next_status = APP_SPI_STATUS_BUSY;
    } else {
        g_spi_next_status = app_spi_compute_status();
    }

    app_spi_restart_dma(g_spi_next_status);
}
