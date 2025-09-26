#include <string.h>
#include <stdint.h>
#include "spi.h"
#include "app.h"
#include "Protocol/frame_defs.h"
#include "Protocol/router.h"
#include "Services/service_adapters.h"
#include "Services/Led/led_service.h"
#include "Services/Log/log_service.h"
#include "Services/Test/test_spi_service.h"
#include "app_spi_handshake.h"

extern DMA_HandleTypeDef hdma_spi1_tx;

#define APP_SPI_RX_QUEUE_DEPTH     APP_SPI_DMA_BUF_LEN
typedef enum {
    APP_SPI_RX_STATUS_NONE = 0u,
    APP_SPI_RX_OVERFLOW_QUEUE_FULL = 0x01u,
    APP_SPI_RX_OVERFLOW_INVALID_FRAME = 0x02u,
} app_spi_rx_error_t;

typedef enum {
    APP_SPI_FRAME_NOT_FOUND = 0,
    APP_SPI_FRAME_FOUND,
    APP_SPI_FRAME_PARTIAL,
    APP_SPI_FRAME_INVALID,
} app_spi_frame_search_result_t;
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
static volatile uint8_t g_spi_rx_error = APP_SPI_RX_STATUS_NONE;

static uint8_t g_spi_tx_pending_buf[APP_SPI_DMA_BUF_LEN];
static volatile uint16_t g_spi_tx_pending_len = 0u;
static volatile uint8_t g_spi_tx_pending_ready = 0u;

LOG_SVC_DEFINE(LOG_SVC_APP, "app");

static void app_spi_queue_reset(void);
static uint8_t app_spi_prime_tx_buffer(uint8_t status);
static uint8_t app_spi_compute_status(void);
static void app_spi_restart_dma(uint8_t status);
static void app_spi_try_restart_dma(void);
static void app_spi_try_commit_pending_to_active(void);
static app_spi_frame_search_result_t app_spi_locate_frame(const uint8_t *buf,
                                                          uint16_t *offset,
                                                          uint16_t *len);
static int app_spi_queue_push_isr(const uint8_t *frame, uint16_t len);
static void app_spi_record_rx_error(app_spi_rx_error_t reason);
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
    app_spi_try_commit_pending_to_active();

    app_spi_frame_t frame;
    while (app_spi_queue_pop(&frame) == 0) {
        router_feed_bytes(&g_router, frame.data, frame.len);
    }

    app_spi_try_commit_pending_to_active();

    if (g_resp_fifo && !g_spi_tx_pending_ready) {
        uint8_t out[APP_SPI_DMA_BUF_LEN];
        int n = resp_fifo_pop(g_resp_fifo, out, sizeof out);
        if (n > 0 && n <= (int)APP_SPI_DMA_BUF_LEN) {
            uint32_t primask = __get_PRIMASK();
            __disable_irq();
            memset(g_spi_tx_pending_buf, 0, APP_SPI_DMA_BUF_LEN);
            memcpy(g_spi_tx_pending_buf, out, (uint32_t)n);
            g_spi_tx_pending_len = (uint16_t)n;
            g_spi_tx_pending_ready = 1u;
            if (primask == 0u) {
                __enable_irq();
            }
        } else if (n == PROTO_ERR_RANGE) {
            LOGT_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "spi_tx", "resp too large for dma frame");
        }
    }

    app_spi_try_commit_pending_to_active();

    app_spi_try_restart_dma();

    if (g_spi_rx_error != APP_SPI_RX_STATUS_NONE) {
        uint8_t reason = g_spi_rx_error;
        g_spi_rx_error = APP_SPI_RX_STATUS_NONE;
        const char *tag = "overflow reason=invalid_frame";
        if (reason == APP_SPI_RX_OVERFLOW_QUEUE_FULL) {
            tag = "overflow reason=queue_full";
        } else if (reason != APP_SPI_RX_OVERFLOW_INVALID_FRAME) {
            tag = "overflow reason=unknown";
        }
        LOGT_THIS(LOG_STATE_ERROR, PROTO_WARN, "spi_rx", tag);
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

static void app_spi_queue_reset(void) {
    __disable_irq();
    g_spi_rx_queue_head = 0u;
    g_spi_rx_queue_tail = 0u;
    g_spi_rx_queue_count = 0u;
    g_spi_rx_error = APP_SPI_RX_STATUS_NONE;
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
    uint8_t pending_copy[APP_SPI_DMA_BUF_LEN];
    uint16_t pending_len = 0u;
    uint8_t has_pending = 0u;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (g_spi_tx_pending_ready && g_spi_tx_pending_len > 0u &&
        g_spi_tx_pending_len <= APP_SPI_DMA_BUF_LEN) {
        pending_len = g_spi_tx_pending_len;
        memcpy(pending_copy, g_spi_tx_pending_buf, pending_len);
        has_pending = 1u;
    }
    if (primask == 0u) {
        __enable_irq();
    }

    uint8_t primed_status = status;
    if (has_pending && pending_len > 0u) {
        /*
         * Ao injetar um payload de resposta no quadro atual, o mestre espera
         * encontrar imediatamente o header 0xAB ao iniciar a próxima enquete.
         * Mesmo que o laço principal tenha reservado BUSY para o próximo
         * ciclo (por exemplo, durante o half-transfer do DMA), o fato de já
         * existir um frame pronto indica que estamos aptos a responder.
         * Forçar READY evita que o byte 0 (preenchimento) herde 0x5A e o host
         * interprete erroneamente a resposta como "ocupado" antes do header.
         */
        primed_status = APP_SPI_STATUS_READY;
    }

    app_spi_handshake_prime_args_t args = {
        .status_byte = primed_status,
        .tx_buf = g_spi_tx_dma_buf,
        .tx_len = APP_SPI_DMA_BUF_LEN,
        .response_buf = has_pending ? pending_copy : NULL,
        .response_len = pending_len,
    };

    app_spi_handshake_prime_result_t result = app_spi_handshake_prime(&args);

    app_spi_clean_dcache(g_spi_tx_dma_buf, APP_SPI_DMA_BUF_LEN);
    return result.consumed_response;
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
    return app_spi_handshake_compute_status(g_spi_rx_queue_count,
                                            APP_SPI_RX_QUEUE_DEPTH);
}

/*
 * Resumo: tenta aplicar uma resposta pendente diretamente ao quadro DMA
 *         já preparado quando o canal de transmissão continua ocioso, de
 *         modo que a próxima enquete do mestre receba imediatamente o
 *         payload disponível.
 * Etapas principais:
 *  1. Confirma a existência de uma resposta válida aguardando envio.
 *  2. Inspeciona o registrador CNDTR do canal de TX para garantir que nenhum
 *     byte foi transmitido na rodada corrente; caso a transferência esteja
 *     ativa, mantém o payload em espera para a próxima janela.
 *  3. Copia os dados para o buffer DMA ativo preservando o byte de handshake
 *     e higieniza o cache para que o periférico enxergue o conteúdo atualizado.
 *  4. Se o canal estiver desabilitado, apenas sinaliza a necessidade de
 *     reinício para que a rotina regular reprograme o DMA reutilizando o
 *     payload pendente.
 */
static void app_spi_try_commit_pending_to_active(void) {
    uint8_t pending_copy[APP_SPI_DMA_BUF_LEN];
    uint16_t pending_len = 0u;
    uint8_t should_commit = 0u;
    uint8_t request_restart = 0u;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    DMA_Channel_TypeDef *tx_dma = hdma_spi1_tx.Instance;
    uint8_t status_byte = g_spi_next_status;
    if (g_spi_tx_pending_ready && g_spi_tx_pending_len > 0u &&
        g_spi_tx_pending_len <= APP_SPI_DMA_BUF_LEN && tx_dma != NULL) {
        uint32_t dma_enabled = tx_dma->CCR & DMA_CCR_EN;
        if (dma_enabled == 0u) {
            request_restart = 1u;
        } else if (tx_dma->CNDTR == APP_SPI_DMA_BUF_LEN) {
            should_commit = 1u;
        }

        if (should_commit) {
            pending_len = g_spi_tx_pending_len;
            memcpy(pending_copy, g_spi_tx_pending_buf, pending_len);
            g_spi_tx_pending_ready = 0u;
            g_spi_tx_pending_len = 0u;
        }
    }

    if (!should_commit) {
        if (primask == 0u) {
            __enable_irq();
        }
        if (request_restart) {
            g_spi_need_restart = 1u;
        }
        return;
    }

    uint8_t primed_status = status_byte;
    if (pending_len > 0u) {
        /*
         * O canal possui uma resposta pronta e ainda não transmitiu nenhum
         * byte desta rodada. Ao sinalizar READY garantimos que o preenchimento
         * no início do quadro seja zerado, impedindo que o mestre observe
         * 0x5A (BUSY) antes do header 0xAB ao efetuar o polling.
         */
        primed_status = APP_SPI_STATUS_READY;
    }

    app_spi_handshake_prime_args_t args = {
        .status_byte = primed_status,
        .tx_buf = g_spi_tx_dma_buf,
        .tx_len = APP_SPI_DMA_BUF_LEN,
        .response_buf = pending_copy,
        .response_len = pending_len,
    };

    app_spi_handshake_prime_result_t result = app_spi_handshake_prime(&args);

    if (result.consumed_response) {
        app_spi_clean_dcache(g_spi_tx_dma_buf, APP_SPI_DMA_BUF_LEN);
    } else {
        memcpy(g_spi_tx_pending_buf, pending_copy, pending_len);
        g_spi_tx_pending_len = pending_len;
        g_spi_tx_pending_ready = 1u;
    }

    if (primask == 0u) {
        __enable_irq();
    }

    if (request_restart) {
        g_spi_need_restart = 1u;
    }
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

static app_spi_frame_search_result_t app_spi_locate_frame(const uint8_t *buf,
                                                          uint16_t *offset,
                                                          uint16_t *len) {
    if (!buf || !offset || !len || APP_SPI_DMA_BUF_LEN < 2u) {
        return APP_SPI_FRAME_NOT_FOUND;
    }

    uint16_t start = 0u;
    /* O mestre envia APP_SPI_CLIENT_POLL_BYTE durante o polling; aqui
     * caminhamos até localizar de fato o header 0xAA ignorando esses bytes. */
    while (start < APP_SPI_DMA_BUF_LEN && buf[start] != REQ_HEADER) {
        ++start;
    }

    if (start >= APP_SPI_DMA_BUF_LEN) {
        return APP_SPI_FRAME_NOT_FOUND;
    }

    for (uint16_t i = (uint16_t)(start + 1u); i < APP_SPI_DMA_BUF_LEN; ++i) {
        if (buf[i] == REQ_TAIL) {
            uint16_t frame_len = (uint16_t)(i - start + 1u);
            if (frame_len > APP_SPI_MAX_REQUEST_LEN) {
                return APP_SPI_FRAME_INVALID;
            }
            *offset = start;
            *len = frame_len;
            return APP_SPI_FRAME_FOUND;
        }
    }

    return APP_SPI_FRAME_PARTIAL;
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

static void app_spi_record_rx_error(app_spi_rx_error_t reason) {
    if (reason == APP_SPI_RX_STATUS_NONE) {
        return;
    }
    if (g_spi_rx_error == APP_SPI_RX_STATUS_NONE) {
        g_spi_rx_error = (uint8_t)reason;
    }
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
 *     posterior. Se a fila estiver cheia ou o quadro ultrapassar o limite
 *     configurado, a rotina registra o motivo através de `app_spi_record_rx_error`
 *     para que o laço principal reporte o evento.
 *  4. A decisão do próximo handshake considera o resultado da etapa anterior:
 *     - Quando um quadro entra na fila, `app_spi_compute_status` avalia a
 *       ocupação para decidir entre READY/BUSY.
 *     - Em situações de erro, forçamos BUSY para indicar ao mestre que o
 *       pedido precisa ser reenviado.
 *     - Caso nenhum quadro completo seja identificado, mantemos o status
 *       baseado na ocupação atual da fila (normalmente READY).
 *  5. Reinicia o DMA com o status apropriado para dar sequência ao ciclo de
 *     comunicação SPI.
 * Variáveis locais:
 *  - offset: posição inicial do quadro válido dentro do buffer DMA.
 *  - len: comprimento do quadro localizado; auxilia na validação de tamanho.
 *  - search: resultado da inspeção do buffer (encontrado, parcial, inválido).
 */
/*
 * Integra regras de polling com 0x3C e respostas de serviços:
 *  - Quando o mestre apenas consulta usando APP_SPI_CLIENT_POLL_BYTE sem um
 *    quadro completo, consideramos que ele espera o próximo resultado dos
 *    serviços. Nesse caso, devolvemos READY se não houver payload pronto, de
 *    modo que o host entenda que nada novo foi produzido.
 *  - Quando existe uma resposta pendente, ela já foi preparada com zeros à
 *    esquerda (42 - len(msg)) garantindo que os bytes de status (READY/BUSY)
 *    sejam completamente sobrescritos pelo conteúdo contextual do serviço.
 *  - Nos demais cenários (quadro válido, inválido ou fila cheia) preservamos o
 *    protocolo de handshake anterior para sinalizar necessidade de retry.
 *  - Após decidir o próximo status, adiamos o reinício do DMA para o laço
 *    principal (`app_poll`), permitindo que qualquer mensagem recém-enfileirada
 *    pelos serviços seja aplicada ao buffer ativo antes que o Raspberry Pi
 *    realize a próxima enquete.
 */
static void app_spi_handle_txrx_complete(void) {
    uint16_t offset = 0u;
    uint16_t len = 0u;

    app_spi_invalidate_dcache(g_spi_rx_dma_buf, APP_SPI_DMA_BUF_LEN);

    app_spi_frame_search_result_t search =
        app_spi_locate_frame(g_spi_rx_dma_buf, &offset, &len);

    uint8_t next_status = APP_SPI_STATUS_BUSY;
    uint8_t first_byte = g_spi_rx_dma_buf[0];
    uint8_t is_poll = (first_byte == APP_SPI_CLIENT_POLL_BYTE);

    switch (search) {
    case APP_SPI_FRAME_FOUND:
        if (app_spi_queue_push_isr(&g_spi_rx_dma_buf[offset], len) == 0) {
            next_status = app_spi_compute_status();
        } else {
            app_spi_record_rx_error(APP_SPI_RX_OVERFLOW_QUEUE_FULL);
            next_status = APP_SPI_STATUS_BUSY;
        }
        break;
    case APP_SPI_FRAME_INVALID:
        app_spi_record_rx_error(APP_SPI_RX_OVERFLOW_INVALID_FRAME);
        next_status = APP_SPI_STATUS_BUSY;
        break;
    case APP_SPI_FRAME_PARTIAL:
    case APP_SPI_FRAME_NOT_FOUND:
    default:
        next_status = is_poll ? APP_SPI_STATUS_READY : app_spi_compute_status();
        break;
    }

    /*
     * Com o tratamento do quadro concluído, armazenamos o próximo status para
     * que o laço principal reprograme o DMA logo após processar a fila de
     * serviços. Isso dá tempo para que qualquer resposta recém-gerada seja
     * copiada para o buffer ativo antes do próximo polling do Raspberry Pi,
     * garantindo que payloads pendentes substituam os bytes READY/BUSY na
     * primeira transferência de enquete.
     */
    g_spi_next_status = next_status;
    g_spi_need_restart = 1u;
}
