#include <string.h>
#include "Services/service_adapters.h"
#include "Services/Log/log_service.h"
#include "Services/Led/led_service.h"
#include "Services/Home/home_service.h"
#include "Services/Probe/probe_service.h"
#include "Services/Safety/safety_service.h"
#include "Services/Motion/motion_service.h"
#include "app.h"
#include "stm32l4xx.h"  /* ITM/DBGMCU presence for optional SWO dump */

/*==============================================================================
 *  SPI (Slave) + DMA – TX/RX de 42 bytes
 *  Layout fixo do frame de TX:
 *    - [0..21] (22 bytes): sempre 0x00
 *    - [22..41] (20 bytes): payload alinhado à direita
 *      - COM resposta: toda a janela é zerada e o payload é copiado à direita
 *      - SEM resposta (poll): janela preenchida com SPI_FILL_BYTE (ex.: 0xA5)
 *==============================================================================*/

/* --- Handle gerado pelo CubeMX --------------- */
extern SPI_HandleTypeDef hspi2;

/* --- Estado e buffers ------------------------------------------------------- */
static router_t             g_router;
static router_handlers_t    g_handlers;
static response_fifo_t     *g_resp_fifo = NULL;

static uint8_t g_spi_rx_dma_buf[APP_SPI_DMA_BUF_LEN];
static uint8_t g_spi_tx_dma_buf[APP_SPI_DMA_BUF_LEN];

static volatile uint8_t         g_spi_round_done = 0u;
static volatile uint8_t         g_spi_error_flag = 0u;
static volatile app_spi_state_t g_state          = APP_SPI_READY;

/* Opcional: dump do payload de TX via SWO (ITM) antes de armar o DMA.
 * - Leve: envia um marcador 'T', em seguida os bytes do payload e um '\n'.
 * - Só emite quando há resposta (n>0) e SWO/ITM está habilitado no core.
 * - Mantém overhead mínimo e não altera o timing do SPI.
 */
#ifndef APP_SWO_TX_DUMP_ENABLE
#define APP_SWO_TX_DUMP_ENABLE 1u
#endif
#ifndef APP_SWO_TX_DUMP_CHANNEL
#define APP_SWO_TX_DUMP_CHANNEL 0u
#endif

static inline int swo_enabled_app(void)
{
    return ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) &&
            (DBGMCU->CR & DBGMCU_CR_TRACE_IOEN) &&
            (ITM->TCR & ITM_TCR_ITMENA_Msk) &&
            (ITM->TER & (1UL << APP_SWO_TX_DUMP_CHANNEL)));
}

static inline void swo_dump_tx_payload(const uint8_t *p, uint16_t n)
{
    if (!p || n == 0u) return;
    ITM_SendChar('T');
    for (uint16_t i = 0; i < n; ++i) {
        ITM_SendChar(p[i]);
    }
    ITM_SendChar('\n');
}

/*==============================================================================
 *  Layout fixo do TX (42 = 22 + 20)
 *==============================================================================*/
#define RESP_LEFT_PAD_LEN   22u   /* bytes [0..21]  = 0x00                    */
#define RESP_RIGHT_LEN      20u   /* bytes [22..41] = payload (até 20 bytes)  */


/**
 * @brief Preenche o buffer de TX com 22×0x00 à esquerda e 20×SPI_FILL_BYTE à direita.
 * @param dst Ponteiro para o buffer de TX (tamanho = 42).
 */
static inline void tx_fill_left_zero_right_filler(uint8_t *dst)
{
    memset(&dst[0],                 0x00,           RESP_LEFT_PAD_LEN); /* 22×0x00  */
    memset(&dst[RESP_LEFT_PAD_LEN], SPI_FILL_BYTE,  RESP_RIGHT_LEN);    /* 20×filler */
}

/*==============================================================================
 *  Helpers mínimos
 *==============================================================================*/

/**
 * @brief Verifica se todo o buffer tem o mesmo valor.
 * @param buf Buffer de entrada (tamanho = 42).
 * @param val Valor esperado em todos os bytes.
 * @return 1 se todos os bytes são 'val', 0 caso contrário.
 */
static int is_fill42(const uint8_t *buf, uint8_t val)
{
    for (uint32_t i = 0; i < APP_SPI_DMA_BUF_LEN; ++i)
        if (buf[i] != val) return 0;
    return 1;
}

/**
 * @brief Localiza um quadro bem formado [REQ_HEADER ... REQ_TAIL] no RX.
 * @param buf  Buffer de entrada (tamanho = 42).
 * @param off  (out) Offset de início do quadro (REQ_HEADER).
 * @param len  (out) Comprimento do quadro (inclui REQ_TAIL).
 * @return 1 se encontrou, 0 caso contrário.
 */
static int find_frame(const uint8_t *buf, uint16_t *off, uint16_t *len)
{
    uint16_t i = 0;
    while (i < APP_SPI_DMA_BUF_LEN && buf[i] != REQ_HEADER) i++;
    if (i >= APP_SPI_DMA_BUF_LEN) return 0;

    for (uint16_t j = (uint16_t)(i + 1); j < APP_SPI_DMA_BUF_LEN; ++j) {
        if (buf[j] == REQ_TAIL) {
            *off = i;
            *len = (uint16_t)(j - i + 1u);
            return 1;
        }
    }
    return 0;
}

/*==============================================================================
 *  Preparação do TX
 *==============================================================================*/

/**
 * @brief Prepara o buffer de TX para a próxima rodada de DMA.
 *
 * Regras:
 *  - COM resposta (n > 0):
 *      * Zera TODO o frame (garante que não haja filler visível);
 *      * Copia o payload alinhado à direita dentro dos últimos 20 bytes.
 *      * Se n > 20, trunca para os 20 últimos bytes.
 *  - SEM resposta:
 *      * 22×0x00 + 20×SPI_FILL_BYTE (poll).
 */
static void prepare_next_tx(void)
{
    uint8_t tmp[APP_SPI_DMA_BUF_LEN];
    int n = 0;

    if (!g_resp_fifo) {
        /* Sem fila -> 22×0x00 + 20×filler */
        tx_fill_left_zero_right_filler(g_spi_tx_dma_buf);
        g_state = APP_SPI_READY;
        return;
    }

    n = resp_fifo_pop(g_resp_fifo, tmp, (int)APP_SPI_DMA_BUF_LEN);
    if (n > 0) {
        /* Zera TODO o frame para evitar A5 antes do payload */
        memset(g_spi_tx_dma_buf, 0x00, APP_SPI_DMA_BUF_LEN);

        /* Copia o payload alinhado à direita (trunca se necessário) */
        uint16_t to_copy = (uint16_t)((n > (int)RESP_RIGHT_LEN) ? RESP_RIGHT_LEN : n);
        uint16_t dst_off = (uint16_t)(APP_SPI_DMA_BUF_LEN - to_copy); /* 42 - to_copy */
        uint16_t src_off = (uint16_t)(n - to_copy);                   /* últimos 'to_copy' bytes */

        memcpy(&g_spi_tx_dma_buf[dst_off], &tmp[src_off], to_copy);
#if APP_SWO_TX_DUMP_ENABLE
        if (swo_enabled_app()) {
            swo_dump_tx_payload(&g_spi_tx_dma_buf[dst_off], to_copy);
        }
#endif
        g_state = APP_SPI_PENDING;
    } else {
        /* Sem resposta -> mantém contrato visual: 22×0x00 + 20×filler */
        tx_fill_left_zero_right_filler(g_spi_tx_dma_buf);
        g_state = APP_SPI_READY;
    }
}

/*==============================================================================
 *  DMA restart
 *==============================================================================*/

/**
 * @brief Reinicia uma transação SPI por DMA (não bloqueante).
 * Seta g_state=BUSY em caso de sucesso; seta g_spi_error_flag em erro.
 */
static void restart_spi_dma(void)
{
    /* Somente reinicia quando a periferia está realmente pronta. Evita falso erro
       quando se utiliza DMA em modo NORMAL e o HAL ainda está finalizando a rodada. */
    if (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY) {
        return;
    }

    if (HAL_SPI_TransmitReceive_DMA(&hspi2,
            g_spi_tx_dma_buf, g_spi_rx_dma_buf,
            (uint16_t)APP_SPI_DMA_BUF_LEN) != HAL_OK) {
        g_spi_error_flag = 1u;
        return;
    }

    g_state = APP_SPI_BUSY;
}

/*==============================================================================
 *  API pública
 *==============================================================================*/

/**
 * @brief Inicializa roteador, fila de respostas e a primeira rodada DMA.
 * Preenche o primeiro TX como 22×0x00 + 20×filler.
 */
void app_init(void)
{
    /* Registra serviços no router (o projeto deve prover os handlers) */
    memset(&g_handlers, 0, sizeof g_handlers);
    services_register_handlers(&g_handlers);

    /* Inicializa serviços (ordem: log/diag, safety, periféricos simples, motion) */
#if LOG_ENABLE
    log_service_init();
#endif
    safety_service_init();
    led_service_init();
    home_service_init();
    probe_service_init();
    motion_service_init();

    g_resp_fifo = resp_fifo_create();
    router_init(&g_router, g_resp_fifo, &g_handlers);

    /* Primeiro frame: 22×0x00 + 20×filler (evita A5 no início quando não há resposta) */
    tx_fill_left_zero_right_filler(g_spi_tx_dma_buf);

    restart_spi_dma();
    g_state = APP_SPI_READY;
}

/**
 * @brief Loop de serviço: processa RX (quando há rodada concluída),
 *        prepara o próximo TX e rearma o DMA.
 */
void app_poll(void)
{
    /* Só processa quando um round DMA foi concluído pelo HAL */
    if (!g_spi_round_done) return;
    g_spi_round_done = 0u;

    /* 1) Interpretar o RX atual */
    if (is_fill42(g_spi_rx_dma_buf, SPI_POLL_BYTE)) {
        /* 42×0x3C => cliente apenas leu respostas; não alimenta router */
    } else {
        /* Tenta extrair [REQ_HEADER ... REQ_TAIL] e empurra para o router */
        uint16_t off = 0, len = 0;
        if (find_frame(g_spi_rx_dma_buf, &off, &len)) {
            router_feed_bytes(&g_router, &g_spi_rx_dma_buf[off], len);
        } else {
            /* Quadro inválido/parcial ou outro padrão -> marca erro leve */
            g_spi_error_flag = 1u;
        }
    }

    /* 2) Preparar TX (resposta à direita ou 22×0x00 + 20×filler) */
    prepare_next_tx();

    /* 3) Reiniciar DMA para o próximo round */
    restart_spi_dma();
}

/**
 * @brief Retorna o estado atual do SPI app.
 */
app_spi_state_t app_spi_get_state(void)
{
    return g_state;
}

/**
 * @brief Retorna o flag de erro (1 se houve erro desde o último clear externo).
 */
uint8_t app_spi_get_error(void)
{
    return g_spi_error_flag;
}

/*==============================================================================
 *  Callbacks do HAL (chamados em ISR)
 *==============================================================================*/

/**
 * @brief Callback para “transfer complete” do SPI+DMA.
 * Apenas sinaliza o loop principal (app_poll) via g_spi_round_done.
 */
void app_spi_isr_txrx_done(SPI_HandleTypeDef *hspi)
{
    if (!hspi) return;
    if (hspi->Instance != APP_SPI_INSTANCE) return;
    g_spi_round_done = 1u;
}

/* HAL callbacks são implementados em Core/Src/main.c e chamam app_spi_isr_txrx_done(). */

/*==============================================================================
 *  Fila de respostas (API do protocolo)
 *==============================================================================*/

/**
 * @brief Empurra um frame de resposta para a fila de transmissão.
 * @param frame Buffer com a resposta (número de bytes = len).
 * @param len   Tamanho da resposta (até 20 bytes). >20 retorna erro.
 * @return 0 em sucesso; PROTO_ERR_ARG ou PROTO_ERR_RANGE em erro.
 */
int app_resp_push(const uint8_t *frame, uint32_t len)
{
    if (!g_resp_fifo || !frame || len == 0u) {
        return PROTO_ERR_ARG;
    }
    if (len > RESP_RIGHT_LEN) {
        return PROTO_ERR_RANGE; /* impede >20 bytes */
    }
    return resp_fifo_push(g_resp_fifo, frame, len);
}
