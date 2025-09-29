#include <string.h>
#include "Services/service_adapters.h"
#include "app.h"
#include "stm32l4xx_hal_spi_ex.h"

// Handle gerado pelo CubeMX (ajuste o nome se necessário)
extern SPI_HandleTypeDef hspi1;

// ----------------- Estado e buffers -----------------
static router_t            g_router;
static router_handlers_t   g_handlers;
static response_fifo_t    *g_resp_fifo = NULL;

static uint8_t g_spi_rx_dma_buf[APP_SPI_DMA_BUF_LEN];
static uint8_t g_spi_tx_dma_buf[APP_SPI_DMA_BUF_LEN];

static volatile uint8_t        g_spi_round_done = 0u;
static volatile uint8_t        g_spi_error_flag = 0u;
static volatile app_spi_state_t g_state         = APP_SPI_READY;

// ----------------- Helpers mínimos -----------------

// Verifica se todo o buffer é preenchido por "val" (ex.: 42×0x3C)
static int is_fill42(const uint8_t *buf, uint8_t val) {
    for (uint32_t i = 0; i < APP_SPI_DMA_BUF_LEN; ++i)
        if (buf[i] != val) return 0;
    return 1;
}

// Localiza um quadro [0xAA ... 0x55] dentro do buffer
static int find_frame(const uint8_t *buf, uint16_t *off, uint16_t *len) {
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

// Prepara TX para o próximo round: copia resposta (se houver) ou 42×A5
// Prepara TX para o próximo round:
// - Se houver resposta (n > 0): zera o quadro e alinha a resposta à direita.
// - Se não houver: 42×A5 (filler/poll sem payload).
static void prepare_next_tx(void) {
    uint8_t tmp[APP_SPI_DMA_BUF_LEN];
    int n = 0;

    if (!g_resp_fifo) {
        // Sem fila -> filler A5
        memset(g_spi_tx_dma_buf, SPI_FILL_BYTE, APP_SPI_DMA_BUF_LEN);
        g_state = APP_SPI_READY;
        return;
    }

    n = resp_fifo_pop(g_resp_fifo, tmp, (int)APP_SPI_DMA_BUF_LEN);
    if (n > 0) {
        // >>> comportamento desejado <<<
        // zeros à esquerda + payload encodado (com 0xAB no começo) à direita
        memset(g_spi_tx_dma_buf, 0x00, APP_SPI_DMA_BUF_LEN);
        uint16_t off = (uint16_t)(APP_SPI_DMA_BUF_LEN - (uint16_t)n);
        memcpy(&g_spi_tx_dma_buf[off], tmp, (size_t)n);
        g_state = APP_SPI_PENDING;
    } else {
        // sem resposta -> 42 × A5
        memset(g_spi_tx_dma_buf, SPI_FILL_BYTE, APP_SPI_DMA_BUF_LEN);
        g_state = APP_SPI_READY;
    }
}


// Reinicia uma transação DMA (não bloqueante)
static void        spi_rx_fifo_drain(SPI_HandleTypeDef *hspi);
static inline void spi_post_dma_rx_fifo_sanity(SPI_HandleTypeDef *hspi);

static void restart_spi_dma(void) {
    // Antes de iniciar uma nova rodada DMA, garanta que o FIFO RX esteja limpo.
    spi_post_dma_rx_fifo_sanity(&hspi1);

    if (HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY) {
        g_spi_error_flag = 1u;
        return;
    }
    if (HAL_SPI_TransmitReceive_DMA(&hspi1,
            g_spi_tx_dma_buf, g_spi_rx_dma_buf,
            (uint16_t)APP_SPI_DMA_BUF_LEN) != HAL_OK) {
        g_spi_error_flag = 1u;
        return;
    }
    g_state = APP_SPI_BUSY;
}

// ----------------- API -----------------

void app_init(void) {
    // Registra serviços no router (seu projeto já deve prover isso)
    memset(&g_handlers, 0, sizeof g_handlers);
    // Se você usa adapters de serviços, registre aqui:
    services_register_handlers(&g_handlers);

    g_resp_fifo = resp_fifo_create();
    router_init(&g_router, g_resp_fifo, &g_handlers);

    // Primeiro frame: apenas filler
    memset(g_spi_tx_dma_buf, SPI_FILL_BYTE, APP_SPI_DMA_BUF_LEN);
    restart_spi_dma();
    g_state = APP_SPI_READY;
}

void app_poll(void) {
    // Só processa quando um round DMA foi concluído pelo HAL
    if (!g_spi_round_done) return;
    g_spi_round_done = 0u;

    // 1) Interpretar o RX atual
    if (is_fill42(g_spi_rx_dma_buf, SPI_POLL_BYTE)) {
        // 42×0x3C => cliente apenas leu respostas; não alimenta router
    } else {
        // Tenta extrair [0xAA ... 0x55] e empurrar para o router
        uint16_t off = 0, len = 0;
        if (find_frame(g_spi_rx_dma_buf, &off, &len)) {
            router_feed_bytes(&g_router, &g_spi_rx_dma_buf[off], len);
        } else {
            // quadro inválido/parcial ou outro padrão -> marca erro
            g_spi_error_flag = 1u;
        }
    }

    // 2) Preparar TX (resposta ou 42×A5)
    prepare_next_tx();

    // 3) Reiniciar o DMA para o próximo round
    restart_spi_dma();
}

app_spi_state_t app_spi_get_state(void) { return g_state; }
uint8_t         app_spi_get_error(void) { return g_spi_error_flag; }

// ----------------- Chamadas a partir dos callbacks do HAL -----------------

void app_spi_isr_txrx_done(SPI_HandleTypeDef *hspi) {
    if (!hspi) return;
    if (hspi->Instance != APP_SPI_INSTANCE) return;

    // Saneia o FIFO RX assim que o DMA sinalizar término para evitar deslocamentos.
    spi_post_dma_rx_fifo_sanity(hspi);

    // Sinaliza para o loop principal processar RX->router e TX->DMA
    g_spi_round_done = 1u;
}

int app_resp_push(const uint8_t *frame, uint32_t len) {
    if (!g_resp_fifo || !frame || len == 0u) {
        return PROTO_ERR_ARG;
    }
    // opcional: proteger contra resposta maior que o quadro DMA
    if (len > APP_SPI_DMA_BUF_LEN) {
        return PROTO_ERR_RANGE;
    }
    return resp_fifo_push(g_resp_fifo, frame, len);
}

/* ===========================  Proteções do FIFO RX  ===========================

   Problema tratado:
   - Em modo full-duplex com DMA (normal mode), underruns pontuais podem deixar
     bytes residuais no FIFO RX após o término da transação, deslocando o frame
     da rodada seguinte.

   Estratégia:
    - Ao término do DMA (callback) drenamos incondicionalmente o FIFO RX, o
      que remove bytes residuais mesmo quando FRLVL/RXNE reportam vazio.
    - Repetimos o mesmo saneamento imediatamente antes de disparar uma nova
      rodada, garantindo que o início do próximo frame esteja alinhado.

   Observação:
   - Código assume DataSize = 8 bits; ajuste a leitura caso utilize 16 bits.
   ============================================================================ */

static void spi_rx_fifo_drain(SPI_HandleTypeDef *hspi) {
    if (!hspi) {
        return;
    }

#if defined(HAL_SPI_MODULE_ENABLED)
    if (HAL_SPIEx_FlushRxFifo(hspi) != HAL_OK) {
        uint32_t guard = 0u;
        while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE) && guard++ < 64u) {
            (void)*(__IO uint8_t *)&hspi->Instance->DR;
        }
    }
#else
    uint32_t guard = 0u;
    while (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXNE) && guard++ < 64u) {
        (void)*(__IO uint8_t *)&hspi->Instance->DR;
    }
#endif

#ifdef SPI_SR_OVR
    if (__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_OVR)) {
        __HAL_SPI_CLEAR_OVRFLAG(hspi);
    }
#endif
}

static inline void spi_post_dma_rx_fifo_sanity(SPI_HandleTypeDef *hspi) {
    // Mesmo que os indicadores de nível do FIFO reportem "vazio", algumas
    // rodadas DMA terminam com bytes residuais. Limpar incondicionalmente o
    // RX evita que esses resíduos contaminem o próximo frame.
    spi_rx_fifo_drain(hspi);
}
