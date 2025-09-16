#include "Services/Log/log_service.h"
#if LOG_ENABLE

#include <string.h>
#include <stdio.h>
#include "main.h"

#if (LOG_BACKEND != LOG_BACKEND_USART1) && (LOG_BACKEND != LOG_BACKEND_SWO)
#error "Unsupported LOG_BACKEND value"
#endif

#if LOG_BACKEND == LOG_BACKEND_USART1
#include "usart.h"
#endif

#ifndef LOG_BUF_SZ
#define LOG_BUF_SZ 1024u
#endif
#ifndef LOG_CHUNK_MAX
#define LOG_CHUNK_MAX 96u
#endif

static volatile int s_enabled = 1;
static volatile log_mode_t s_mode = LOG_MODE_CONCISE;
static uint8_t s_buf[LOG_BUF_SZ];
static volatile uint16_t s_head = 0; // write index
static volatile uint16_t s_tail = 0; // read index

#if LOG_BACKEND == LOG_BACKEND_USART1
static volatile uint8_t s_tx_busy = 0;
static uint8_t s_tx_buf[LOG_CHUNK_MAX];
#endif

static inline uint16_t rb_count(void){
    uint16_t h = s_head, t = s_tail;
    return (uint16_t)((h >= t) ? (h - t) : (LOG_BUF_SZ - t + h));
}
static inline uint16_t rb_space(void){
    return (uint16_t)(LOG_BUF_SZ - 1 - rb_count());
}
static void rb_push_bytes(const uint8_t* data, uint16_t len){
    if(!data || !len || !s_enabled) return;
    uint16_t space = rb_space();
    if(len > space) len = space; // drop excess (lowest priority)
    for(uint16_t i=0;i<len;i++){
        s_buf[s_head] = data[i];
        s_head = (uint16_t)((s_head + 1) % LOG_BUF_SZ);
    }
}

#if LOG_BACKEND == LOG_BACKEND_SWO
#if !SWO_TRACE_ENABLE
#error "LOG_BACKEND_SWO requires SWO_TRACE_ENABLE=1"
#endif
static void swo_gpio_init(void) {
    /* PB3 is the SWO pin on STM32L475. Configure it for asynchronous trace output. */
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF0_TRACE;
    HAL_GPIO_Init(GPIOB, &gpio);
}
#else
static void swo_gpio_init(void) {}
#endif

static void push_line(const char* line){
    if(!line) return;
    size_t n = strlen(line);
    if(n > 240) n = 240; // trim
    rb_push_bytes((const uint8_t*)line, (uint16_t)n);
}

void log_service_init(void){
    s_enabled = LOG_DEFAULT_ENABLED;
    s_mode = LOG_DEFAULT_MODE;
    s_head = s_tail = 0;
#if LOG_BACKEND == LOG_BACKEND_USART1
    s_tx_busy = 0;
#elif LOG_BACKEND == LOG_BACKEND_SWO
    swo_gpio_init();

    /* Enable trace and configure SWO for ITM output */
    DBGMCU->CR |= DBGMCU_CR_TRACE_IOEN;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    ITM->LAR = 0xC5ACCE55;           /* Unlock ITM */
    TPI->SPPR = 2;                   /* NRZ/USART encoding */
    TPI->ACPR = (HAL_RCC_GetHCLKFreq() / 2000000U) - 1U; /* 2 MHz SWO */
    TPI->FFCR = 0x100U;              /* Formatter: flush on TPIU disable */
    ITM->TPR = 0U;                   /* All stimulus ports accessible */
    ITM->TCR = ITM_TCR_ITMENA_Msk | ITM_TCR_SWOENA_Msk; /* Enable ITM + SWO */
    ITM->TER = 1U;                   /* Enable stimulus port 0 */
#endif
}

void log_set_enabled(int enabled){ s_enabled = (enabled != 0); }
void log_set_mode(log_mode_t mode){ s_mode = mode; }

void log_event_ids(uint8_t service_id, uint8_t state_id, int32_t status){
    if(!s_enabled) return;
    if(s_mode != LOG_MODE_CONCISE) return;
    char line[64];
    // Format: L,svc=<id>,state=<id>,status=<num>\r\n
    int nn = snprintf(line, sizeof line, "L:svc=%u,state=%u,status=%ld\r\n",
                      (unsigned)service_id, (unsigned)state_id, (long)status);
    if(nn > 0) push_line(line);
}

void log_event_names(const char* service_name, const char* state_name, const char* status_text){
    if(!s_enabled) return;
    if(s_mode != LOG_MODE_VERBOSE) return;
    if(!service_name) service_name = "?";
    if(!state_name) state_name = "?";
    if(!status_text) status_text = "?";
    char line[160];
    // Format: LOG,service=<name>,state=<name>,status=<text>\r\n
    int nn = snprintf(line, sizeof line, "LOG:service=%s,state=%s,status=%s\r\n",
                      service_name, state_name, status_text);
    if(nn > 0) push_line(line);
}

void log_poll(void){
    if(!s_enabled) return;
    uint16_t cnt = rb_count();
    if(!cnt) return;
    uint16_t n = (cnt > LOG_CHUNK_MAX) ? LOG_CHUNK_MAX : cnt;

#if LOG_BACKEND == LOG_BACKEND_USART1
    if(s_tx_busy) return;

    uint16_t first = (uint16_t)((s_head >= s_tail) ? n : (LOG_BUF_SZ - s_tail));
    if(first > n) first = n;
    memcpy(s_tx_buf, &s_buf[s_tail], first);
    if(first < n){
        memcpy(s_tx_buf + first, &s_buf[0], n - first);
    }
    if(HAL_UART_Transmit_IT(&huart1, s_tx_buf, n) == HAL_OK){
        s_tail = (uint16_t)((s_tail + n) % LOG_BUF_SZ);
        s_tx_busy = 1;
    }
#elif LOG_BACKEND == LOG_BACKEND_SWO
    for(uint16_t i = 0; i < n; ++i){
        ITM_SendChar(s_buf[s_tail]);
        s_tail = (uint16_t)((s_tail + 1) % LOG_BUF_SZ);
    }
#endif
}

#if LOG_BACKEND == LOG_BACKEND_USART1
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart && huart->Instance == USART1){
        s_tx_busy = 0;
    }
}
#endif

int __io_putchar(int ch){
#if LOG_BACKEND == LOG_BACKEND_USART1
    uint8_t c = (uint8_t)ch;
    (void)HAL_UART_Transmit(&huart1, &c, 1u, HAL_MAX_DELAY);
    return ch;
#else
    ITM_SendChar((uint32_t)ch);
    return ch;
#endif
}

#endif // LOG_ENABLE


