#include "Services/Log/log_service.h"
#if LOG_ENABLE

#include <string.h>
#include <stdio.h>
#include "stm32l4xx_hal.h" // for ITM_SendChar

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
    for(uint16_t i = 0; i < n; ++i){
        ITM_SendChar(s_buf[s_tail]);
        s_tail = (uint16_t)((s_tail + 1) % LOG_BUF_SZ);
    }
}

int __io_putchar(int ch){
    ITM_SendChar((uint32_t)ch);
    return ch;
}

#endif // LOG_ENABLE


