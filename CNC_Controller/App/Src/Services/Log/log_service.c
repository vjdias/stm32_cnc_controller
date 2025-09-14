#include "Services/Log/log_service.h"
#if LOG_ENABLE

#include <string.h>
#include <stdio.h>
#include "usart.h"

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
static volatile uint8_t s_tx_busy = 0;
static uint8_t s_tx_buf[LOG_CHUNK_MAX];
static uint16_t s_tx_len = 0;

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
    s_tx_busy = 0;
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
    if(s_tx_busy) return; // wait for current IT transfer to complete
    uint16_t cnt = rb_count();
    if(!cnt) return;
    uint16_t n = (cnt > LOG_CHUNK_MAX) ? LOG_CHUNK_MAX : cnt;
    uint16_t first = (uint16_t)((s_head >= s_tail) ? (n) : (uint16_t)(LOG_BUF_SZ - s_tail));
    if(first > n) first = n;
    memcpy(s_tx_buf, &s_buf[s_tail], first);
    if(first < n){
        memcpy(s_tx_buf + first, &s_buf[0], n - first);
    }
    if(HAL_UART_Transmit_IT(&huart1, s_tx_buf, (uint16_t)n) == HAL_OK){
        s_tail = (uint16_t)((s_tail + n) % LOG_BUF_SZ);
        s_tx_len = n;
        s_tx_busy = 1;
    }
}

// Keep ISR work minimal; just clear busy so app can schedule next chunk.
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if(huart && huart->Instance == USART1){
        s_tx_busy = 0;
    }
}

#endif // LOG_ENABLE


