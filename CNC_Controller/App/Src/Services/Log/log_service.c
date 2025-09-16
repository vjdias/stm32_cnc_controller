// Simplified log service: route everything to printf (USART1 via _write)
#include "Services/Log/log_service.h"
#if LOG_ENABLE

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "usart.h"

void log_service_init(void){
    // Ensure stdout is unbuffered so printf flushes immediately to UART.
    setvbuf(stdout, NULL, _IONBF, 0);
}

void log_poll(void){
    // No-op: transmission is synchronous via _write/HAL_UART_Transmit.
}

void log_event_ids(uint8_t service_id, uint8_t state_id, int32_t status){
    printf("L:svc=%u,state=%u,status=%ld\r\n", (unsigned)service_id, (unsigned)state_id, (long)status);
}

void log_event_names(const char* service_name, const char* state_name, const char* status_text){
    if(!service_name) service_name = "?";
    if(!state_name) state_name = "?";
    if(!status_text) status_text = "?";
    printf("LOG:service=%s,state=%s,status=%s\r\n", service_name, state_name, status_text);
}

// Keep _write exactly as-is: used by printf to send to USART1.
int _write(int fd, char *ptr, int len) {
    HAL_StatusTypeDef hstatus;

    if (fd == 1 || fd == 2) {
      hstatus = HAL_UART_Transmit(&huart1, (uint8_t *) ptr, len, HAL_MAX_DELAY);
      if (hstatus == HAL_OK)
        return len;
      else
        return -1;
    }
    return -1;
}

void log_event_auto(log_service_id_t service_id, log_state_id_t state_id, int32_t status,
                    const char* service_name, const char* state_name,
                    const char* fmt, ...){
    (void)service_id;
    (void)state_id;
    (void)status;

    char text[128];
    if(fmt && fmt[0]){
        va_list ap;
        va_start(ap, fmt);
        (void)vsnprintf(text, sizeof text, fmt, ap);
        va_end(ap);
    }else{
        text[0] = '?';
        text[1] = '\0';
    }
    const char* svc = service_name ? service_name : "?";
    const char* stn = state_name ? state_name : "?";
    printf("LOG:service=%s,state=%s,status=%s\r\n", svc, stn, text);
}

#endif // LOG_ENABLE

