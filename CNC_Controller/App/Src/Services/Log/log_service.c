// Serviço de log simplificado: retarget de printf para SWO (ITM) quando ativo,
// com fallback para USART1. Assim, se o SWV estiver habilitado no debugger,
// as mensagens seguem via SWO; caso contrário, seguem para a UART.
#include "Services/Log/log_service.h"
#if LOG_ENABLE

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "usart.h"
#include "stm32l4xx.h"  // ITM_SendChar/CoreDebug/DBGMCU/TPI

void log_service_init(void){
    // Garante stdout sem buffer para que o printf descarregue imediatamente na UART.
    setvbuf(stdout, NULL, _IONBF, 0);
}

void log_poll(void){
    // No-op: a transmissão é síncrona via _write/HAL_UART_Transmit.
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

// Verifica em tempo de execução se o SWO/ITM está habilitado (porta 0).
static inline int log_swo_enabled(void)
{
    return ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) &&
            (DBGMCU->CR & DBGMCU_CR_TRACE_IOEN) &&
            (ITM->TCR & ITM_TCR_ITMENA_Msk) &&
            (ITM->TER & (1UL << 0)));
}

// Retarget de printf: usa SWO quando disponível; senão, USART1.
int _write(int fd, char *ptr, int len)
{
    if (fd != 1 && fd != 2)
        return -1;

    if (log_swo_enabled()) {
        for (int i = 0; i < len; ++i) {
            ITM_SendChar((uint32_t)ptr[i]);
        }
        return len;
    }

    // Fallback: UART1 síncrona
    if (HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, HAL_MAX_DELAY) == HAL_OK)
        return len;
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

