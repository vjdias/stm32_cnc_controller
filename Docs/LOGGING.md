# Logging (USART1 / VCP)

- Gate de compilação: defina `LOG_ENABLE` para 1 (padrão) ou 0 (remove chamadas de log no build).
- Modos:
  - `LOG_MODE_CONCISE`: usa IDs numéricos via `log_event_ids(svc_id, state_id, status)` (baixo overhead).
  - `LOG_MODE_VERBOSE`: usa nomes/texto via `log_event_names("svc","state","status")` (legível por humanos).
- IDs canônicos (veja `CNC_Controller/App/Inc/Services/Log/log_service.h`):
  - Serviços: `LOG_SVC_APP=0`, `LED=1`, `MOTION=2`, `HOME=3`, `PROBE=4`, `SAFETY=5`.
  - Estados comuns: `LOG_STATE_START=0`, `RECEIVED=1`, `APPLIED=2`, `ESTOP_ASSERT=10`, `ESTOP_RELEASE=11`, `ERROR=100`.
- Padrão de uso nos serviços: logar em `*_init()` e em cada handler (recebido/aplicado/erro).
- Escoamento não-bloqueante: `log_poll()` é chamado em `app_poll()`; mantém TX por `HAL_UART_Transmit_IT()` (USART1).
- Boot: `app_init()` publica um log de inicialização.
