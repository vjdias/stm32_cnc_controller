// Serviço de logging via SWO sem interrupções
#pragma once

#include <stdint.h>
#include <stddef.h>

// Chave de compilação: ajuste para 0 para remover o logging na construção
#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

// IDs canônicos de serviço e estado para o modo conciso (sempre disponíveis)
typedef enum {
    LOG_SVC_APP = 0,
    LOG_SVC_LED = 1,
    LOG_SVC_MOTION = 2,
    LOG_SVC_HOME = 3,
    LOG_SVC_PROBE = 4,
    LOG_SVC_SAFETY = 5,
} log_service_id_t;

typedef enum {
    LOG_STATE_START = 0,
    LOG_STATE_RECEIVED = 1,
    LOG_STATE_APPLIED = 2,
    // Eventos específicos de segurança
    LOG_STATE_ESTOP_ASSERT = 10,
    LOG_STATE_ESTOP_RELEASE = 11,
    // Balde genérico de erro (usar com PROTO_ERR_*)
    LOG_STATE_ERROR = 100,
} log_state_id_t;

#if LOG_ENABLE

// Inicializa o logging (coloca stdout sem buffer)
void log_service_init(void);

// Empilha evento conciso: IDs de serviço/estado e status numérico (ok/warn/err)
void log_event_ids(uint8_t service_id, uint8_t state_id, int32_t status);

// Empilha evento verboso: nomes e status textual
void log_event_names(const char* service_name, const char* state_name, const char* status_text);

// Drena bytes pendentes do log de forma oportunista (não bloqueante)
void log_poll(void);

// Conveniência: ponto único que formata status com printf e argumentos variáveis.
void log_event_auto(log_service_id_t service_id, log_state_id_t state_id, int32_t status,
                    const char* service_name, const char* state_name,
                    const char* fmt, ...);

// Macros mínimas para reduzir ruído nos pontos de chamada
#define LOG_SVC_DEFINE(id, name) \
    enum { LOG_SVC_THIS = (id) }; \
    static const char* const LOG_SVC_THIS_NAME = (name)

#define LOGA(svc_id, state_id, status, svc_name, state_name, ...) \
    do { log_event_auto((svc_id),(state_id),(status),(svc_name),(state_name), __VA_ARGS__); } while(0)

#define LOGA_THIS(state_id, status, state_name, ...) \
    do { LOGA(LOG_SVC_THIS,(state_id),(status),LOG_SVC_THIS_NAME,(state_name), __VA_ARGS__); } while(0)

// Auxiliares apenas com texto (sem marcadores %):
#define LOGT(svc_id, state_id, status, svc_name, state_name, text) \
    LOGA((svc_id),(state_id),(status),(svc_name),(state_name), "%s", (text))
#define LOGT_THIS(state_id, status, state_name, text) \
    LOGT(LOG_SVC_THIS,(state_id),(status),LOG_SVC_THIS_NAME,(state_name),(text))

#else // LOG_ENABLE == 0

static inline void log_service_init(void) {}
static inline void log_event_ids(uint8_t s, uint8_t t, int32_t st) { (void)s; (void)t; (void)st; }
static inline void log_event_names(const char* a, const char* b, const char* c) { (void)a; (void)b; (void)c; }
static inline void log_poll(void) {}
// Quando desativado, deixa as macros como no-ops para manter os call-sites limpos
#define LOG_SVC_DEFINE(id, name)
#define LOGA(svc_id, state_id, status, svc_name, state_name, ...)
#define LOGA_THIS(state_id, status, state_name, ...)
#define LOGT(svc_id, state_id, status, svc_name, state_name, text)
#define LOGT_THIS(state_id, status, state_name, text)

#endif
