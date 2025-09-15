// Non-interruptive SWO logging service
#pragma once

#include <stdint.h>
#include <stddef.h>

// Compile-time gate: set to 0 to strip logging at build time
#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

// Canonical service and state IDs for concise mode (always available)
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
    // Safety-specific
    LOG_STATE_ESTOP_ASSERT = 10,
    LOG_STATE_ESTOP_RELEASE = 11,
    // Generic error bucket (use with PROTO_ERR_*)
    LOG_STATE_ERROR = 100,
} log_state_id_t;

#if LOG_ENABLE

typedef enum {
    LOG_MODE_CONCISE = 0, // IDs + numeric status
    LOG_MODE_VERBOSE = 1  // Names + textual status
} log_mode_t;

// Defaults (can be overridden at compile time)
#ifndef LOG_DEFAULT_ENABLED
#define LOG_DEFAULT_ENABLED 1
#endif
#ifndef LOG_DEFAULT_MODE
#define LOG_DEFAULT_MODE LOG_MODE_VERBOSE
#endif

// Initialize internal buffers and state
void log_service_init(void);

// Runtime on/off and mode
void log_set_enabled(int enabled);
void log_set_mode(log_mode_t mode);

// Enqueue concise event: service/state IDs and numeric status (ok/warn/err)
void log_event_ids(uint8_t service_id, uint8_t state_id, int32_t status);

// Enqueue verbose event: names and textual status
void log_event_names(const char* service_name, const char* state_name, const char* status_text);

// Drain pending log bytes opportunistically (non-blocking)
void log_poll(void);

// Convenience: single entry point that emits concise or verbose based on current mode.
// - In concise mode: ignores names and formats IDs + status number.
// - In verbose mode: formats using printf-style fmt and args into status text.
void log_event_auto(log_service_id_t service_id, log_state_id_t state_id, int32_t status,
                    const char* service_name, const char* state_name,
                    const char* fmt, ...);

// Minimal macros to reduce call-site noise
#define LOG_SVC_DEFINE(id, name) \
    enum { LOG_SVC_THIS = (id) }; \
    static const char* const LOG_SVC_THIS_NAME = (name)

#define LOGA(svc_id, state_id, status, svc_name, state_name, ...) \
    do { log_event_auto((svc_id),(state_id),(status),(svc_name),(state_name), __VA_ARGS__); } while(0)

#define LOGA_THIS(state_id, status, state_name, ...) \
    do { LOGA(LOG_SVC_THIS,(state_id),(status),LOG_SVC_THIS_NAME,(state_name), __VA_ARGS__); } while(0)

// Text-only helpers (no % placeholders):
#define LOGT(svc_id, state_id, status, svc_name, state_name, text) \
    LOGA((svc_id),(state_id),(status),(svc_name),(state_name), "%s", (text))
#define LOGT_THIS(state_id, status, state_name, text) \
    LOGT(LOG_SVC_THIS,(state_id),(status),LOG_SVC_THIS_NAME,(state_name),(text))

#else // LOG_ENABLE == 0

static inline void log_service_init(void) {}
static inline void log_set_enabled(int enabled) { (void)enabled; }
typedef enum { LOG_MODE_CONCISE=0, LOG_MODE_VERBOSE=1 } log_mode_t;
static inline void log_set_mode(log_mode_t mode) { (void)mode; }
static inline void log_event_ids(uint8_t s, uint8_t t, int32_t st) { (void)s; (void)t; (void)st; }
static inline void log_event_names(const char* a, const char* b, const char* c) { (void)a; (void)b; (void)c; }
static inline void log_poll(void) {}
// When disabled, make macros no-ops so call-sites remain clean
#define LOG_SVC_DEFINE(id, name)
#define LOGA(svc_id, state_id, status, svc_name, state_name, ...)
#define LOGA_THIS(state_id, status, state_name, ...)
#define LOGT(svc_id, state_id, status, svc_name, state_name, text)
#define LOGT_THIS(state_id, status, state_name, text)

#endif
