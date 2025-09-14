// Non-interruptive USB (USART1 VCP) logging service
#pragma once

#include <stdint.h>
#include <stddef.h>

// Compile-time gate: set to 0 to strip logging at build time
#ifndef LOG_ENABLE
#define LOG_ENABLE 1
#endif

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

#else // LOG_ENABLE == 0

static inline void log_service_init(void) {}
static inline void log_set_enabled(int enabled) { (void)enabled; }
typedef enum { LOG_MODE_CONCISE=0, LOG_MODE_VERBOSE=1 } log_mode_t;
static inline void log_set_mode(log_mode_t mode) { (void)mode; }
static inline void log_event_ids(uint8_t s, uint8_t t, int32_t st) { (void)s; (void)t; (void)st; }
static inline void log_event_names(const char* a, const char* b, const char* c) { (void)a; (void)b; (void)c; }
static inline void log_poll(void) {}

#endif
