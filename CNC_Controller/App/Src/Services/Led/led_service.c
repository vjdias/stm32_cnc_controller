#include "Services/Led/led_service.h"
#include "gpio.h"
#include "Protocol/Requests/led_control_request.h"
#include "Services/Log/log_service.h"
#include <stdio.h>

LOG_SVC_DEFINE(LOG_SVC_LED, "led");

void led_service_init(void) {
    GPIO_InitTypeDef gi = {0};
#if defined(LED_R_GPIO_PIN) && defined(LED_G_GPIO_PIN) && defined(LED_B_GPIO_PIN)
    // Configure RGB pins
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Pin = LED_R_GPIO_PIN; HAL_GPIO_Init(LED_R_GPIO_PORT, &gi);
    gi.Pin = LED_G_GPIO_PIN; HAL_GPIO_Init(LED_G_GPIO_PORT, &gi);
    gi.Pin = LED_B_GPIO_PIN; HAL_GPIO_Init(LED_B_GPIO_PORT, &gi);
    // Default OFF
#if LED_ACTIVE_HIGH
    HAL_GPIO_WritePin(LED_R_GPIO_PORT, LED_R_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_GPIO_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(LED_R_GPIO_PORT, LED_R_GPIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_GPIO_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_GPIO_PIN, GPIO_PIN_SET);
#endif
#else
    // Fallback: single LED
    gi.Pin = LED_GPIO_PIN;
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &gi);
#if LED_ACTIVE_HIGH
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, GPIO_PIN_SET);
#endif
#endif
}

static inline void led_apply_mono(uint8_t on) {
#if LED_ACTIVE_HIGH
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

#if defined(LED_R_GPIO_PIN) && defined(LED_G_GPIO_PIN) && defined(LED_B_GPIO_PIN)
static inline void led_apply_rgb(uint8_t r, uint8_t g, uint8_t b, uint8_t mask) {
    // Treat non-zero as ON (binary per channel). For PWM, integrate TIM later.
    if (mask & LED_MASK_R) {
#if LED_ACTIVE_HIGH
        HAL_GPIO_WritePin(LED_R_GPIO_PORT, LED_R_GPIO_PIN, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
        HAL_GPIO_WritePin(LED_R_GPIO_PORT, LED_R_GPIO_PIN, r ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
    }
    if (mask & LED_MASK_G) {
#if LED_ACTIVE_HIGH
        HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_GPIO_PIN, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
        HAL_GPIO_WritePin(LED_G_GPIO_PORT, LED_G_GPIO_PIN, g ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
    }
    if (mask & LED_MASK_B) {
#if LED_ACTIVE_HIGH
        HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_GPIO_PIN, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
#else
        HAL_GPIO_WritePin(LED_B_GPIO_PORT, LED_B_GPIO_PIN, b ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
    }
}
#endif

void led_on_led_ctrl(const uint8_t *frame, uint32_t len) {
    led_ctrl_req_t req;
    if (!frame)
        return;
    if (led_ctrl_req_decoder(frame, len, &req) != PROTO_OK)
        return;
#if defined(LED_R_GPIO_PIN) && defined(LED_G_GPIO_PIN) && defined(LED_B_GPIO_PIN)
    led_apply_rgb(req.r, req.g, req.b, req.ledMask);
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "applied", "mask=0x%02X rgb=%u,%u,%u", (unsigned)req.ledMask, req.r, req.g, req.b);
#else
    // Use green component as ON/OFF for mono LED when RGB not wired
    if (req.ledMask & (LED_MASK_R | LED_MASK_G | LED_MASK_B))
        led_apply_mono((req.r | req.g | req.b) ? 1u : 0u);
    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "applied", "%s", ((req.r | req.g | req.b) ? "on" : "off"));
#endif
}
