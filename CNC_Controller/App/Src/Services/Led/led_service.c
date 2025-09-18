#include "Services/Led/led_service.h"
#include "gpio.h"
#include "Protocol/Requests/led_control_request.h"
#include "Services/Log/log_service.h"
#include <stdio.h>

LOG_SVC_DEFINE(LOG_SVC_LED, "led");

#if LED_CTRL_CHANNEL_COUNT != 2u
#error "LED service expects exactly two LED channels"
#endif

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t mode;
    uint8_t is_on;
    uint16_t frequency_hz;
    uint32_t half_period_ms;
    uint32_t next_toggle_ms;
} led_channel_state_t;

static led_channel_state_t g_leds[LED_CTRL_CHANNEL_COUNT] = {
    { LED1_GPIO_PORT, LED1_GPIO_PIN, LED_MODE_OFF, 0u, 0u, 0u, 0u },
    { LED2_GPIO_PORT, LED2_GPIO_PIN, LED_MODE_OFF, 0u, 0u, 0u, 0u },
};

static const char *const g_led_names[LED_CTRL_CHANNEL_COUNT] = { "LED1", "LED2" };

static void led_drive(led_channel_state_t *led, uint8_t on) {
    if (!led)
        return;
    GPIO_PinState level;
#if LED_ACTIVE_HIGH
    level = on ? GPIO_PIN_SET : GPIO_PIN_RESET;
#else
    level = on ? GPIO_PIN_RESET : GPIO_PIN_SET;
#endif
    HAL_GPIO_WritePin(led->port, led->pin, level);
    led->is_on = on ? 1u : 0u;
}

static uint32_t led_compute_half_period_ms(uint16_t freq_hz) {
    if (!freq_hz)
        return 0u;
    uint32_t half_period = 500u / (uint32_t)freq_hz;
    if (half_period == 0u)
        half_period = 1u; // limita à resolução do SysTick (1 ms)
    return half_period;
}

static void led_apply_config(led_channel_state_t *led, uint8_t mode, uint16_t freq_hz) {
    if (!led)
        return;
    if (mode > LED_MODE_BLINK)
        mode = LED_MODE_OFF;

    led->half_period_ms = 0u;
    led->next_toggle_ms = 0u;

    if (mode == LED_MODE_ON) {
        led->mode = LED_MODE_ON;
        led->frequency_hz = 0u;
        led_drive(led, 1u);
    } else if (mode == LED_MODE_BLINK) {
        uint32_t now = HAL_GetTick();
        uint32_t half_period = led_compute_half_period_ms(freq_hz);
        if (half_period == 0u) {
            led->mode = LED_MODE_OFF;
            led->frequency_hz = 0u;
            led_drive(led, 0u);
        } else {
            led->mode = LED_MODE_BLINK;
            led->frequency_hz = freq_hz;
            led->half_period_ms = half_period;
            led_drive(led, 1u);
            led->next_toggle_ms = now + half_period;
        }
    } else {
        led->mode = LED_MODE_OFF;
        led->frequency_hz = 0u;
        led_drive(led, 0u);
    }
}

void led_service_init(void) {
    GPIO_InitTypeDef gi = {0};
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;

    for (uint32_t i = 0; i < LED_CTRL_CHANNEL_COUNT; ++i) {
        gi.Pin = g_leds[i].pin;
        HAL_GPIO_Init(g_leds[i].port, &gi);
        g_leds[i].mode = LED_MODE_OFF;
        g_leds[i].frequency_hz = 0u;
        g_leds[i].half_period_ms = 0u;
        g_leds[i].next_toggle_ms = 0u;
        led_drive(&g_leds[i], 0u);
    }
}

void led_service_poll(void) {
    uint32_t now = HAL_GetTick();
    for (uint32_t i = 0; i < LED_CTRL_CHANNEL_COUNT; ++i) {
        led_channel_state_t *led = &g_leds[i];
        if (led->mode == LED_MODE_BLINK && led->half_period_ms > 0u) {
            if ((int32_t)(now - led->next_toggle_ms) >= 0) {
                led->next_toggle_ms = now + led->half_period_ms;
                led_drive(led, led->is_on ? 0u : 1u);
            }
        }
    }
}

void led_on_led_ctrl(const uint8_t *frame, uint32_t len) {
    led_ctrl_req_t req;
    if (!frame)
        return;
    if (led_ctrl_req_decoder(frame, len, &req) != PROTO_OK)
        return;

    for (uint32_t i = 0; i < LED_CTRL_CHANNEL_COUNT; ++i) {
        uint8_t mask_bit = (i == 0u) ? LED_MASK_LED1 : LED_MASK_LED2;
        if (req.ledMask & mask_bit) {
            led_apply_config(&g_leds[i], req.channel[i].mode, req.channel[i].frequency);
        }
    }

    LOGA_THIS(LOG_STATE_APPLIED, PROTO_OK, "applied",
              "mask=0x%02X %s(mode=%u,f=%uHz,on=%u) %s(mode=%u,f=%uHz,on=%u)",
              (unsigned)req.ledMask,
              g_led_names[0], g_leds[0].mode, g_leds[0].frequency_hz, g_leds[0].is_on,
              g_led_names[1], g_leds[1].mode, g_leds[1].frequency_hz, g_leds[1].is_on);
}
