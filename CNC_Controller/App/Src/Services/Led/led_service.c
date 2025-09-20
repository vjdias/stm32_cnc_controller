#include "Services/Led/led_service.h"
#include "gpio.h"
#include "tim.h"
#include "Protocol/Requests/led_control_request.h"
#include "Protocol/Responses/led_control_response.h"
#include "app.h"
#include "Services/Log/log_service.h"
#include <stdio.h>

LOG_SVC_DEFINE(LOG_SVC_LED, "led");

#if LED_CTRL_CHANNEL_COUNT != 1u
#error "LED service expects exactly one LED channel"
#endif

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint8_t mode;
    uint8_t is_on;
    uint16_t frequency_hz;
    uint32_t half_period_ticks;
    uint32_t ticks_until_toggle;
} led_channel_state_t;

static led_channel_state_t g_leds[LED_CTRL_CHANNEL_COUNT] = {
    { LED1_GPIO_PORT, LED1_GPIO_PIN, LED_MODE_OFF, 0u, 0u, 0u, 0u },
};

static void led_push_response(uint8_t frame_id, uint8_t mask, uint8_t status) {
    uint8_t raw[7];
    led_ctrl_resp_t resp = { frame_id, mask, status };
    if (led_ctrl_resp_encoder(&resp, raw, sizeof raw) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_FRAME, "resp", "failed to encode led ack");
        return;
    }
    if (app_resp_push(raw, (uint32_t)sizeof raw) != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "resp", "failed to queue led ack");
    }
}

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

static uint32_t led_compute_half_period_ticks(uint16_t freq_hz) {
    if (!freq_hz)
        return 0u;
    uint32_t half_period = 500u / (uint32_t)freq_hz;
    if (half_period == 0u)
        half_period = 1u; // limita à resolução de 1 ms do temporizador dedicado

    return half_period;
}

static void led_apply_config(led_channel_state_t *led, uint8_t mode, uint16_t freq_hz) {
    if (!led)
        return;
    if (mode > LED_MODE_BLINK)
        mode = LED_MODE_OFF;

    uint32_t half_period = (mode == LED_MODE_BLINK) ? led_compute_half_period_ticks(freq_hz) : 0u;
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (mode == LED_MODE_ON) {
        led->mode = LED_MODE_ON;
        led->frequency_hz = 0u;
        led->half_period_ticks = 0u;
        led->ticks_until_toggle = 0u;
        led_drive(led, 1u);
    } else if (mode == LED_MODE_BLINK && half_period > 0u) {
        led->mode = LED_MODE_BLINK;
        led->frequency_hz = freq_hz;
        led->half_period_ticks = half_period;
        led->ticks_until_toggle = half_period;
        led_drive(led, 1u);
    } else {
        led->mode = LED_MODE_OFF;
        led->frequency_hz = 0u;
        led->half_period_ticks = 0u;
        led->ticks_until_toggle = 0u;
        led_drive(led, 0u);
    }

    if (primask == 0u) {
        __enable_irq();
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
        g_leds[i].half_period_ticks = 0u;
        g_leds[i].ticks_until_toggle = 0u;
        led_drive(&g_leds[i], 0u);
    }

    if (htim15.Instance != TIM15) {
        MX_TIM15_Init();
    }
    if (HAL_TIM_Base_Start_IT(&htim15) != HAL_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "timer", "falha ao iniciar TIM15");

    }
}

static void led_service_on_tick(void) {
    for (uint32_t i = 0; i < LED_CTRL_CHANNEL_COUNT; ++i) {
        led_channel_state_t *led = &g_leds[i];
        if (led->mode == LED_MODE_BLINK && led->half_period_ticks > 0u) {
            if (led->ticks_until_toggle > 0u) {
                --led->ticks_until_toggle;
            }
            if (led->ticks_until_toggle == 0u) {
                led->ticks_until_toggle = led->half_period_ticks;
                led_drive(led, led->is_on ? 0u : 1u);
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (!htim)
        return;
    if (htim == &htim15) {
        led_service_on_tick();
    }
}

void led_on_led_ctrl(const uint8_t *frame, uint32_t len) {
    led_ctrl_req_t req;
    if (!frame)
        return;
    proto_result_t decode_status = led_ctrl_req_decoder(frame, len, &req);
    if (decode_status != PROTO_OK) {
        LOGA_THIS(LOG_STATE_ERROR, decode_status, "decode", "failed to decode led request (%d)", (int)decode_status);
        return;
    }

    const uint8_t requested_mask = req.ledMask;
    const uint8_t valid_mask = LED_MASK_LED1;
    uint8_t ack_mask = 0u;
    uint8_t status = PROTO_OK;

    for (uint32_t i = 0; i < LED_CTRL_CHANNEL_COUNT; ++i) {
        uint8_t mask_bit = LED_MASK_LED1;
        if ((requested_mask & mask_bit) == 0u) {
            continue;
        }
        ack_mask |= mask_bit;
        led_apply_config(&g_leds[i], req.channel[i].mode, req.channel[i].frequency);
    }

    if ((requested_mask & (uint8_t)~valid_mask) != 0u) {
        status = PROTO_WARN;
    } else if (ack_mask == 0u && requested_mask != 0u) {
        status = PROTO_WARN;
    }

    led_push_response(req.frameId, ack_mask, status);

    LOGA_THIS(LOG_STATE_APPLIED, status, "applied",
              "reqMask=0x%02X ackMask=0x%02X LED1(mode=%u,f=%uHz,on=%u)",
              (unsigned)requested_mask, (unsigned)ack_mask,
              g_leds[0].mode, g_leds[0].frequency_hz, g_leds[0].is_on);
}
