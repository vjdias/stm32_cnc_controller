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
} led_channel_state_t;

static led_channel_state_t g_leds[LED_CTRL_CHANNEL_COUNT] = {
    { LED1_GPIO_PORT, LED1_GPIO_PIN, LED_MODE_OFF, 0u, 0u },
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

static uint32_t led_timer_get_clock(void) {
    uint32_t clk = HAL_RCC_GetPCLK2Freq();
#if defined(RCC_CFGR_PPRE2) && defined(RCC_CFGR_PPRE2_DIV1)
    uint32_t presc = (RCC->CFGR & RCC_CFGR_PPRE2);
    if (presc != RCC_CFGR_PPRE2_DIV1 && presc != 0u) {
        clk *= 2u;
    }
#endif
    return clk;
}

static uint32_t led_compute_period_ticks(uint16_t freq_hz) {
    if (!freq_hz)
        return 0u;
    uint32_t timer_clk = led_timer_get_clock();
    uint32_t prescaler = (uint32_t)htim15.Init.Prescaler + 1u;
    if (prescaler == 0u)
        return 0u;
    uint32_t ticks = timer_clk / (prescaler * (uint32_t)freq_hz);
    return ticks;
}

static void led_apply_pwm(uint32_t period_ticks, uint32_t pulse_ticks) {
    if (period_ticks == 0u)
        period_ticks = 1u;
    if (pulse_ticks > period_ticks)
        pulse_ticks = period_ticks;

    uint32_t arr = (period_ticks > 0u) ? (period_ticks - 1u) : 0u;
    __HAL_TIM_SET_AUTORELOAD(&htim15, arr);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pulse_ticks);
    HAL_TIM_GenerateEvent(&htim15, TIM_EVENTSOURCE_UPDATE);
    htim15.Init.Period = arr;
}

static void led_force_off(led_channel_state_t *led) {
    if (!led)
        return;
    led_apply_pwm((uint32_t)htim15.Init.Period + 1u, 0u);
    led->mode = LED_MODE_OFF;
    led->frequency_hz = 0u;
    led->is_on = 0u;
}

static void led_force_on(led_channel_state_t *led) {
    if (!led)
        return;
    uint32_t period = (uint32_t)htim15.Init.Period + 1u;
    if (period == 0u)
        period = 10u;
    led_apply_pwm(period, period);
    led->mode = LED_MODE_ON;
    led->frequency_hz = 0u;
    led->is_on = 1u;
}

static void led_force_blink(led_channel_state_t *led, uint16_t freq_hz) {
    if (!led || freq_hz == 0u)
        return;
    uint32_t period_ticks = led_compute_period_ticks(freq_hz);
    if (period_ticks < 2u)
        period_ticks = 2u;
    if (period_ticks > (uint32_t)0x10000u)
        period_ticks = 0x10000u;

    uint32_t pulse_ticks = period_ticks / 2u;
    led_apply_pwm(period_ticks, pulse_ticks);
    led->mode = LED_MODE_BLINK;
    led->frequency_hz = freq_hz;
    led->is_on = 0u;
}

static void led_apply_config(led_channel_state_t *led, uint8_t mode, uint16_t freq_hz) {
    if (!led)
        return;

    if (mode > LED_MODE_BLINK)
        mode = LED_MODE_OFF;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (mode == LED_MODE_ON) {
        led_force_on(led);
    } else if (mode == LED_MODE_BLINK && freq_hz > 0u) {
        led_force_blink(led, freq_hz);
    } else {
        led_force_off(led);
    }

    if (primask == 0u) {
        __enable_irq();
    }
}

static HAL_StatusTypeDef led_pwm_start(void) {
    HAL_StatusTypeDef st = HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
#if defined(TIM_CHANNEL_1N)
    if (st == HAL_OK) {
        st = HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
    }
#endif
    return st;
}

void led_service_init(void) {
    GPIO_InitTypeDef gi = {0};
    gi.Mode = GPIO_MODE_AF_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Alternate = GPIO_AF14_TIM15;

    for (uint32_t i = 0; i < LED_CTRL_CHANNEL_COUNT; ++i) {
        gi.Pin = g_leds[i].pin;
        HAL_GPIO_Init(g_leds[i].port, &gi);
        g_leds[i].mode = LED_MODE_OFF;
        g_leds[i].frequency_hz = 0u;
        g_leds[i].is_on = 0u;
    }

    if (htim15.Instance != TIM15) {
        MX_TIM15_Init();
    }

    if (HAL_TIM_PWM_Init(&htim15) != HAL_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "timer", "falha ao inicializar PWM do TIM15");
        return;
    }

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode = TIM_OCMODE_PWM1;
#if LED_ACTIVE_HIGH
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
#else
    oc.OCPolarity = TIM_OCPOLARITY_LOW;
    oc.OCNPolarity = TIM_OCNPOLARITY_LOW;
#endif
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    oc.Pulse = 0u;

    if (HAL_TIM_PWM_ConfigChannel(&htim15, &oc, TIM_CHANNEL_1) != HAL_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "timer", "falha ao configurar canal PWM do TIM15");
        return;
    }

    if (led_pwm_start() != HAL_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "timer", "falha ao iniciar PWM do TIM15");
        return;
    }

    led_force_off(&g_leds[0]);
}

void led_on_led_ctrl(const uint8_t *frame, uint32_t len) {
    led_ctrl_req_t req;
    if (!frame)
        return;
    if (len < LED_CTRL_REQ_TOTAL_LEN || len > LED_CTRL_REQ_PADDED_TOTAL_LEN) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "len", "invalid led frame len=%lu", (unsigned long)len);
        return;
    }
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
              "reqMask=0x%02X ackMask=0x%02X LED1(mode=%u,f=%uHz,on=%u,ARR=%lu,CCR=%lu)",
              (unsigned)requested_mask, (unsigned)ack_mask,
              g_leds[0].mode, g_leds[0].frequency_hz, g_leds[0].is_on,
              (unsigned long)(__HAL_TIM_GET_AUTORELOAD(&htim15) + 1u),
              (unsigned long)__HAL_TIM_GET_COMPARE(&htim15, TIM_CHANNEL_1));
}
