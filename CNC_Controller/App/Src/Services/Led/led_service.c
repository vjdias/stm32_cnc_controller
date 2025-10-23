#include "Services/Led/led_service.h"
#include "gpio.h"
#include "tim.h"
#include "Protocol/Requests/led_control_request.h"
#include "Protocol/Responses/led_control_response.h"
#include "app.h"
#include "Services/Log/log_service.h"
#include <stdio.h>
#include <stdint.h>

LOG_SVC_DEFINE(LOG_SVC_LED, "led");

#if LED_CTRL_CHANNEL_COUNT != 1u
#error "LED service expects exactly one LED channel"
#endif

typedef struct {
    GPIO_TypeDef *port;
    uint16_t pin;
    uint32_t alternate;
    uint8_t mode;
    uint8_t is_on;
    uint16_t frequency_centi_hz;
} led_channel_state_t;

static led_channel_state_t g_leds[LED_CTRL_CHANNEL_COUNT] = {
    { LED1_GPIO_PORT, LED1_GPIO_PIN, LED1_GPIO_AF, LED_MODE_OFF, 0u, 0u },
};

static uint8_t g_pwm_running = 0u;

#if LED_ACTIVE_HIGH
#define LED_GPIO_ON_LEVEL  GPIO_PIN_SET
#define LED_GPIO_OFF_LEVEL GPIO_PIN_RESET
#else
#define LED_GPIO_ON_LEVEL  GPIO_PIN_RESET
#define LED_GPIO_OFF_LEVEL GPIO_PIN_SET
#endif

static void led_gpio_config_output(const led_channel_state_t *led) {
    if (!led)
        return;
    GPIO_InitTypeDef gi = {0};
    gi.Pin = led->pin;
    gi.Mode = GPIO_MODE_OUTPUT_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(led->port, &gi);
}

static void led_gpio_config_pwm(const led_channel_state_t *led) {
    if (!led)
        return;
    GPIO_InitTypeDef gi = {0};
    gi.Pin = led->pin;
    gi.Mode = GPIO_MODE_AF_PP;
    gi.Pull = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Alternate = led->alternate;
    HAL_GPIO_Init(led->port, &gi);
}

static HAL_StatusTypeDef led_pwm_start(void);
static HAL_StatusTypeDef led_pwm_stop(void);

/*
 * Resumo: monta e publica um quadro de resposta LED copiando-o para o buffer
 *         de saída global, registrando falhas de codificação ou enfileiramento
 *         caso qualquer etapa da rotina não suceda.
 * Parâmetros:
 *  - frame_id: identifica o comando original que está sendo respondido, para
 *              que o host consiga correlacionar requisição e ACK.
 *  - mask: bitmap de canais cujo estado foi alterado e que precisa ser
 *          refletido no payload gerado pelo encoder do protocolo.
 *  - status: código de resultado informado ao host (por exemplo, sucesso ou
 *            erro de validação).
 * Variáveis locais:
 *  - raw: buffer automático de 7 bytes que recebe o quadro codificado antes de
 *         ser enfileirado na FIFO de respostas.
 *  - resp: estrutura tipada preenchida com os campos acima, repassada ao
 *          encoder de respostas.
 * Validações internas:
 *  - Verifica se o encoder concluiu com PROTO_OK antes de tentar publicar a
 *    resposta; em caso negativo, registra o erro e abandona o envio para evitar
 *    inserir dados inválidos na fila.
 *  - Após a codificação, confere o resultado de app_resp_push para sinalizar e
 *    logar falhas na fila de saída (por exemplo, quando estiver cheia).
 */
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

/*
 * Converte a frequência de pisca (em centi-hertz) no período correspondente
 * para o TIM15, levando em conta o prescaler configurado.
 *
 * freq_centi_hz carrega a frequência já multiplicada por 100. Para obter o
 * período (ticks) precisamos multiplicar o clock efetivo do timer por 100 e
 * dividir novamente por freq_centi_hz (isto é, clk_per_second / (freq/100)).
 * A soma de freq_centi_hz/2 implementa arredondamento para o inteiro mais
 * próximo, mantendo o cálculo puramente inteiro.
 *
 * Exemplos com o clock atual (80 MHz) e prescaler = 0 (divisor efetivo 1):
 *   - 1,00 Hz → freq_centi_hz = 100 → ticks calculados = 80 000 000.
 *   - 0,20 Hz → freq_centi_hz =  20 → ticks calculados = 400 000 000.
 * Ambos excedem os 65 536 passos que o ARR de 16 bits suporta, portanto
 * led_force_blink() satura o período em 0x10000 e o LED pisca de fato a
 * ~1,22 kHz (80 MHz / 65 536). Para atingir frequências como 1 Hz ou 0,2 Hz
 * é necessário reduzir o clock efetivo do TIM15 via prescaler (por exemplo,
 * PSC = 7999 → divisor efetivo 8 000 → f_min ≈ 0,15 Hz).
 */

static uint32_t led_compute_period_ticks(uint16_t freq_centi_hz) {
    if (freq_centi_hz == 0u)
        return 0u;

    uint32_t timer_clk = led_timer_get_clock();
    uint32_t prescaler = (uint32_t)htim15.Init.Prescaler + 1u;
    if (prescaler == 0u)
        return 0u;

    uint32_t clk_per_second = timer_clk / prescaler;
    if (clk_per_second == 0u)
        return 0u;

    uint64_t scaled_clock = (uint64_t)clk_per_second * 100u;
    uint64_t ticks = (scaled_clock + ((uint64_t)freq_centi_hz / 2u)) / (uint64_t)freq_centi_hz;
    if (ticks > 0xFFFFFFFFu)
        ticks = 0xFFFFFFFFu;
    return (uint32_t)ticks;
}

static void led_apply_pwm(uint32_t period_ticks, uint32_t pulse_ticks) {
    if (period_ticks == 0u)
        period_ticks = 1u;
    if (pulse_ticks > period_ticks)
        pulse_ticks = period_ticks;

    uint32_t arr = (period_ticks > 0u) ? (period_ticks - 1u) : 0u;
    __HAL_TIM_SET_AUTORELOAD(&htim15, arr);
    __HAL_TIM_SET_COMPARE(&htim15, TIM_CHANNEL_1, pulse_ticks);
    __HAL_TIM_SET_COUNTER(&htim15, 0);
    HAL_TIM_GenerateEvent(&htim15, TIM_EVENTSOURCE_UPDATE);
    htim15.Init.Period = arr;
}

static void led_force_off(led_channel_state_t *led) {
    if (!led)
        return;
    HAL_StatusTypeDef st = led_pwm_stop();
    if (st != HAL_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "timer", "falha ao parar PWM do TIM15 (%d)", (int)st);
    }
    led_gpio_config_output(led);
    HAL_GPIO_WritePin(led->port, led->pin, LED_GPIO_OFF_LEVEL);
    led->mode = LED_MODE_OFF;
    led->frequency_centi_hz = 0u;
    led->is_on = 0u;
}

static void led_force_on(led_channel_state_t *led) {
    if (!led)
        return;
    HAL_StatusTypeDef st = led_pwm_stop();
    if (st != HAL_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "timer", "falha ao parar PWM do TIM15 (%d)", (int)st);
    }
    led_gpio_config_output(led);
    HAL_GPIO_WritePin(led->port, led->pin, LED_GPIO_ON_LEVEL);
    led->mode = LED_MODE_ON;
    led->frequency_centi_hz = 0u;
    led->is_on = 1u;
}

static void led_force_blink(led_channel_state_t *led, uint16_t freq_centi_hz) {
    if (!led || freq_centi_hz == 0u)
        return;
    uint32_t period_ticks = led_compute_period_ticks(freq_centi_hz);
    if (period_ticks < 2u)
        period_ticks = 2u;
    if (period_ticks > (uint32_t)0x10000u)
        period_ticks = 0x10000u;

    uint32_t pulse_ticks = period_ticks / 2u;
    led_gpio_config_pwm(led);
    led_apply_pwm(period_ticks, pulse_ticks);
    if (led_pwm_start() != HAL_OK) {
        LOGA_THIS(LOG_STATE_ERROR, PROTO_ERR_RANGE, "timer", "falha ao iniciar PWM do TIM15");
        led_force_off(led);
        return;
    }
    led->mode = LED_MODE_BLINK;
    led->frequency_centi_hz = freq_centi_hz;
    led->is_on = 0u;
}

static void led_apply_config(led_channel_state_t *led, uint8_t mode, uint16_t freq_centi_hz) {
    if (!led)
        return;

    if (mode > LED_MODE_BLINK)
        mode = LED_MODE_OFF;

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (mode == LED_MODE_ON) {
        led_force_on(led);
    } else if (mode == LED_MODE_BLINK && freq_centi_hz > 0u) {
        led_force_blink(led, freq_centi_hz);
    } else {
        led_force_off(led);
    }

    if (primask == 0u) {
        __enable_irq();
    }
}

static HAL_StatusTypeDef led_pwm_start(void) {
    if (g_pwm_running)
        return HAL_OK;

    HAL_StatusTypeDef st = HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
#if defined(TIM_CHANNEL_1N)
    if (st == HAL_OK) {
        st = HAL_TIMEx_PWMN_Start(&htim15, TIM_CHANNEL_1);
    }
#endif
    if (st == HAL_OK) {
        g_pwm_running = 1u;
    }
    return st;
}

static HAL_StatusTypeDef led_pwm_stop(void) {
    if (!g_pwm_running)
        return HAL_OK;

    HAL_StatusTypeDef st = HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
#if defined(TIM_CHANNEL_1N)
    if (st == HAL_OK) {
        st = HAL_TIMEx_PWMN_Stop(&htim15, TIM_CHANNEL_1);
    }
#endif
    if (st == HAL_OK) {
        __HAL_TIM_DISABLE(&htim15);
        g_pwm_running = 0u;
    }
    return st;
}

void led_service_init(void) {
    g_pwm_running = 0u;

    for (uint32_t i = 0; i < LED_CTRL_CHANNEL_COUNT; ++i) {
        led_gpio_config_output(&g_leds[i]);
        HAL_GPIO_WritePin(g_leds[i].port, g_leds[i].pin, LED_GPIO_OFF_LEVEL);
        g_leds[i].mode = LED_MODE_OFF;
        g_leds[i].frequency_centi_hz = 0u;
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
              "reqMask=0x%02X ackMask=0x%02X LED1(mode=%u,f=%lu.%02luHz,on=%u,ARR=%lu,CCR=%lu)",
              (unsigned)requested_mask, (unsigned)ack_mask,
              g_leds[0].mode,
              (unsigned long)(g_leds[0].frequency_centi_hz / 100u),
              (unsigned long)(g_leds[0].frequency_centi_hz % 100u),
              g_leds[0].is_on,
              (unsigned long)(__HAL_TIM_GET_AUTORELOAD(&htim15) + 1u),
              (unsigned long)__HAL_TIM_GET_COMPARE(&htim15, TIM_CHANNEL_1));
}
