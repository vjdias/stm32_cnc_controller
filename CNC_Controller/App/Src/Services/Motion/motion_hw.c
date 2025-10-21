#include "Services/Motion/motion_hw.h"
#include "gpio.h"
#include "tim.h"
#include "lptim.h"
#include "main.h"

// Mapeamento de pinos e timers por eixo
typedef enum {
    MOTION_ENCODER_TYPE_TIM = 0,
    MOTION_ENCODER_TYPE_LPTIM = 1,
} motion_encoder_type_t;

typedef struct {
    GPIO_TypeDef *step_port; uint16_t step_pin;
    GPIO_TypeDef *dir_port;  uint16_t dir_pin;
    GPIO_TypeDef *ena_port;  uint16_t ena_pin;
    motion_encoder_type_t encoder_type;
    TIM_HandleTypeDef *tim;
    LPTIM_HandleTypeDef *lptim;
    uint8_t counter_bits; // 16 ou 32
} motion_axis_hw_t;

#define LPTIM_ENCODER_PERIOD 0xFFFFu

static const motion_axis_hw_t g_axis[MOTION_AXIS_COUNT] = {
    // X -> TIM3 em PA6/PA7 (16 bits)
    { GPIOB, GPIO_PIN_4, GPIOA, GPIO_PIN_3, GPIOC, GPIO_PIN_4,
      MOTION_ENCODER_TYPE_TIM, &htim3, NULL, 16u },
    // Y -> LPTIM1 em PA4/PA5 (16 bits)
    { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_2, GPIOC, GPIO_PIN_5,
      MOTION_ENCODER_TYPE_LPTIM, NULL, &hlptim1, 16u },
    // Z -> TIM5 em PA0/PA1 (32 bits)
    { GPIOB, GPIO_PIN_1, GPIOA, GPIO_PIN_2, GPIOA, GPIO_PIN_8,
      MOTION_ENCODER_TYPE_TIM, &htim5, NULL, 32u },
};

static inline void gpio_bsrr_set(GPIO_TypeDef *port, uint16_t pin)
{
    if (!port) return;
    port->BSRR = pin;
}
static inline void gpio_bsrr_reset(GPIO_TypeDef *port, uint16_t pin)
{
    if (!port) return;
    port->BSRR = ((uint32_t)pin) << 16u;
}

void motion_hw_init(void)
{
    // Garante STEP baixo e drivers desabilitados
    for (uint8_t i = 0; i < MOTION_AXIS_COUNT; ++i) {
        gpio_bsrr_reset(g_axis[i].step_port, g_axis[i].step_pin);
        // Enable ativo em baixo: mantém alto (desabilitado)
        gpio_bsrr_set(g_axis[i].ena_port, g_axis[i].ena_pin);
    }

    // Zera contadores e inicia encoders conforme o tipo de periférico
    for (uint8_t i = 0; i < MOTION_AXIS_COUNT; ++i) {
        const motion_axis_hw_t *axis = &g_axis[i];
        if (axis->encoder_type == MOTION_ENCODER_TYPE_TIM) {
            __HAL_TIM_SET_COUNTER(axis->tim, 0u);
            if (HAL_TIM_Encoder_Start(axis->tim, TIM_CHANNEL_ALL) != HAL_OK) {
                Error_Handler();
            }
        } else if (axis->encoder_type == MOTION_ENCODER_TYPE_LPTIM) {
            (void)HAL_LPTIM_Encoder_Stop(axis->lptim);
            if (HAL_LPTIM_Encoder_Start(axis->lptim, LPTIM_ENCODER_PERIOD) != HAL_OK) {
                Error_Handler();
            }
            __HAL_LPTIM_RESET_COUNTER(axis->lptim);
        }
    }
}

void motion_hw_set_dir(uint8_t axis, uint8_t dir)
{
    if (axis >= MOTION_AXIS_COUNT) return;
    if (dir) gpio_bsrr_set(g_axis[axis].dir_port, g_axis[axis].dir_pin);
    else     gpio_bsrr_reset(g_axis[axis].dir_port, g_axis[axis].dir_pin);
}

void motion_hw_enable(uint8_t axis, uint8_t enable)
{
    if (axis >= MOTION_AXIS_COUNT) return;
    // Enable ativo em baixo: enable=1 -> força baixo
    if (enable) gpio_bsrr_reset(g_axis[axis].ena_port, g_axis[axis].ena_pin);
    else        gpio_bsrr_set(g_axis[axis].ena_port, g_axis[axis].ena_pin);
}

void motion_hw_step_high(uint8_t axis)
{
    if (axis >= MOTION_AXIS_COUNT) return;
    gpio_bsrr_set(g_axis[axis].step_port, g_axis[axis].step_pin);
}
void motion_hw_step_low(uint8_t axis)
{
    if (axis >= MOTION_AXIS_COUNT) return;
    gpio_bsrr_reset(g_axis[axis].step_port, g_axis[axis].step_pin);
}

uint32_t motion_hw_encoder_read_raw(uint8_t axis)
{
    if (axis >= MOTION_AXIS_COUNT) return 0;
    const motion_axis_hw_t *hw = &g_axis[axis];
    if (hw->encoder_type == MOTION_ENCODER_TYPE_TIM) {
        return (uint32_t)__HAL_TIM_GET_COUNTER(hw->tim);
    } else {
        return (uint32_t)(hw->lptim->Instance->CNT & 0xFFFFu);
    }
}

uint8_t motion_hw_encoder_bits(uint8_t axis)
{
    if (axis >= MOTION_AXIS_COUNT) return 0u;
    return g_axis[axis].counter_bits;
}

