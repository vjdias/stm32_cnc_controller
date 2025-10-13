#include "Services/Motion/motion_hw.h"
#include "gpio.h"
#include "tim.h"
#include "main.h"

// Mapeamento de pinos e timers por eixo
typedef struct {
    GPIO_TypeDef *step_port; uint16_t step_pin;
    GPIO_TypeDef *dir_port;  uint16_t dir_pin;
    GPIO_TypeDef *ena_port;  uint16_t ena_pin;
    TIM_HandleTypeDef *encoder;
    uint8_t counter_bits; // 16 ou 32
} motion_axis_hw_t;

static const motion_axis_hw_t g_axis[MOTION_AXIS_COUNT] = {
    // X
    { GPIOB, GPIO_PIN_4, GPIOA, GPIO_PIN_3, GPIOC, GPIO_PIN_4, &htim2, 32u },
    // Y
    { GPIOB, GPIO_PIN_0, GPIOB, GPIO_PIN_2, GPIOC, GPIO_PIN_5, &htim5, 32u },
    // Z
    { GPIOB, GPIO_PIN_1, GPIOA, GPIO_PIN_2, GPIOA, GPIO_PIN_8, &htim3, 16u },
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

    // Zera contadores e inicia encoders
    __HAL_TIM_SET_COUNTER(g_axis[MOTION_AXIS_X].encoder, 0u);
    __HAL_TIM_SET_COUNTER(g_axis[MOTION_AXIS_Y].encoder, 0u);
    __HAL_TIM_SET_COUNTER(g_axis[MOTION_AXIS_Z].encoder, 0u);

    (void)HAL_TIM_Encoder_Start(g_axis[MOTION_AXIS_X].encoder, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(g_axis[MOTION_AXIS_Y].encoder, TIM_CHANNEL_ALL);
    (void)HAL_TIM_Encoder_Start(g_axis[MOTION_AXIS_Z].encoder, TIM_CHANNEL_ALL);
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
    if (g_axis[axis].counter_bits == 16u) {
        return (uint32_t)(__HAL_TIM_GET_COUNTER(g_axis[axis].encoder) & 0xFFFFu);
    } else {
        return (uint32_t)__HAL_TIM_GET_COUNTER(g_axis[axis].encoder);
    }
}

