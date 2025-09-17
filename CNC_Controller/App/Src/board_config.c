#include "board_config.h"

#include "dma.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "tim.h"

static volatile uint32_t g_safety_flags;

static void configure_encoder_quadrature(TIM_HandleTypeDef *htim);

void board_config_apply_motion_gpio(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* STEP/DIR signals (push-pull, very high speed, default low) */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Enable outputs default high (drivers disabled until firmware arms them) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_4 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOC, &gpio);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    gpio.Pin = GPIO_PIN_8;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* Safety inputs (E-STOP / PROX) */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    gpio.Mode = GPIO_MODE_IT_RISING_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);

    gpio.Pin = GPIO_PIN_13;
    gpio.Mode = GPIO_MODE_IT_FALLING;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &gpio);
}

void board_config_remap_tim3_encoder_pins(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOE_CLK_ENABLE();
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_3 | GPIO_PIN_4);

    __HAL_RCC_GPIOC_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &gpio);
}

void board_config_force_encoder_quadrature(void)
{
    configure_encoder_quadrature(&htim2);
    configure_encoder_quadrature(&htim5);
    configure_encoder_quadrature(&htim3);
}

void board_config_apply_interrupt_priorities(void)
{
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
    HAL_NVIC_SetPriority(SPI1_IRQn, 2, 1);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
    HAL_NVIC_SetPriority(TIM7_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

void board_config_apply_spi_dma_profile(void)
{
    if (hdma_spi1_rx.Instance != NULL)
    {
        MODIFY_REG(hdma_spi1_rx.Instance->CCR, DMA_CCR_PL, DMA_PRIORITY_HIGH);
    }
    if (hdma_spi1_tx.Instance != NULL)
    {
        CLEAR_BIT(hdma_spi1_tx.Instance->CCR, DMA_CCR_CIRC);
        MODIFY_REG(hdma_spi1_tx.Instance->CCR, DMA_CCR_PL, DMA_PRIORITY_LOW);
    }
}

uint32_t board_config_get_safety_flags(void)
{
    return g_safety_flags;
}

void board_config_clear_safety_flags(uint32_t mask)
{
    g_safety_flags &= ~mask;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin)
    {
    case GPIO_PIN_0:
        g_safety_flags |= BOARD_CONFIG_FLAG_PROX_X;
        break;
    case GPIO_PIN_1:
        g_safety_flags |= BOARD_CONFIG_FLAG_PROX_Y;
        break;
    case GPIO_PIN_2:
        g_safety_flags |= BOARD_CONFIG_FLAG_PROX_Z;
        break;
    case GPIO_PIN_13:
        g_safety_flags |= BOARD_CONFIG_FLAG_ESTOP;
        break;
    default:
        break;
    }
}

static void configure_encoder_quadrature(TIM_HandleTypeDef *htim)
{
    TIM_Encoder_InitTypeDef cfg = {0};

    cfg.EncoderMode = TIM_ENCODERMODE_TI12;
    cfg.IC1Polarity = TIM_ICPOLARITY_RISING;
    cfg.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    cfg.IC1Prescaler = TIM_ICPSC_DIV1;
    cfg.IC1Filter = 0;
    cfg.IC2Polarity = TIM_ICPOLARITY_RISING;
    cfg.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    cfg.IC2Prescaler = TIM_ICPSC_DIV1;
    cfg.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(htim, &cfg) != HAL_OK)
    {
        Error_Handler();
    }
}
