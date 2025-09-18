#include "board_config.h"

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "spi.h"
#include "usart.h"

/*
 * Este módulo concentra ajustes que não podem ser descritos diretamente no
 * CubeMX: modos específicos dos timers de encoder, remapeamento de pinos para
 * casar com o chicote do maquinário e prioridades de interrupção pensadas para
 * preservar o determinismo do controlador CNC. As rotinas aqui devem ser
 * chamadas logo após as funções `MX_*_Init()` geradas pelo código-base.
 */

/**
 * @brief Reconfigura um timer de encoder para quadratura TI12.
 *
 * A estrutura gerada pelo CubeMX usa TI1 por padrão. Esta função sobrescreve a
 * configuração para capturar os dois canais do encoder, mantendo todos os
 * filtros e *prescalers* em 0/1 para preservar a resolução máxima.
 */
static void configure_encoder_timer(TIM_HandleTypeDef *htim)
{
    TIM_Encoder_InitTypeDef config = {0};
    TIM_MasterConfigTypeDef master = {0};

    config.EncoderMode = TIM_ENCODERMODE_TI12;
    config.IC1Polarity = TIM_ICPOLARITY_RISING;
    config.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC1Prescaler = TIM_ICPSC_DIV1;
    config.IC1Filter = 0;
    config.IC2Polarity = TIM_ICPOLARITY_RISING;
    config.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    config.IC2Prescaler = TIM_ICPSC_DIV1;
    config.IC2Filter = 0;

    if (HAL_TIM_Encoder_Init(htim, &config) != HAL_OK)
    {
        Error_Handler();
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(htim, &master) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief Ajusta um conjunto de saídas STEP/DIR/ENABLE para modo *push-pull*.
 */
static void configure_output(GPIO_TypeDef *port, uint32_t pins, uint32_t speed)
{
    GPIO_InitTypeDef init = {0};
    init.Pin = pins;
    init.Mode = GPIO_MODE_OUTPUT_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = speed;
    HAL_GPIO_Init(port, &init);
}

void board_config_apply_motion_gpio(void)
{
    GPIO_InitTypeDef init = {0};

    /* Saídas de movimento com tempos de borda compatíveis com STEP/DIR */
    configure_output(GPIOB, GPIO_PIN_4 | GPIO_PIN_0 | GPIO_PIN_1, GPIO_SPEED_FREQ_VERY_HIGH);
    configure_output(GPIOB, GPIO_PIN_2, GPIO_SPEED_FREQ_VERY_HIGH);
    configure_output(GPIOA, GPIO_PIN_3 | GPIO_PIN_2, GPIO_SPEED_FREQ_VERY_HIGH);
    configure_output(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_SPEED_FREQ_LOW);
    configure_output(GPIOA, GPIO_PIN_8, GPIO_SPEED_FREQ_LOW);

    /* Estados seguros antes de habilitar drivers: ENA alto, STEP/DIR baixos */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_2 | GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

    /* Entradas de segurança em *pull-up* com detecção de bordas de ambos os sentidos */
    init.Mode = GPIO_MODE_IT_RISING_FALLING;
    init.Pull = GPIO_PULLUP;

    init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
    HAL_GPIO_Init(GPIOC, &init);

    init.Pin = GPIO_PIN_13;
    HAL_GPIO_Init(GPIOC, &init);
}

void board_config_force_encoder_quadrature(void)
{
    configure_encoder_timer(&htim2);
    configure_encoder_timer(&htim3);
    configure_encoder_timer(&htim5);
}

void board_config_remap_tim3_encoder_pins(void)
{
    GPIO_InitTypeDef init = {0};

    /* Libera a configuração padrão do CubeMX e migra o encoder para PC6/PC7 */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_3 | GPIO_PIN_4);

    __HAL_RCC_GPIOC_CLK_ENABLE();

    init.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    init.Mode = GPIO_MODE_AF_PP;
    init.Pull = GPIO_NOPULL;
    init.Speed = GPIO_SPEED_FREQ_LOW;
    init.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &init);
}

void board_config_apply_interrupt_priorities(void)
{
    /* EXTI de segurança: interrupções mais altas para abortar movimento */
    HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);

    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

    /* Temporização do núcleo de movimento (TIM6/TIM7) e transporte SPI */
    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

    HAL_NVIC_SetPriority(TIM7_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);

    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
}

void board_config_apply_spi_dma_profile(void)
{
    if (HAL_DMA_DeInit(&hdma_spi1_rx) != HAL_OK)
    {
        Error_Handler();
    }
    /* RX circular e prioritário: evita perda de comandos do mestre SPI */
    hdma_spi1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    hdma_spi1_rx.Init.Mode = DMA_CIRCULAR;
    if (HAL_DMA_Init(&hdma_spi1_rx) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(&hspi1, hdmarx, hdma_spi1_rx);

    if (HAL_DMA_DeInit(&hdma_spi1_tx) != HAL_OK)
    {
        Error_Handler();
    }
    /* TX em prioridade normal, disparado apenas quando há resposta no FIFO */
    hdma_spi1_tx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_spi1_tx.Init.Mode = DMA_NORMAL;
    if (HAL_DMA_Init(&hdma_spi1_tx) != HAL_OK)
    {
        Error_Handler();
    }
    __HAL_LINKDMA(&hspi1, hdmatx, hdma_spi1_tx);
}
