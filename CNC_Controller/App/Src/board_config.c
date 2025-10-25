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
//
//static void configure_spi_dma(DMA_HandleTypeDef *handle,
//                              DMA_Channel_TypeDef *instance,
//                              uint32_t request,
//                              uint32_t direction,
//                              uint32_t mode,
//                              uint32_t priority)
//{
//    if (!handle || !instance)
//    {
//        Error_Handler();
//    }
//
//    handle->Instance = instance;
//
//    if (HAL_DMA_DeInit(handle) != HAL_OK)
//    {
//        Error_Handler();
//    }
//
//    handle->Init.Request = request;
//    handle->Init.Direction = direction;
//    handle->Init.PeriphInc = DMA_PINC_DISABLE;
//    handle->Init.MemInc = DMA_MINC_ENABLE;
//    handle->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
//    handle->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
//    handle->Init.Mode = mode;
//    handle->Init.Priority = priority;
//
//    if (HAL_DMA_Init(handle) != HAL_OK)
//    {
//        Error_Handler();
//    }
//}

void board_config_apply_motion_gpio(void)
{
    GPIO_InitTypeDef init = {0};

    /* Saídas de movimento com tempos de borda compatíveis com STEP/DIR */
    configure_output(GPIOB, GPIO_PIN_4 | GPIO_PIN_0 | GPIO_PIN_1, GPIO_SPEED_FREQ_VERY_HIGH);
    configure_output(GPIOB, GPIO_PIN_2, GPIO_SPEED_FREQ_VERY_HIGH);
    configure_output(GPIOA, GPIO_PIN_3 | GPIO_PIN_2, GPIO_SPEED_FREQ_VERY_HIGH);
    configure_output(GPIOC, GPIO_PIN_4 | GPIO_PIN_5, GPIO_SPEED_FREQ_LOW);
    configure_output(GPIOD, GPIO_PIN_14, GPIO_SPEED_FREQ_LOW);

    /* Estados seguros antes de habilitar drivers: ENA alto, STEP/DIR baixos */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_2 | GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3 | GPIO_PIN_2, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);     /* EN_X desabilitado (alto) */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);   /* EN_Y habilitado (baixo) */
    // EN_Z (PD14) ativo em nível baixo por solicitação: inicia habilitado
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

    /* Entradas de segurança em *pull-up* com detecção de bordas de ambos os sentidos */
    init.Mode = GPIO_MODE_IT_RISING_FALLING;
    init.Pull = GPIO_PULLUP;

    /* IMPORTANTE: PC0 e PC2 são usados pelo encoder Y (LPTIM1_IN1/IN2).
     * Reconfigurá-los como EXTI com PULLUP quebra:
     *  - o Alternate Function do LPTIM1 (deixa de contar corretamente);
     *  - a tolerância a 5 V dos pinos do tipo FT_a (puxadores internos ativados).
     *
     * Para eliminar a interferência no encoder, comentamos a configuração de EXTI
     * para PC0/PC2. Mantemos apenas PC1 (se necessário) e PC13.
     *
     * Para voltar ao comportamento anterior, remapeie o encoder para outros pinos
     * ou troque as entradas de EXTI; depois, remova o bloco #if 0 abaixo.
     */
#if 0
    init.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;  /* PC0/PC1/PC2 como EXTI */
    HAL_GPIO_Init(GPIOC, &init);
#else
    init.Pin = GPIO_PIN_1;                             /* Somente PC1 como EXTI */
    HAL_GPIO_Init(GPIOC, &init);
#endif

    init.Pin = GPIO_PIN_13;                            /* EXTI do PC13 permanece */
    HAL_GPIO_Init(GPIOC, &init);
}

void board_config_force_encoder_quadrature(void)
{
    configure_encoder_timer(&htim3);
    configure_encoder_timer(&htim5);
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

    HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

    HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

    HAL_NVIC_SetPriority(TIM7_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);

    HAL_NVIC_SetPriority(USART1_IRQn, 4, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    HAL_NVIC_SetPriority(SPI2_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI2_IRQn);

    HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
}

//'hdma_spi2_rx' undeclared (first use in this function)	board_config.c	/CNC_Controller/App/Src	line 192	C/C++ Problem
//void board_config_apply_spi_dma_profile(void)
//{
//    /* RX em modo normal: cada quadro AA..55 + handshakes byte a byte ocupa um slot */
//    configure_spi_dma(&hdma_spi2_rx,
//                      DMA1_Channel4,
//                      DMA_REQUEST_SPI2_RX,
//                      DMA_PERIPH_TO_MEMORY,
//                      DMA_CIRCULAR,
//                      DMA_PRIORITY_HIGH);
//    __HAL_LINKDMA(&hspi2, hdmarx, hdma_spi2_rx);
//
//    /* TX também em modo normal; buffers são preenchidos a cada transação */
//    configure_spi_dma(&hdma_spi2_tx,
//                      DMA1_Channel5,
//                      DMA_REQUEST_SPI2_TX,
//                      DMA_MEMORY_TO_PERIPH,
//					  DMA_CIRCULAR,
//                      DMA_PRIORITY_LOW);
//    __HAL_LINKDMA(&hspi2, hdmatx, hdma_spi2_tx);
//}
