#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/**
 * @brief Aplica o mapeamento de GPIO definido para o controlador CNC.
 *
 * Este helper recompõe todos os pinos sensíveis após a inicialização gerada
 * pelo CubeMX: configura STEP/DIR/ENABLE com *push-pull* em alta velocidade,
 * grava o nível seguro de repouso para cada saída e arma as entradas de
 * segurança (E-STOP e sensores de proximidade) com interrupção externa.
 * Deve ser chamado logo após `MX_GPIO_Init()` para garantir que nenhum pino
 * permaneça com direção ou *pull* incorretos enquanto os drivers são energizados.
 */
void board_config_apply_motion_gpio(void);

/**
 * @brief Força os *timers* de encoder a operar em modo de quadratura TI12 (X4).
 *
 * CubeMX gera os contadores no modo TI1 por padrão, o que perderia metade dos
 * flancos. Esta rotina reconfigura TIM2/TIM3/TIM5 para capturar ambos os
 * canais do encoder e habilita a contagem em quatro vezes a resolução.
 */
void board_config_force_encoder_quadrature(void);

/**
 * @brief Remapeia os sinais do encoder do eixo Z para os pinos PC6/PC7.
 *
 * O chicote do controlador leva TIM3_CH1/CH2 até o conector em GPIOC, mas o
 * CubeMX ainda prende os canais a PE3/PE4. Esta função libera os pinos
 * originais e refaz a configuração de *alternate function* em PC6/PC7.
 */
void board_config_remap_tim3_encoder_pins(void);

/**
 * @brief Programa a hierarquia de prioridades de interrupção usada pelo CNC.
 *
 * Define os níveis do NVIC (Nested Vectored Interrupt Controller) para que as
 * rotinas críticas respeitem a ordem descrita no README: EXTI de segurança no
 * topo, seguido do laço de passos (TIM6), DMA de SPI, laço de controle (TIM7)
 * e, por último, os periféricos de diagnóstico como a USART.
 */
void board_config_apply_interrupt_priorities(void);

/**
 * @brief Ajusta o perfil de DMA do SPI2 para o transporte mestre-escravo.
 *
 * Reconstrói os canais gerados pelo CubeMX para que a recepção opere em modo
 * circular com prioridade alta (evitando *overrun* de comandos) e a transmissão
 * use prioridade normal em modo *one-shot*, alinhada ao escoamento assíncrono
 * do `router` do aplicativo.
 */
//void board_config_apply_spi_dma_profile(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
