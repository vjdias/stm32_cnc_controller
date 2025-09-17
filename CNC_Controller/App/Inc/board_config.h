#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"

/**
 * @brief Applies the motion-controller specific GPIO configuration.
 *
 * This helper programs STEP/DIR/ENABLE outputs and arms the E-STOP/PROX inputs
 * with external interrupts after the CubeMX generated defaults are in place.
 */
void board_config_apply_motion_gpio(void);

/**
 * @brief Ensures encoder timers run in quadrature (TI12) mode.
 *
 * Call once after the CubeMX initialisation to upgrade TIM2/TIM3/TIM5 to the
 * X4 counting scheme required by the CNC encoders.
 */
void board_config_force_encoder_quadrature(void);

/**
 * @brief Remaps the Z axis encoder inputs to GPIOC pins.
 *
 * The firmware expects TIM3 channels on PC6/PC7 to match the wiring harness,
 * so this helper reconfigures the alternate function mapping accordingly.
 */
void board_config_remap_tim3_encoder_pins(void);

/**
 * @brief Applies the interrupt priority hierarchy defined for the CNC project.
 *
 * This sets NVIC levels for safety EXTI lines, motion timers, SPI DMA and the
 * diagnostic USART to honour the deterministic schedule documented in the README.
 */
void board_config_apply_interrupt_priorities(void);

/**
 * @brief Applies the SPI1 DMA priority/shape used by the CNC transport.
 *
 * Call once after the CubeMX initialisation to rebuild the RX channel with high
 * priority (circular) and the TX channel with normal priority (one-shot).
 */
void board_config_apply_spi_dma_profile(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
