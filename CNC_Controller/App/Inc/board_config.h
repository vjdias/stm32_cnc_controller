#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_CONFIG_FLAG_PROX_X   (1u << 0)
#define BOARD_CONFIG_FLAG_PROX_Y   (1u << 1)
#define BOARD_CONFIG_FLAG_PROX_Z   (1u << 2)
#define BOARD_CONFIG_FLAG_ESTOP    (1u << 3)

void board_config_apply_motion_gpio(void);
void board_config_remap_tim3_encoder_pins(void);
void board_config_force_encoder_quadrature(void);
void board_config_apply_interrupt_priorities(void);
void board_config_apply_spi_dma_profile(void);

uint32_t board_config_get_safety_flags(void);
void board_config_clear_safety_flags(uint32_t mask);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
