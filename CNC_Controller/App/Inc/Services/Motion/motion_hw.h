// Camada de acesso ao hardware de movimento (STEP/DIR/ENA + encoders)
#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum { MOTION_AXIS_X = 0, MOTION_AXIS_Y = 1, MOTION_AXIS_Z = 2, MOTION_AXIS_COUNT = 3 };

// Inicializa IOs e encoders (mantém pinos em estado seguro e zera contadores)
void motion_hw_init(void);

// Controle de direção/enable (enable=1 liga driver, ativo em baixo)
void motion_hw_set_dir(uint8_t axis, uint8_t dir);
void motion_hw_enable(uint8_t axis, uint8_t enable);

// STEP: força nível alto/baixo rapidamente (sem HAL, via BSRR)
void motion_hw_step_high(uint8_t axis);
void motion_hw_step_low(uint8_t axis);

// Leitura de encoders (valor bruto)
uint32_t motion_hw_encoder_read_raw(uint8_t axis);
uint8_t motion_hw_encoder_bits(uint8_t axis);

#ifdef __cplusplus
}
#endif

