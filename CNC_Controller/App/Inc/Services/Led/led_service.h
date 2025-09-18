// Serviço de LED (controle simples via frame)
#pragma once
#include <stdint.h>

// Mapeamento de GPIO para os dois LEDs discretos presentes no controlador.
// Os padrões seguem a placa B-L475E-IOT01A, onde o LED1 (verde) está no PA5
// e o LED2 (verde) está no PB14. Sobrescreva via definições de compilação
// ao direcionar para outra placa ou ligação.
#ifndef LED1_GPIO_PORT
#define LED1_GPIO_PORT GPIOA
#endif
#ifndef LED1_GPIO_PIN
#define LED1_GPIO_PIN  GPIO_PIN_5
#endif
#ifndef LED2_GPIO_PORT
#define LED2_GPIO_PORT GPIOB
#endif
#ifndef LED2_GPIO_PIN
#define LED2_GPIO_PIN  GPIO_PIN_14
#endif

// Nível lógico que acende o LED (compartilhado pelos dois LEDs).
#ifndef LED_ACTIVE_HIGH
#define LED_ACTIVE_HIGH 1
#endif

void led_service_init(void);
void led_service_poll(void);
void led_on_led_ctrl(const uint8_t *frame, uint32_t len);
