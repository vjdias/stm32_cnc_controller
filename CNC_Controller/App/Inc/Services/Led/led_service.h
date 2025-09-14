// LED service (controle simples via frame)
#pragma once
#include <stdint.h>

// GPIO mapping for on-board user LED(s).
// Defaults assume B-L475E-IOT01A user LEDs, active high.
// Override via compile definitions if needed for other boards.

// RGB channels (default to PB1/PB14/PB7)
#ifndef LED_R_GPIO_PORT
#define LED_R_GPIO_PORT GPIOB
#endif
#ifndef LED_R_GPIO_PIN
#define LED_R_GPIO_PIN  GPIO_PIN_1
#endif
#ifndef LED_G_GPIO_PORT
#define LED_G_GPIO_PORT GPIOB
#endif
#ifndef LED_G_GPIO_PIN
#define LED_G_GPIO_PIN  GPIO_PIN_14
#endif
#ifndef LED_B_GPIO_PORT
#define LED_B_GPIO_PORT GPIOB
#endif
#ifndef LED_B_GPIO_PIN
#define LED_B_GPIO_PIN  GPIO_PIN_7
#endif

// Fallback single LED (if RGB not defined/used)
#ifndef LED_GPIO_PORT
#define LED_GPIO_PORT GPIOB
#endif
#ifndef LED_GPIO_PIN
#define LED_GPIO_PIN  GPIO_PIN_14
#endif
#ifndef LED_ACTIVE_HIGH
#define LED_ACTIVE_HIGH 1
#endif

void led_service_init(void);
void led_on_led_ctrl(const uint8_t *frame, uint32_t len);
