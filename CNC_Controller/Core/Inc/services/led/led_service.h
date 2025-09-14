// LED service (controle simples via frame)
#pragma once
#include <stdint.h>

void led_service_init(void);
void led_on_led_ctrl(const uint8_t *frame, uint32_t len);

