// Serviço de LED (controle simples via frame)
#pragma once
#include <stdint.h>

// Mapeamento de GPIO para o LED discreto presente no controlador.
// Os padrões seguem a placa B-L475E-IOT01A. Conforme a UM2153 (tabela de
// pinagem do kit), o LED do conector Arduino ("LED1") divide o pino PA5 com o
// sinal SCK do SPI1. Para evitar que a inicialização do serviço reconfigure o
// barramento SPI, adotamos o LED discreto LD2 soldado na placa, conectado ao
// PB14. Sobrescreva via definições de compilação ao direcionar para outra
// placa ou ligação.
#ifndef LED1_GPIO_PORT
#define LED1_GPIO_PORT GPIOB
#endif
#ifndef LED1_GPIO_PIN
#define LED1_GPIO_PIN  GPIO_PIN_14
#endif

// Função alternativa utilizada pelo PWM do TIM15.
#ifndef LED1_GPIO_AF
#define LED1_GPIO_AF GPIO_AF14_TIM15
#endif

// Nível lógico que acende o LED.
#ifndef LED_ACTIVE_HIGH
#define LED_ACTIVE_HIGH 1
#endif

/**
 * @brief Inicializa o serviço de LED configurando os GPIOs e garantindo que os
 *        canais iniciem apagados.
 */
void led_service_init(void);

/**
 * @brief Manipula um frame REQ_LED_CTRL recebido via protocolo.
 *
 * @param frame Ponteiro para o buffer bruto contendo o frame completo,
 *              começando pelo byte de header.
 * @param len   Comprimento, em bytes, do frame apontado por @p frame.
 */
void led_on_led_ctrl(const uint8_t *frame, uint32_t len);

/**
 * @brief Mantido por compatibilidade: realiza tarefas pendentes do serviço.
 *
 * A temporização passou a ser feita por interrupção de hardware (TIM15),
 * portanto esta função não possui mais trabalho periódico. Ela permanece
 * disponível apenas para builds que ainda a invoquem.
 */
void led_service_poll(void);
