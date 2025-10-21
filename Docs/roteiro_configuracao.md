# Roteiro de Configuração e Testes do Controlador CNC

Este roteiro segue o fluxo sugerido do mais crítico ao menos crítico. Utilize as checklists para acompanhar o progresso de configuração e validação de cada etapa.

## 1. Fundação de clock, pinos e prioridades de NVIC (Nested Vectored Interrupt Controller)
- [x] Ajustar o clock global para 80 MHz no CubeMX e regenerar o código.
- [x] Confirmar os prescalers dos barramentos APB1/APB2 conforme o plano do projeto.
- [x] Aplicar o mapeamento de pinos previsto para encoders, sinais STEP/DIR/EN, SPI (Serial Peripheral Interface) e USART (Universal Synchronous/Asynchronous Receiver/Transmitter).
- [x] Definir as prioridades do NVIC seguindo a hierarquia: EXTI (External Interrupt) → TIM6 → SPI2 DMA (Direct Memory Access) → TIM7 → USART1.
- [x] Teste: compilar e, via debugger/leitura de registradores, confirmar `SystemCoreClock = 80 MHz` e verificar as prioridades no vetor NVIC.

## 2. Interrupções de segurança (E-STOP/PROX via EXTI)
- [ ] Configurar os GPIOs (General Purpose Input/Output) de E-STOP e proximidade como entradas com EXTI.
- [ ] Na ISR (Interrupt Service Routine), apenas sinalizar flags de parada/segurança sem lógica pesada.
- [ ] Garantir que as interrupções estejam atribuídas à prioridade mais alta do NVIC.
- [ ] Teste: com apenas os GPIOs ativos, acionar manualmente cada entrada (pull-up/pull-down ou jumper) e observar no debugger/console se as flags disparam imediatamente.

## 3. TIM6 – Gerador de passos (DDA a 50 kHz)
- [ ] Configurar o TIM6 como base de tempo com `PSC = 79` e `ARR = 19`.
- [ ] Habilitar a interrupção do TIM6 e configurar os sinais STEP/DIR/EN nas portas correspondentes.
- [ ] Implementar na ISR a geração de pulsos de teste para validação inicial.
- [ ] Teste: medir com osciloscópio/analisador lógico e confirmar frequência de 50 kHz com duty mínimo de 1 µs.

## 4. TIM2/TIM5/TIM3 – Encoders em modo TI1&TI2 (Quadratura X4)
- [ ] Ajustar cada timer para `TIM_ENCODERMODE_TI12`, com `PSC = 0` e `ARR` máximo (32 bits para X/Y, 16 bits para Z).
- [ ] Verificar o mapeamento de pinos dos encoders conforme o chicote instalado.
- [ ] Iniciar os timers em modo encoder usando `HAL_TIM_Encoder_Start`.
- [ ] Teste: girar os encoders (ou injetar sinais quadratura) e monitorar os contadores no debugger, garantindo contagem correta em ambos os sentidos.

## 5. TIM7 – Loop de controle e serviços a 1 kHz
- [ ] Configurar o TIM7 para 1 kHz com `PSC = 7999` e `ARR = 9`.
- [ ] Implementar na ISR o esqueleto do controle: leitura incremental dos encoders e atualização dos controladores PI/PID (Proportional-Integral/Proportional-Integral-Derivative).
- [ ] Agendar serviços auxiliares necessários (diagnósticos, filtros, etc.) respeitando o orçamento de tempo do laço.
- [ ] Teste: instrumentar a ISR e medir jitter/tempo de execução para garantir que o loop mantém 1 kHz sem interferir na janela do TIM6.

## 6. SPI2 escravo com DMA circular (Raspberry Pi ↔ STM32)
- [ ] Ativar SPI2 em modo escravo MODE 3 com DMA RX circular de alta prioridade.
- [ ] Habilitar callbacks de half/full transfer para alimentar o roteador de mensagens.
- [ ] Configurar o TX para operar em modo normal ou via interrupção, evitando contenção com os timers críticos.
- [ ] Teste: usar a Raspberry Pi (ou gerador SPI) para enviar frames de teste e validar o disparo dos callbacks sem perda de sincronismo.

## 7. USART1 (Virtual COM Port) e serviço de log
- [ ] Configurar a USART1 para 115200 bps, 8 N 1.
- [ ] Implementar `_write()` ou utilizar o serviço de console previsto no firmware.
- [ ] Se necessário, habilitar uma fila não bloqueante para logs extensos (`log_poll()` + TX IT).
- [ ] Teste: enviar mensagens de boot/diagnóstico no `app_init`/loop e monitorar no terminal USB a continuidade do fluxo mesmo com SPI e timers ativos.

Atualize este roteiro marcando as etapas concluídas e registrando observações adicionais conforme avançar.
