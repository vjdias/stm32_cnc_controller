# Mapeamento de Pinos configurado no STM32 (CubeMX)

Este arquivo resume todas as atribuições de pinos que constam no projeto
`CNC_Controller.ioc`. Os dados foram coletados diretamente do arquivo `.ioc`
para refletir o estado atual gerado pelo STM32CubeMX.

| Pino | Função/Periférico | Modo/Observações |
| --- | --- | --- |
| **PA0** | TIM5_CH1 | Canal A do encoder conectado ao TIM5 (quadratura X/Y). |
| **PA1** | TIM5_CH2 | Canal B do encoder no TIM5. |
| **PA2** | TIM15_CH1 | Saída principal do TIM15 (PWM utilizado pelo firmware). |
| **PA4** | SPI1_NSS | Entrada NSS do SPI1 em modo escravo com hardware NSS. |
| **PA5** | SPI1_SCK | Clock do SPI1 configurado em modo escravo full-duplex. |
| **PA6** | SPI1_MISO | MISO do SPI1 (modo escravo, full-duplex). |
| **PA7** | SPI1_MOSI | MOSI do SPI1 (modo escravo, full-duplex). |
| **PA13** | SYS_JTMS-SWDIO | Linha SWDIO para depuração (Serial Wire). |
| **PA14** | SYS_JTCK-SWCLK | Linha SWCLK para depuração (Serial Wire). |
| **PA15** | TIM2_CH1 | Canal A do encoder ligado ao TIM2 (quadratura X). |
| **PB3** | TIM2_CH2 | Canal B do encoder ligado ao TIM2 (quadratura X). |
| **PB6** | USART1_TX | Transmissão da USART1 (Virtual COM Port). |
| **PB7** | USART1_RX | Recepção da USART1 (Virtual COM Port). |
| **PB13** | TIM15_CH1N | Saída complementar do TIM15 (PWM). |
| **PE3** | TIM3_CH1 | Canal A do encoder ligado ao TIM3 (quadratura adicional). |
| **PE4** | TIM3_CH2 | Canal B do encoder ligado ao TIM3. |

## Saídas de controle dos motores (STEP / DIR / EN)

Os sinais de movimentação dos três eixos são configurados manualmente em
`App/Src/board_config.c` para garantir modo *push-pull* em alta velocidade.
O CubeMX mantém esses pinos como analógicos por padrão; portanto, utilize a
tabela abaixo como referência para a fiação dos drivers externos:

| Eixo | STEP | DIR | EN (nível baixo = habilita) |
| --- | --- | --- | --- |
| **X** | **PB4** | **PA3** | **PC4** |
| **Y** | **PB0** | **PB2** | **PC5** |
| **Z** | **PB1** | **PA2** | **PE1** |

Notas:

- O firmware inicializa STEP/DIR em nível baixo e EN em nível alto para deixar
  os drivers desabilitados até que o movimento seja liberado. Para energizar
  um eixo, force o respectivo pino EN a nível baixo.
- **PE1** (conector Arduino CN3, pino D2) é reservado pelo firmware para o sinal
  **EN_Z**. No CubeMX ele permanece como analógico, porém
  `board_config_apply_motion_gpio()` o reconfigura como *push-pull* e o mantém em
  nível alto até que o eixo Z seja habilitado. Para reutilizar o pino é
  necessário remapear manualmente o enable do eixo Z para outro GPIO disponível.
- **PC7** permanece dedicado ao **TIM3_CH2** (canal B do encoder do eixo Z) após o
  remapeamento aplicado em `board_config_remap_tim3_encoder_pins()`. Evite
  direcionar outros sinais para esse pino para não romper a leitura do encoder.
- Ajustes adicionais (velocidade das bordas e estado seguro) são aplicados em
  `board_config_apply_motion_gpio()`.

Notas adicionais:

- Todos os demais pinos permanecem configurados como **Analógico (GPIO_MODE_ANALOG)**
  pelo CubeMX, conforme a geração automática em `Core/Src/gpio.c`.
- O CubeMX marca pinos críticos como `Locked=true` (ex.: PA4–PA7, PA15, PB6, PB7,
  PB13, PE3, PE4) para evitar alterações acidentais; mantenha **PA13/PA14** livres
  para a interface de depuração **SWD**.
- Os modos específicos de SPI (`NSS_Signal_Hard_Input`, `Full_Duplex_Slave`) e UART
  (`Asynchronous`) seguem definidos na configuração do CubeMX.

Para atualizar esta lista após qualquer modificação no CubeMX, consulte novamente o
arquivo `CNC_Controller/CNC_Controller.ioc` e regenere os trechos acima.
