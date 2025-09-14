Estrutura do App (FSM como servicos) e protocolo

Pastas
- Inc/: headers autorais (Protocol/, Services/). Adicione `App/Inc` ao include path.
- Src/: fontes autorais (Protocol/, Services/).
- Tests/: testes unitarios (CMake/CTest) que compilam no host (Ubuntu).

Integracao (resumo)
- O RX DMA do SPI alimenta o roteador (router_feed_bytes), que monta frames AA..55.
- O roteador valida header/tail/paridade e despacha pelo msgType para servicos
  (motion, home, probe, led, safety).
- As respostas (AB..54) sao enfileiradas em um FIFO para TX.

Cola SPI <-> Router
- Arquivos: `App/Src/app.c`, `App/Inc/app.h`.
- `app_init()` cria o FIFO de respostas, registra os handlers dos servicos, inicializa o roteador e inicia `HAL_SPI_Receive_DMA()` em buffer circular.
- `HAL_SPI_RxHalfCpltCallback`/`HAL_SPI_RxCpltCallback` alimentam o roteador com metade/segunda metade do buffer.
- `app_poll()` verifica o FIFO e transmite uma resposta disponivel via `HAL_SPI_Transmit_IT()` (TX por interrupcao). Obs.: o Cube gerou TX DMA em modo circular; para TX de tamanho variavel recomenda-se trocar para modo "normal" se desejar usar DMA no TX.

Servico de Log (USART1 VCP)
- Arquivos: `Services/Log/log_service.h`, `Services/Log/log_service.c`.
- Nao-interruptivo: usa buffer circular e transmite em pequenos blocos via `HAL_UART_Transmit_IT()` apenas quando o barramento esta livre (acionado por `log_poll()` no final do `app_poll()`).
- Dois modos:
  - Conciso (`LOG_MODE_CONCISE`): IDs de servico/estado e status numerico (ex.: `L,svc=1,state=2,status=0`).
  - Verboso (`LOG_MODE_VERBOSE`): nomes por extenso e status textual (ex.: `LOG,service=motion,state=queued,status=ok`).
- Habilitacao: `LOG_ENABLE` (compile-time, padrao=1) e `log_set_enabled()` (runtime). Quando desabilitado em build-time, as funcoes viram no-ops e nao afetam a compilacao.
- Padrao: inicia em modo VERBOSE (`LOG_DEFAULT_MODE=LOG_MODE_VERBOSE`). Pode alterar via compile-time ou `log_set_mode()`.

Servico de LED — RGB
- Arquivos: `Services/Led/led_service.h`, `Services/Led/led_service.c`.
- O servico suporta LED RGB por GPIO (nivel logico; PWM pode ser integrado depois via TIM).
- Mapeamento de pinos por macros de pre-processador (defina conforme sua placa):
  - `LED_R_GPIO_PORT`, `LED_R_GPIO_PIN`
  - `LED_G_GPIO_PORT`, `LED_G_GPIO_PIN`
  - `LED_B_GPIO_PORT`, `LED_B_GPIO_PIN`
  - `LED_ACTIVE_HIGH` (0 padrao na B-L475E-IOT01A) — quando 1: nivel alto liga; quando 0: nivel baixo liga (ativo em baixo)
- Caso NAO defina os tres canais RGB, o servico usa um LED unico (macros antigas `LED_GPIO_PORT`/`LED_GPIO_PIN`) e interpreta qualquer componente R/G/B != 0 como ON.
- Estado inicial: todos os canais iniciam DESLIGADOS (nível lógico de OFF) em `led_service_init()`.

Pinos padrao (B-L475E-IOT01A)
- `LED_R_GPIO_PORT=GPIOB`, `LED_R_GPIO_PIN=GPIO_PIN_1`  (LDx/vermelho)
- `LED_G_GPIO_PORT=GPIOB`, `LED_G_GPIO_PIN=GPIO_PIN_14` (LD1/verde)
- `LED_B_GPIO_PORT=GPIOB`, `LED_B_GPIO_PIN=GPIO_PIN_7`  (LD2/azul)
- `LED_ACTIVE_HIGH=0` (ativo em baixo nesta placa)

Protocolo LED_CTRL (RGB)
- Tipo: `REQ_LED_CTRL = 0x07`
- Tamanho: 9 bytes
  - [0] 0xAA (header)
  - [1] 0x07 (tipo)
  - [2] frameId
  - [3] ledMask — bit0=R, bit1=G, bit2=B
  - [4] R (0..255)
  - [5] G (0..255)
  - [6] B (0..255)
  - [7] parity — XOR dos bytes [1..6]
  - [8] 0x55 (tail)
- Semantica:
  - Somente os canais marcados em ledMask sao atualizados.
  - Valores nao-zero ligam o canal (nivel logico); para intensidade/cores reais, integrar PWM por TIM.
- Resposta `RESP_LED_CTRL` permanece com 7 bytes (ACK/estado) para simplicidade.

Observacao
- Arquivos aqui ficam fora de Core/ para evitar conflitos com o CubeMX.
- Inclua os .c de `App/Src` no projeto CubeIDE e garanta `App/Inc` em Paths & Symbols.
- Para rodar testes: ver README principal (secao Testes Unitarios).
