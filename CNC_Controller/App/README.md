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

Servico de LED — LED1/LED2 discretos
- Arquivos: `Services/Led/led_service.h`, `Services/Led/led_service.c`.
- O servico controla dois LEDs verdes independentes (LED1 e LED2) presentes na placa.
- Mapeamento de pinos via macros (ajuste conforme sua placa/wiring):
  - `LED1_GPIO_PORT`, `LED1_GPIO_PIN`
  - `LED2_GPIO_PORT`, `LED2_GPIO_PIN`
  - `LED_ACTIVE_HIGH` — quando 1: nível alto liga; quando 0: nível baixo liga (ativo em baixo)
- Estado inicial: ambos iniciam DESLIGADOS em `led_service_init()`.
- `led_service_poll()` deve ser chamado periodicamente (já integrado em `app_poll()`) para manter o pisca.

Pinos padrao (B-L475E-IOT01A)
- `LED1_GPIO_PORT=GPIOA`, `LED1_GPIO_PIN=GPIO_PIN_5`  (LD1/verde)
- `LED2_GPIO_PORT=GPIOB`, `LED2_GPIO_PIN=GPIO_PIN_14` (LD2)
- `LED_ACTIVE_HIGH=1`

Protocolo LED_CTRL (LED1/LED2)
- Tipo: `REQ_LED_CTRL = 0x07`
- Tamanho: 12 bytes
  - [0] 0xAA (header)
  - [1] 0x07 (tipo)
  - [2] frameId
  - [3] ledMask — bit0=LED1, bit1=LED2
  - [4] LED1_mode (0=off, 1=on, 2=pisca)
  - [5..6] LED1_freqHz (big-endian) — ciclos por segundo; ignorado se modo != 2
  - [7] LED2_mode (0=off, 1=on, 2=pisca)
  - [8..9] LED2_freqHz (big-endian)
  - [10] parity — XOR dos bytes [1..9]
  - [11] 0x55 (tail)
- Semantica:
  - Apenas os LEDs com bit correspondente em `ledMask` sao atualizados.
  - Frequencia zero em modo pisca cancela o pisca e desliga o LED.
  - O pisca utiliza resolucao de 1 ms baseada em `HAL_GetTick()` (aprox. 500 Hz max).
- Resposta `RESP_LED_CTRL` permanece com 7 bytes (ACK/estado).

Observacao
- Arquivos aqui ficam fora de Core/ para evitar conflitos com o CubeMX.
- Inclua os .c de `App/Src` no projeto CubeIDE e garanta `App/Inc` em Paths & Symbols.
- Para rodar testes: ver README principal (secao Testes Unitarios).
