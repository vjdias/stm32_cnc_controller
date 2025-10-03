Cliente SPI para Raspberry Pi — CNC_Controller

Descrição
- Implementa o framing SPI (requests AA..55 / responses AB..54) idêntico ao definido em `CNC_Controller/App/Inc/Protocol`.
- Comandos principais do protocolo: LED, Queue Add/Status, Start/End Move, Home, Probe Level.
- Localização: esta pasta fica fora de `CNC_Controller/` conforme solicitado.

Requisitos (no Raspberry Pi)
- Python 3.9+
- Biblioteca `spidev` instalada: `sudo apt-get install python3-spidev` (ou `pip install spidev`)
- SPI habilitado: `sudo raspi-config` → Interface Options → SPI → Enable

Fiação (Raspberry Pi → STM32 SPI1 Slave)
- MISO ↔ MISO, MOSI ↔ MOSI, SCK ↔ SCK, CE0 (NSS) ↔ NSS, GND comum. Níveis 3V3.
- Modo SPI: MODE 3 (CPOL=1, CPHA=1 no Linux). 8 bits, MSB first.
- Velocidade sugerida inicial: 1 MHz (ajustável). O firmware usa RX DMA circular.

Uso rápido
- LED (configura LED1 piscando a 0,50 Hz, frameId=1):
  `python3 cnc_spi_client.py led-control --frame-id 1 --mask 0x01 --led1-mode 2 --led1-freq 0.5`

- Status da fila de movimentos:
  `python3 cnc_spi_client.py queue-status --frame-id 2`

- Início e fim de movimento:
  `python3 cnc_spi_client.py start-move --frame-id 3`
  `python3 cnc_spi_client.py end-move --frame-id 4`

- Home e Probe Level:
  `python3 cnc_spi_client.py home --frame-id 5 --axes 0x03 --dirs 0x01 --vhome 0x1234`
  `python3 cnc_spi_client.py probe-level --frame-id 6 --axes 0x04 --vprobe 0x0100`

- Teste de eco "hello" via SPI:
  `python3 cnc_spi_client.py hello`
  (envia `AA 68 65 6C 6C 6F 55` e aguarda `AB 68 65 6C 6C 6F 54`)

- (Opcional) Frame de boot "hello" — disponível apenas quando o firmware é
  compilado com `APP_ENABLE_BOOT_TEST_RESPONSES=1`:
  `python3 cnc_spi_client.py boot-hello --tries 10 --chunk-len 7`
  (por padrão o firmware não enfileira mais esse frame automaticamente para
  evitar que respostas reais sejam precedidas por dados de teste)

- Lista resumida com exemplos (sem necessidade de SPI ativo):
  `python3 cnc_spi_client.py examples`

Parâmetros comuns
- `--bus` (padrão 0) e `--dev` (padrão 0) selecionam `/dev/spidev<bus>.<dev>`.
- `--speed` em Hz (padrão 1_000_000).
- `--poll-byte` altera o byte usado durante o polling (padrão 0x3C). Use `--disable-poll`
  para confiar apenas no frame de handshake (útil para testes específicos).

Configuração do driver TMC5160 a partir do Raspberry Pi
-------------------------------------------------------
- O módulo `tmc5160.py` expõe a classe `TMC5160Configurator`, pensada para o barramento
  `/dev/spidev0.1` (bus=0, device=1) utilizado na placa intermediária do projeto. Ela
  encapsula a sequência de registradores essenciais para deixar o driver pronto para
  receber pulsos STEP/DIR.
- Pré-requisitos no Raspberry Pi:
  - Ativar o SPI em `raspi-config` (menu Interfaces → SPI) e garantir que a sobreposição
    (`dtoverlay`) para o barramento esteja habilitada no `config.txt`. O overlay padrão
    `dtparam=spi=on` cria `/dev/spidev0.0` / `/dev/spidev0.1`, ligados ao **controlador SPI0**
    do BCM2711 e compatíveis com o cabeamento atual (STM32 em `/dev/spidev0.0`, TMC5160 em
    `/dev/spidev0.1`). Caso utilize o SPI1 em futuras revisões, habilite `dtoverlay=spi1-3cs`
    para expor `/dev/spidev1.0` / `/dev/spidev1.1` / `/dev/spidev1.2`.
  - Instalar `python3-spidev` ou `pip install spidev`.
  - Ligar os sinais: SCK ↔ CLK do TMC5160, MOSI ↔ SDI, MISO ↔ SDO, CE0 ↔ CSN. Alimente
    a lógica em 3V3 e compartilhe o GND.
- Uso rápido em Python:
  ```python
  from raspberry_spi.tmc5160 import TMC5160Configurator

  with TMC5160Configurator(bus=0, device=1, speed_hz=4_000_000) as driver:
      driver.configure()  # escreve GSTAT, GCONF, IHOLD_IRUN, etc. com o preset padrão

      # Opcional: sobrescreva algum registrador específico
      driver.write_register(0x10, 0x00071F0A)  # Ex.: aumenta o tempo de ramp down
  ```
- Interface de terminal (`tmc5160_cli.py`):
  ```bash
  # Aplica o preset padrão usando /dev/spidev0.1 (bus=0, dev=1)
  python3 tmc5160_cli.py

  # Ajusta registradores adicionais (endereços em decimal/hex ou aliases)
  python3 tmc5160_cli.py --gconf 0x00000005 --write 0x20=0x12345678

  # Executa apenas as escritas informadas, sem preset padrão
  python3 tmc5160_cli.py --no-defaults --write chopconf=0x10410150 --write pwmconf=0xC10D0024
  ```
  O comando aceita `--bus`, `--dev` e `--speed` para apontar outro dispositivo SPI
  e expõe flags para sobrescrever rapidamente os registradores do preset
  (`--gconf`, `--gstat`, `--ihold-irun`, etc.) ou listas extras de escrita via
  `--write`. Caso nenhuma flag seja passada, o preset padrão é aplicado e o driver
  fica pronto para receber pulsos STEP/DIR.
- O método `configure()` aplica o preset padrão (`TMC5160RegisterPreset.default()`), que
  limpa falhas (`GSTAT`), ativa modo Step/Dir (`GCONF`), define correntes de hold/run e
  parâmetros de chopper/pwm adequados para microstepping de 1/16.
- Para ajustes finos, crie um preset próprio:
  ```python
  from raspberry_spi.tmc5160 import TMC5160RegisterPreset, TMC5160Configurator

  silent_preset = TMC5160RegisterPreset(
      writes=((0x01, 0x07), (0x00, 0x04), (0x10, 0x00050708), (0x70, 0xC10D0010))
  )

  cfg = TMC5160Configurator(register_preset=silent_preset)
  cfg.configure()
  cfg.close()
  ```
- Em modo contexto (`with ...`), o driver é aberto automaticamente e fechado ao final.
  Fora dele, chame `close()` manualmente após terminar a configuração.
- Se precisar disparar sequências adicionais (ex.: corrente dinâmica durante manutenção),
  use `apply_registers()` com uma lista de pares `(endereço, valor)`.
- Antes de iniciar movimentos, garanta que o EN do TMC5160 esteja em nível ativo e que o
  STM32 esteja pronto para gerar pulsos STEP/DIR usando os parâmetros acordados.

Notas de protocolo
- Requests: header `0xAA`, tail `0x55`.
- Responses: header `0xAB`, tail `0x54`.
- Paridade:
  - Alguns frames usam paridade BYTE (XOR de [1..N])
  - Outros usam paridade BIT (redução de XOR em um único bit no LSB do byte de paridade)
  - Start/End/Queue-Status-Req/FPGA-Status-Req não possuem campo de paridade (4 bytes)

Limitações e dicas
- O STM32 é escravo: para “ouvir” uma resposta é necessário gerar clock no master (RPi).
  Quem decide quando disparar um novo *poll* é o método
  `CNCClient.exchange()` do cliente Python. Ele valida o handshake e só
  continua se `_extract_response_frame()` **não** encontrar `0xAB ... 0x54` no
  primeiro retorno do DMA. Essa condição (handshake composto apenas por
  `0xA5/0x00`, ou qualquer sequência que não satisfaça o decoder) dispara o
  próximo poll. Quando o frame de handshake já traz um payload completo, a
  função encerra imediatamente, sem polls extras. O loop também termina se o
  limite imposto por `--tries` for atingido sem uma resposta válida. Por padrão,
  o cliente envia `0x3C` em todos os bytes durante esse polling (configurável
  via `--poll-byte`), evitando colisão com o header `0xAA` dos requests.【F:raspberry_spi/cnc_client.py†L310-L339】
- Caso o serviço no firmware ainda não publique respostas, um timeout pode ocorrer.
- Comandos com resposta aguardam, por padrão, até 5 polls (`--tries`) com
  atraso de 1 ms (`--settle-delay`). Esses parâmetros apenas controlam quanto
  tempo o cliente insiste antes de desistir: eles **não** eliminam o terceiro
  ciclo observado no início da sessão porque o firmware STM32 sempre segura um
  reinício do DMA por pelo menos uma iteração do laço principal. Esse atraso é
  implementado em `CNC_Controller/App/Src/app.c` via `APP_SPI_RESTART_DEFER_MAX`
  e a flag `g_spi_restart_defer`, garantindo que o roteador tenha tempo de
  decodificar o pedido e gerar uma resposta antes de disponibilizar um novo
  quadro de 42 bytes.【F:CNC_Controller/App/Src/app.c†L44-L76】【F:CNC_Controller/App/Src/app.c†L326-L404】【F:CNC_Controller/App/Src/app.c†L596-L683】
  Enquanto `g_spi_tx_pending_ready` permanece limpo, `app_spi_try_restart_dma`
  posterga o reinício e, quando o timeout interno expira, o canal volta a
  transmitir apenas o padrão `0xA5`. Por isso, mesmo com `--settle-delay`
  elevado, o segundo poll ainda coleta somente o handshake; o payload real só
  aparece no ciclo seguinte, depois que o serviço publica a resposta no FIFO e
  o buffer DMA é rearmado.
- Para experimentar uma troca em dois ciclos (handshake + 1 poll) é necessário
  alterar o firmware: reduzir `APP_SPI_RESTART_DEFER_MAX` para `0` elimina a
  espera obrigatória, mas isso reabre a possibilidade do mestre capturar apenas
  `0xA5` caso a resposta demore mais que o intervalo entre polls. Outra opção é
  aumentar o valor da constante para conceder mais iterações ao laço principal
  antes do timeout e observar, via os logs do SPI, se o segundo ciclo já passa
  a carregar `0xAB ... 0x54`. Em qualquer caso, os parâmetros do cliente Python
  continuam apenas como proteção contra timeouts.
- Handshake inicial `0xA5`: quando o primeiro frame recebido contém somente `0xA5`
  (READY) ou `0x00`, isso indica apenas que o firmware aceitou o request, mas ainda está
  preparando a resposta. O cliente então executa polls adicionais usando o byte
  configurado (ex.: `0x3C`) até encontrar `0xAB ... 0x54`. É comum a resposta aparecer
  apenas no segundo ou terceiro poll, especialmente quando o firmware precisa atualizar
  registradores antes de publicar o frame.
- Exemplificando um ciclo completo de LED Control (`--frame-id 1 --mask 0x01`):
  ```
  SPI TX bits: ... AA 07 01 01 01 00 00 06 55
  SPI RX bits: ... A5 A5 A5 A5 A5 A5 A5 A5 A5
  SPI TX bits: ... 3C 3C 3C 3C 3C 3C 3C 3C 3C
  SPI RX bits: ... A5 A5 A5 A5 A5 A5 A5 A5 A5
  SPI TX bits: ... 3C 3C 3C 3C 3C 3C 3C 3C 3C
  SPI RX bits: ... AB 07 01 01 00 07 54
  ```
  Na primeira leitura vemos apenas `0xA5` (READY). O segundo poll ainda não contém a
  resposta porque o DMA foi religado apenas com o padrão de handshake; o payload só
  aparece no terceiro ciclo depois que o firmware finaliza o processamento interno e
  publica o ACK no FIFO. Modificar `APP_SPI_RESTART_DEFER_MAX` no firmware é o caminho
  correto para experimentar uma troca em apenas dois ciclos.

