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

Seletores de escravos (Chip Select)
- **SPI0** (padrão nos cabeçalhos de 40 pinos do Raspberry Pi)
  - **CE0** → GPIO8 (pino físico 24)
  - **CE1** → GPIO7 (pino físico 26)
  - Dispositivos `spidev`: `/dev/spidev0.0` e `/dev/spidev0.1`.
- **SPI1** (pinos alternativos, úteis quando SPI0 está em uso por outro periférico)
  - **CE0** → GPIO18 (pino físico 12)
  - **CE1** → GPIO17 (pino físico 11)
  - **CE2** → GPIO16 (pino físico 36)
  - Dispositivos `spidev`: `/dev/spidev1.0`, `/dev/spidev1.1` e `/dev/spidev1.2`.

Selecione o valor de `--bus`/`--dev` condizente com o chip-select que pretende usar ao inicializar o cliente SPI.

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

- Frame de boot "hello":
  `python3 cnc_spi_client.py boot-hello --tries 10 --chunk-len 7`
  (espera-se o frame `AB 68 65 6C 6C 6F 54` com header/tail válidos)

- Lista resumida com exemplos (sem necessidade de SPI ativo):
  `python3 cnc_spi_client.py examples`

- Diagnóstico do TMC5160 (limpa `GSTAT` e lê `DRV_STATUS`):
  `python3 cnc_spi_client.py tmc-status --tmc-index 3 --flush-pipeline`
  (garante que o Raspberry Pi descarte flags antigos com `GSTAT=0x07` antes de
  ler o status mais recente `DRV_STATUS (0x6F)`; `--tmc-index` escolhe o CS do
  SPI0 para falar com o driver correto e a saída lista todos os cinco bytes
  recebidos em cada resposta do driver)

Parâmetros comuns
- `--bus` (padrão 0) e `--dev` (padrão 0) selecionam `/dev/spidev<bus>.<dev>`.
- `--tmc-index` (0–4) troca o chip-select do SPI0 ao diagnosticar múltiplos
  TMC5160; quando usado, substitui o valor de `--dev`.
- `--speed` em Hz (padrão 1_000_000).

Notas de protocolo
- Requests: header `0xAA`, tail `0x55`.
- Responses: header `0xAB`, tail `0x54`.
- Paridade:
  - Alguns frames usam paridade BYTE (XOR de [1..N])
  - Outros usam paridade BIT (redução de XOR em um único bit no LSB do byte de paridade)
  - Start/End/Queue-Status-Req/FPGA-Status-Req não possuem campo de paridade (4 bytes)

Limitações e dicas
- O STM32 é escravo: para “ouvir” uma resposta é necessário gerar clock no master (RPi). O cliente envia `0x3C` em todos os bytes durante esse polling, evitando colisão com o header `0xAA` dos requests.
- Caso o serviço no firmware ainda não publique respostas, um timeout pode ocorrer.
- Comandos com resposta aguardam, por padrão, até 5 polls (`--tries`) com
  atraso de 1 ms (`--settle-delay`). Se o firmware demorar mais para responder,
  aumente uma ou ambas as opções para evitar timeouts.

