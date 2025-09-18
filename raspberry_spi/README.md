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
- LED (configura LED1 piscando a 5 Hz e LED2 ligado, frameId=1):
  `python3 cnc_spi_client.py led --frame-id 1 --mask 0x03 --led1-mode 2 --led1-freq 5 --led2-mode 1`

- Status da fila de movimentos:
  `python3 cnc_spi_client.py queue-status --frame-id 2`

- Início e fim de movimento:
  `python3 cnc_spi_client.py start-move --frame-id 3`
  `python3 cnc_spi_client.py end-move --frame-id 4`

- Home e Probe Level:
  `python3 cnc_spi_client.py home --frame-id 5 --axes 0x03 --dirs 0x01 --vhome 0x1234`
  `python3 cnc_spi_client.py probe-level --frame-id 6 --axes 0x04 --vprobe 0x0100`

Parâmetros comuns
- `--bus` (padrão 0) e `--dev` (padrão 0) selecionam `/dev/spidev<bus>.<dev>`.
- `--speed` em Hz (padrão 1_000_000).

Notas de protocolo
- Requests: header `0xAA`, tail `0x55`.
- Responses: header `0xAB`, tail `0x54`.
- Paridade:
  - Alguns frames usam paridade BYTE (XOR de [1..N])
  - Outros usam paridade BIT (redução de XOR em um único bit no LSB do byte de paridade)
  - Start/End/Queue-Status-Req/FPGA-Status-Req não possuem campo de paridade (4 bytes)

Limitações e dicas
- O STM32 é escravo: para “ouvir” uma resposta é necessário gerar clock no master (RPi). O cliente já realiza uma leitura com clocks após enviar o request.
- Caso o serviço no firmware ainda não publique respostas, um timeout pode ocorrer.

