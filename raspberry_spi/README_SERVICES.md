# CNC Controller — Guia Rápido de Serviços (SPI)

Este guia mostra, de ponta a ponta, como acionar os serviços do firmware via SPI usando os utilitários Python em `raspberry_spi/`.

Importante
- O modo demo contínuo foi desativado: a linha `motion_demo_set_continuous(1);` está comentada em `Core/Src/main.c`. O STM32 agora aguarda somente comandos reais via SPI.
- O SPI2 do STM32 opera “por quadro” (DMA NORMAL) e responde a cada transação de 42 bytes.
- Para começar, use velocidades baixas e movimentos curtos (evite exceder o normal do maquinário).

## Pré‑requisitos
- Raspberry Pi com SPI habilitado e Python 3.
- Pacotes: `spidev` (`sudo apt install python3-spidev` ou `pip install spidev`).
- Cabos: `PD0/PD1/PD3/PD4` (NSS/SCK/MISO/MOSI) do STM32 conectados ao SPI mestre; GND comum.
- Modo SPI 3 no host (CPOL=1, CPHA=1).

## Fluxo de Movimento (básico)

1) Estado inicial (opcional)
- `python3 cnc_spi_client.py queue-status --frame-id 1 --speed 100000`

2) Enfileirar um segmento (ex.: XYZ = 2000/1600/1200 passos; dir = forward)
- `python3 cnc_spi_client.py queue-add \
    --frame-id 2 \
    --dir 0x07 \
    --vx 1000 --sx 2000 \
    --vy 800  --sy 1600 \
    --vz 600  --sz 1200 --speed 100000`

3) Iniciar execução
- `python3 cnc_spi_client.py start-move --frame-id 3 --speed 100000`

4) Acompanhar status (state/pct/erros PID)
- `python3 cnc_spi_client.py queue-status --frame-id 4 --speed 100000`

5) Finalizar/abortar
- `python3 cnc_spi_client.py end-move --frame-id 5 --speed 100000`
- Emergência (independe dos TMCs):
  - `python3 cnc_spi_client.py safe-off --frame-id 6 --skip-tmc --speed 100000`

Observações
- `--speed 100000` estabiliza o início; suba gradualmente (250k/500k/1M) conforme o cabeamento permitir.
- Reenvie `queue-status` para ver `state` e `pctX/Y/Z` evoluindo.

## Serviço de LED (teste de ida/volta SPI)

- Piscar LED1 a 10,00 Hz:
  - `python3 cnc_spi_client.py led-control --frame-id 1 --mask 0x01 --led1-mode 2 --led1-freq 10.0 --settle-delay 2 --speed 100000`
- Resultado esperado: frame de resposta `AB 07 ... 54` e LED piscando.

## TMC5160 (opcional)

- Inicialização para STEP/DIR (exemplo conservador):
  - `python3 tmc5160_cli.py init-stepdir --bus 0 --dev 1 --speed 100000 --ihold 1 --irun 5 --microsteps 16 --stealth`
- Safe‑off no driver específico:
  - `python3 tmc5160_cli.py safe-off --bus 0 --dev 1 --speed 100000 --clear-gstat`
- Dica: se o dev não responder, reduza `--speed`, revise CS/MISO e DIPs (modo SPI). Para parar o movimento imediatamente use `--skip-tmc` no comando do STM32.

## Encoders (leitura)

- Absoluto (int64 no firmware; sombras 32‑bit para depuração):
  - `g_enc_abs32[0..2]` (X/Y/Z)
- Relativo ao “zero” lógico atual:
  - `g_enc_rel32[0..2]`
- Impressão de delta/posições no console (apenas depuração):
  - Habilite `MOTION_DEBUG_ENCODERS` em `App/Src/Services/Motion/motion_service.c`
  - As mensagens só saem quando há movimento (delta != 0) e uma linha final `[ENC DONE ...]` aparece ao término do segmento.

## Dicas de diagnóstico (queue‑status)

Se `queue-status` ficar em handshake `0xA5` sem resposta:
- Regrave e **reset** a placa (garanta build com DMA NORMAL no SPI2).
- Tente `--speed 100000 --tries 10 --settle-delay 0.002`.
- Confirme que o **CS/NSS do STM32 (PD0)** está comutando no seu host.
- Verifique modo SPI 3 no host (CPOL=1, CPHA=1).
- Veja logs via SWO/UART se o firmware imprime `timers_ready` no boot.

## Segurança e boas práticas
- Comece sempre com **velocidades baixas e poucos passos**.
- Garanta GND comum e cabeamento curto; pare o movimento com `safe-off --skip-tmc` sempre que necessário.
- Para encoders NPN OC (ex.: E6B2‑CWZ6C), use **pull‑ups externos para 3,3 V** em A/B/Z e mantenha `PC0/PC2` (Y) em AF + NOPULL.

## Referências de arquivos
- Desativação do demo: `Core/Src/main.c`
- SPI app (RX/TX 42 bytes + DMA + router): `App/Src/app.c`
- SPI2 (DMA NORMAL por quadro): `Core/Src/spi.c`
- Serviço Motion (queue/status/start/end + encoders): `App/Src/Services/Motion/motion_service.c`
- Serviço LED: `App/Src/Services/Led/led_service.c`

