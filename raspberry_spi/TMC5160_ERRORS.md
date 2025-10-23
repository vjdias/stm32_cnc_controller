TMC5160 — Erros, Diagnóstico e Leitura pelo SPI

1) Visão Geral
- O TMC5160 reporta condições de erro por três fontes principais:
  - Byte de status (primeiro byte de toda resposta SPI)
  - Registrador GSTAT (0x01): flags latched de falha global
  - Registrador DRV_STATUS (0x6F): detalhes do estágio de potência (curto, open‑load, temperatura)
- A leitura correta dos registradores é feita via um protocolo SPI de duas etapas (descrita abaixo).

2) Representação do Barramento SPI (quadros de 5 bytes)
- Escrita (Write): envia 5 bytes; o primeiro tem o bit 7 = 1 (0x80 | endereço).
  - Ex.: limpar GSTAT (escrever 0x07 em 0x01): [0x81, 0x00, 0x00, 0x00, 0x07]
- Leitura (Read) em duas etapas:
  - 1) Solicita leitura (bit 7 = 0): [endereço, 0x00, 0x00, 0x00, 0x00]
  - 2) Lê resposta com um NOP (0x00): [0x00, 0x00, 0x00, 0x00, 0x00]
  - Em ambas as respostas, o primeiro byte é o “byte de status” do TMC5160; os 4 seguintes são o valor do registrador da operação anterior.
- Endianess dos dados: big‑endian (MSB primeiro nos 4 bytes de dados).

3) Byte de Status (1º byte de toda resposta)
- Bit‑a‑bit (do mais significativo ao menos):
  - 7: status_stop_r
  - 6: status_stop_l
  - 5: position_reached
  - 4: velocity_reached
  - 3: standstill
  - 2: stallguard
  - 1: driver_error  ⟵ indica que existe uma falha; confira GSTAT/DRV_STATUS
  - 0: reset_flag
- Observações:
  - driver_error=1 não diz qual é a falha — apenas que há uma. Consulte GSTAT/DRV_STATUS.
  - reset_flag=1 após reset; limpe GSTAT para zerá‑lo.

4) GSTAT (0x01) — Falhas “latched” globais
- Bits (valor retornado no campo de dados de 32 bits):
  - bit 0 — RESET: houve reset desde a última leitura
  - bit 1 — DRV_ERR: falha no driver (superconjunto; confira DRV_STATUS)
  - bit 2 — UV_CP: subtensão do charge pump (verifique VM/enable)
- Como limpar: escreva 1 nos bits que deseja limpar (ex.: 0x07 limpa todos: RESET/DRV_ERR/UV_CP).

5) DRV_STATUS (0x6F) — Detalhes do estágio de potência
- Bits mais relevantes (valor 32 bits):
  - 31: STST — standstill (não é uma falha)
  - 26: OT   — sobretemperatura
  - 25: OTPW — pré‑aviso de sobretemperatura
  - 24: S2GA — short‑to‑ground na fase A
  - 23: S2GB — short‑to‑ground na fase B
  - 22: S2VSA — short‑to‑supply na fase A
  - 21: S2VSB — short‑to‑supply na fase B
  - 20: OLA — open‑load fase A
  - 19: OLB — open‑load fase B
- Interpretação prática:
  - OT/OTPW: dissipação insuficiente; reduza corrente (IRUN), melhore dissipação
  - S2G*/S2VS*: curto nas fases/alimentação; desligue, confira fiação e módulo
  - OL* (open‑load): fio solto ou motor desconectado

6) Como Ler Erros — Passo a Passo (via CLI deste repo)
- Limpar e ler erros (ex.: /dev/spidev0.2):
  - Consultar status com limpeza de GSTAT:
    - python3 tmc5160_cli.py status --bus 0 --dev 2 --clear-gstat --register gstat,drv_status
  - Somente DRV_STATUS:
    - python3 tmc5160_cli.py status --bus 0 --dev 2 --register drv_status
- Comandos úteis:
  - safe‑off (correntes=0, TOFF=0, FREEWHEEL configurável):
    - python3 tmc5160_cli.py safe-off --bus 0 --dev 2 --clear-gstat
  - Inicialização para STEP/DIR:
    - python3 tmc5160_cli.py init-stepdir --bus 0 --dev 2 --ihold 1 --irun 8 --microsteps 32 --stealth

7) Exemplos de Quadros SPI (brutos)
- Ler DRV_STATUS (0x6F):
  - Request:  [0x6F, 0x00, 0x00, 0x00, 0x00]
  - Reply:    [status, b1, b2, b3, b4]  ⇒ valor = (b1<<24)|(b2<<16)|(b3<<8)|b4
- Limpar GSTAT (0x01):
  - Write:    [0x81, 0x00, 0x00, 0x00, 0x07]
  - Reply:    [status, …dados do comando anterior…]
- Ler GSTAT (0x01):
  - Request:  [0x01, 0x00, 0x00, 0x00, 0x00]
  - Reply:    [status, b1, b2, b3, b4]

8) Fluxo de Diagnóstico Recomendado
- 1) Rode ‘status --clear-gstat’ e observe:
  - Se GSTAT retorna 0 após o clear, mas o byte de status mostra driver_error=1 apenas antes do clear, trate como falha antiga (latched) já resolvida.
  - Se DRV_STATUS acusa OT/OTPW/S2G*/S2VS*/OL*, corrija a causa física e limpe GSTAT novamente.
- 2) Verifique energia/fiação:
  - VM presente e GND comum
  - EN ativo em LOW (ENN) quando desejar operar
  - Conexão de motor: A1/A2 e B1/B2 corretos; sem curtos/open‑load
- 3) Corrente adequada:
  - Reduza IHOLD para diminuir aquecimento em repouso; ajuste IRUN para torque em movimento
- 4) Reexecutar testes (init‑stepdir, movimentos, leituras)

9) Tabela‑resumo (erros comuns)
- Byte de status (bit 1 = driver_error): indica “existe falha”; detalhes em GSTAT/DRV_STATUS
- GSTAT.DRV_ERR=1: há falha — consulte DRV_STATUS e corrija; escreva 1 para limpar
- GSTAT.UV_CP=1: verifique VM, enable (ENN), e integridade de alimentação
- DRV_STATUS.OT/OTPW: sobretemperatura — dissipação/corrente
- DRV_STATUS.S2GA/S2GB/S2VSA/S2VSB: curtos — fiação/módulo
- DRV_STATUS.OLA/OLB: open‑load — fio do motor desconectado ou falho

10) Referências neste projeto
- Decodificação e construção de quadros: raspberry_spi/tmc5160.py:1
- CLI (comandos status/config/init/safe‑off): raspberry_spi/tmc5160_cli.py:1
- Exemplo de preset de registradores: raspberry_spi/tmc5160.py (classe TMC5160RegisterPreset)

