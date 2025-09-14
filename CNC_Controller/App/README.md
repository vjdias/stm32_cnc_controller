Estrutura de App proposta (FSM como serviços) e protocolo.

Pastas
- services/: cada FSM é um serviço isolado, com subpastas por domínio.
- protocol/: definições de frames (IDs, headers, tails), helpers e roteador SPI.

Integração (resumo)
- O RX DMA do SPI alimenta o roteador (router_feed_bytes), que monta frames AA..55.
- O roteador valida header/tail/paridade e despacha pelo msgType para serviços
  (motion, home, probe, led, safety).
- As respostas (AB..54) são enfileiradas em um FIFO para TX (DMA).

Observação
- Arquivos aqui ficam fora de Core/ para evitar conflitos com o CubeMX.
- A integração exige incluir os .c no projeto CubeIDE quando desejar.

