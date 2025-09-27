# Pipeline de polls SPI e motivo do terceiro ciclo

Este documento explica, passo a passo, por que o firmware STM32 retorna a
resposta do comando `led-control` somente no terceiro ciclo SPI (handshake +
2 polls) quando utilizado com o cliente `cnc_spi_client.py`.

## Visão geral do fluxo DMA

1. **Recepção do frame `0xAA ... 0x55`** – cada transferência SPI utiliza 42
   bytes, preenchidos no escravo pelos estados de handshake `READY/BUSY`.
   Quando um request completo chega, a ISR de `HAL_SPI_TransmitReceive_DMA`
   chama `app_spi_handle_txrx_complete`. Essa rotina analisa o buffer, localiza
   o frame e o enfileira para processamento, além de sinalizar que o DMA precisa
   ser religado posteriormente (`g_spi_need_restart = 1`).【F:CNC_Controller/App/Src/app.c†L596-L683】
2. **Janela de defer** – a mesma ISR arma `g_spi_restart_defer = 1`. Enquanto
   essa flag estiver ativa, `app_spi_try_restart_dma` não reabre o canal de DMA
   caso ainda não exista payload pronto (`g_spi_tx_pending_ready == 0`). A ideia
   é dar ao laço principal (`app_poll`) uma oportunidade de processar o pedido e
   gerar a resposta antes que outro poll do mestre aconteça.【F:CNC_Controller/App/Src/app.c†L44-L76】【F:CNC_Controller/App/Src/app.c†L326-L404】
3. **Processamento no laço principal** – `app_poll` retira o request da fila,
   alimenta o roteador e, no caso do LED, o serviço codifica um ACK imediatamente
   em `led_push_response`, colocando o frame na FIFO de respostas.【F:CNC_Controller/App/Src/app.c†L183-L225】【F:CNC_Controller/App/Src/Services/Led/led_service.c†L313-L347】
4. **Promoção do payload para o DMA** – assim que há dados pendentes,
   `app_poll` copia o frame para `g_spi_tx_pending_buf` e limpa a flag de defer
   (`g_spi_restart_defer = 0`). Se o canal de transmissão ainda estiver parado,
   `app_spi_restart_dma` injeta o payload no buffer ativo antes de liberar o
   próximo ciclo, fazendo com que o mestre leia `0xAB ... 0x54` logo em
   seguida.【F:CNC_Controller/App/Src/app.c†L202-L260】【F:CNC_Controller/App/Src/app.c†L326-L404】

## O que o firmware considera como "poll"

- **Quadros só com `APP_SPI_CLIENT_POLL_BYTE`** – quando a transferência
  recebida pelo DMA começa com `0x3C` e não carrega `0xAA ... 0x55`,
  `app_spi_handle_txrx_complete` trata a rodada como uma enquete do mestre. O
  status devolvido passa a ser `READY`, sinalizando que o escravo apenas mantém
  a porta aberta aguardando que o host repita a leitura.【F:CNC_Controller/App/Src/app.c†L429-L509】
- **Quadros válidos `0xAA ... 0x55`** – estes são interpretados como comandos.
  Após enfileirar o pedido, a ISR ativa `g_spi_restart_defer` e deixa o canal de
  DMA pausado até que `app_poll` tenha a chance de preparar a resposta para o
  próximo ciclo.【F:CNC_Controller/App/Src/app.c†L429-L500】

Em ambos os casos, a conclusão da transferência por DMA é o gatilho que fecha
o "poll" atual e arma o próximo. O mestre só avança porque o escravo invocou
`HAL_SPI_TransmitReceive_DMA` novamente em `app_spi_restart_dma`, realimentando
o periférico com um novo quadro de 42 bytes.【F:CNC_Controller/App/Src/app.c†L356-L420】

## O que faz o `pop` da fila SPI andar?

1. **Fila preenchida na interrupção** – ao localizar um frame válido,
   `app_spi_handle_txrx_complete` o coloca em `g_spi_rx_queue` e aumenta
   `g_spi_rx_queue_count`. Nenhum dado é consumido na ISR; ela apenas sinaliza
   que o loop principal deve processar o item.【F:CNC_Controller/App/Src/app.c†L429-L500】
2. **Consumo periódico em `app_poll`** – o laço principal chama
   `app_spi_queue_pop` até esvaziar a fila. Cada chamada que encontra dados
   incrementa `g_spi_rx_queue_tail` e decrementa o contador, efetivamente
   avançando o "ponteiro" da memória circular onde os requests ficam
   armazenados.【F:CNC_Controller/App/Src/app.c†L160-L218】【F:CNC_Controller/App/Src/app.c†L509-L569】
3. **Resposta pronta libera o próximo poll** – após processar o comando, o
   serviço correspondente insere a resposta na FIFO global. Quando
   `app_poll` detecta esse payload, ele copia o conteúdo para
   `g_spi_tx_pending_buf`, derruba o defer e chama `app_spi_try_restart_dma`, o
   que reabre o DMA e permite que o mestre observe a nova resposta na enquete
   seguinte.【F:CNC_Controller/App/Src/app.c†L202-L260】【F:CNC_Controller/App/Src/app.c†L326-L404】

### Dá para encurtar o caminho sem reescrever tudo?

Existe alguma margem para remover etapas intermediárias sem refatorar o
firmware inteiro:

1. **Consumir o FIFO do LED já no contexto do serviço** – hoje o ACK só é
   promovido quando `app_poll` volta a rodar, porque `led_push_response`
   enfileira a mensagem em `g_app_responses`. Uma pequena alteração seria o
   serviço, ao enfileirar, também acionar `app_spi_try_restart_dma` caso
   `g_spi_tx_pending_ready` esteja zerado. Assim o DMA é rearmado logo após o
   comando ser concluído, sem esperar pelo próximo tick do laço principal.
   【F:CNC_Controller/App/Src/Services/Led/led_service.c†L313-L347】【F:CNC_Controller/App/Src/app.c†L202-L260】
2. **Reaproveitar o buffer ativo do DMA** – outra alternativa de baixo impacto
   é substituir a cópia para `g_spi_tx_pending_buf` por um swap com o buffer que
   o DMA usou na última recepção (`g_spi_rx_dma_buf`). O código já possui ambas
   as regiões alinhadas e com o mesmo tamanho; bastaria trocar os ponteiros em
   `app_poll` antes de chamar `app_spi_restart_dma`. Dessa forma, a resposta que
   saiu da FIFO vira o próximo quadro sem passar por uma etapa de `memcpy`.
   【F:CNC_Controller/App/Src/app.c†L202-L260】【F:CNC_Controller/App/Src/app.c†L509-L569】

Ambas as abordagens mantêm o desenho existente (fila + DMA) e encurtam o
intervalo entre processar o comando e entregar o frame de saída, reduzindo a
chance de o mestre precisar de um terceiro poll.

Portanto, o "pop" da fila depende do `app_poll`: é ele quem consome os quadros
recebidos e inicia o tratamento que culmina na promoção do payload para o DMA,
destravando o próximo polling vindo do Raspberry Pi.

## Por que o terceiro poll aparece?

A constante `APP_SPI_RESTART_DEFER_MAX` define quantas iterações de `app_poll`
precisam ocorrer antes que o firmware entregue um novo quadro ao mestre mesmo
sem payload disponível. O valor atual (`1`) garante ao menos uma passagem pelo
laço principal, mas, se o serviço ainda não tiver produzido a resposta ao final
desse primeiro passe (por exemplo, devido ao tempo gasto reconfigurando GPIO e
TIM para o LED), `app_spi_try_restart_dma` sai do modo de defer e religa o DMA
apenas com o padrão `0xA5`. Isso explica por que o segundo poll continua vendo
`READY` apesar de `--settle-delay` alto: trata-se do timeout interno da janela de
preparação expirar antes que o payload seja promovido.【F:CNC_Controller/App/Src/app.c†L44-L76】【F:CNC_Controller/App/Src/app.c†L326-L404】

Somente no ciclo subsequente o serviço conclui as operações, o FIFO disponibiliza
os bytes e o DMA é rearmado com a resposta real, aparecendo finalmente no terceiro
poll (handshake + 2 consultas).

## Quando o cliente Python decide repetir o poll?

- **Handshakes que não contêm `0xAB ... 0x54`** – após transmitir o request,
  `CNCClient.exchange()` chama `_extract_response_frame`. Se o decoder não
  encontrar um frame válido (por exemplo, porque o escravo devolveu apenas
  `0xA5` ou `0x00`), o cliente considera que a resposta ainda não está pronta e
  envia um novo poll preenchido com o byte acordado (`0x3C` por padrão).【F:raspberry_spi/cnc_client.py†L319-L354】
- **Handshakes com resposta completa** – quando o primeiro retorno do DMA já
  carrega o frame `0xAB ... 0x54`, `_extract_response_frame` devolve a mensagem
  e o laço termina imediatamente, sem polls adicionais.【F:raspberry_spi/cnc_client.py†L310-L333】
- **Falhas após o número máximo de tentativas** – caso nenhum poll produza um
  frame válido, o método levanta `TimeoutError` depois de esgotar `--tries`,
  indicando que o firmware não publicou a resposta a tempo.【F:raspberry_spi/cnc_client.py†L332-L354】

## Como testar alterações

- Reduzir `APP_SPI_RESTART_DEFER_MAX` para `0` remove o timeout interno e fará o
  DMA ser rearmado imediatamente. O segundo poll passará a depender estritamente
  do tempo de processamento do serviço; se ele ainda não tiver resposta pronta,
  o mestre voltará a capturar apenas `0xA5`.
- Aumentar o valor para `2` ou `3` concede mais iterações ao laço principal
  antes do fallback, permitindo avaliar se o ACK do LED fica pronto a tempo para
  o segundo ciclo.
- Monitorar `LOG_SVC_APP` ajuda a confirmar quando o payload foi realmente
  enfileirado (`spi_tx` / `spi_rx`) e em que momento o DMA é reiniciado.

Em todos os casos, `--settle-delay` e `--tries` continuam servindo apenas para
controlar o timeout do cliente Python; eles não interferem nos mecanismos acima.
