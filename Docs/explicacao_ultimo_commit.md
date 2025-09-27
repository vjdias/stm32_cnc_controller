# Explicação do commit "Delay SPI DMA restart until payload is ready"

Este documento resume, em português, as alterações introduzidas pelo commit que
adiou a reativação do DMA de SPI até que o firmware tenha preparado um payload
de resposta.

## Problema observado

Quando o Raspberry Pi concluía o primeiro ciclo de transmissão (42 bytes) com o
STM32, ele precisava efetuar um segundo ciclo idêntico para finalmente receber a
resposta real. O motivo era que o firmware reiniciava o DMA de transmissão
imediatamente após concluir uma transferência, antes que o serviço responsável
pela resposta tivesse carregado os dados no buffer. Como consequência, o STM32
retornava apenas o padrão de handshake `0xA5`, forçando o mestre a executar uma
nova rodada de polling para então obter o payload válido.

## Estratégia adotada

O commit introduziu um mecanismo simples de "adiamento" (defer) que segura a
reinicialização do DMA por um ciclo de `app_poll` assim que uma requisição é
processada. Enquanto a flag de defer estiver ativa, o firmware espera que o
serviço produtor de resposta preencha um buffer pendente. Assim que o buffer é
preparado (ou, no máximo, após um pequeno timeout de segurança), o DMA é
reativado já com o payload real, eliminando a necessidade da segunda rodada de
42 bytes.

## Principais mudanças no código

- Foram adicionados buffers e flags auxiliares (`g_spi_tx_pending_buf`,
  `g_spi_tx_pending_ready`, `g_spi_restart_defer`, entre outros) para armazenar o
  payload pronto e indicar quando o DMA pode ser religado.【F:CNC_Controller/App/Src/app.c†L50-L78】
- A função `app_spi_try_commit_pending_to_active` agora copia o conteúdo do
  buffer pendente para o buffer DMA ativo somente quando há payload disponível,
  limpando simultaneamente a flag de defer.【F:CNC_Controller/App/Src/app.c†L362-L433】
- `app_poll` passou a chamar a rotina de commit antes e depois de alimentar o
  roteador, garantindo que qualquer resposta enfileirada seja promovida ao DMA
  o quanto antes.【F:CNC_Controller/App/Src/app.c†L183-L217】
- Ao detectar o fim de uma transferência (`app_spi_handle_txrx_complete`), o
  firmware marca que um restart é necessário, mas delega à rotina de defer a
  decisão de quando reiniciar o DMA, respeitando a janela em que o payload é
  preparado.【F:CNC_Controller/App/Src/app.c†L638-L683】

## Resultado esperado

Com essa abordagem, o STM32 já começa a preparar o payload real durante a
janela do handshake `0xA5`. Dessa forma, quando o Raspberry Pi inicia o próximo
ciclo de 42 bytes com o padrão `0x3C`, o firmware responde imediatamente com os
bytes efetivos do serviço solicitado, sem precisar de uma repetição extra.

