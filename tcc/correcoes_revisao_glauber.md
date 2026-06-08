# Correções da revisão do Prof. Glauber

Branch de trabalho: `codex/ultima-correcao-tcc`

PDF usado como base: `indicação_correção/TCC_Valdir_Dias_Silva_Junior_revisao_glauber.pdf`

## Mudanças realizadas

| Mudança | Tópico de origem na revisão | Arquivos alterados |
|---|---|---|
| Reescrita do resumo em linguagem mais fluida e macro, reduzindo a concentração inicial de siglas e detalhes de temporizadores. | Resumo: comentário sobre texto muito denso e tecnicalidade excessiva antes das definições. | `src/Cap00/Resumo.tex` |
| Ajuste do abstract para acompanhar o novo nível de abstração do resumo. | Abstract: consequência direta da mudança no Resumo para manter equivalência entre os textos. | `src/Cap00/Abstract.tex` |
| Reorganização da Lista de Abreviaturas em ordem alfabética e inclusão das siglas usadas no texto, como APP, ARR, CRC, E-STOP, FOPDT, ISR, LPTIM, PSC, PWM, SPI, SWV, TIM e TTL. | Lista de Abreviaturas: comentário para ordenar alfabeticamente e incluir siglas citadas em Objetivos, Fundamentação e Protocolo SPI. | `src/Cap00/Abreviaturas.tex` |
| Ampliação da Lista de Símbolos com variáveis usadas na modelagem do motor, PID e FOPDT. | Lista de Símbolos / Fundamentação: comentários pedindo definição de sinais, equações e notações. | `src/Cap00/Simbolos.tex` |
| Inclusão do repositório GitHub do projeto para facilitar continuidade do trabalho. | Capítulo 3: comentário solicitando organização e disponibilização em repositório. | `src/Cap01/Capitulo1.tex` |
| Definição de variação temporal (`jitter`) no primeiro uso relevante e substituição de usos isolados por "variação temporal". | Objetivos / Resultados: comentário pedindo definição de variações temporais. | `src/Cap01/Capitulo1.tex`, `src/Cap03/Capitulo3.tex`, `src/Cap04/Capitulo4.tex`, `src/Cap05/Capitulo5.tex` |
| Inclusão de equações básicas de modelagem de motor de passo e explicação da diferença em relação a motores CC convencionais. | Seção 2.4, Modelagem de motores de passo: comentário pedindo equações e relação/diferença com modelagem de motores CC. | `src/Cap02/Capitulo2.tex` |
| Explicação dos sinais da equação PID incremental, incluindo o papel de `e[k]`, `u[k]`, `T_s` e da diferença finita de segunda ordem. | Seção 2.5, Controlador PID digital: comentário questionando sinais e termo derivativo. | `src/Cap02/Capitulo2.tex` |
| Inclusão de uma seção FOPDT com resposta ao degrau e explicação explícita do papel da função de Heaviside. | Seção FOPDT: comentário perguntando por que o termo `H(t-L)` parecia desaparecer. | `src/Cap02/Capitulo2.tex` |
| Ajuste da linguagem sobre controle cruzado de eixos, usando "controle cruzado de eixos (cross-coupled)" em vez de uma forma solta. | Fundamentação: comentário de linguagem/formatação envolvendo `cross-coupled`. | `src/Cap02/Capitulo2.tex` |
| Justificativa da escolha de SPI para comandos e USART para observabilidade. | Comunicação SPI e USART: comentário para explicar, na apresentação/texto, como se deu a escolha dos itens. | `src/Cap02/Capitulo2.tex` |
| Correção do quadro de espera SPI para 42 bytes e ajuste de referência interna para `\ref{sec:spi_usart}`. | Protocolo SPI: comentários sobre Lista de Abreviaturas, referência e coerência do fluxo. | `src/Cap02/SPIProtocol.tex` |
| Explicação de onde a cinemática é tratada no comando de movimento: o quadro recebe passos relativos cartesianos por eixo, e não uma transformação geométrica completa. | Tabela de requisição de movimento: comentário perguntando onde a cinemática é configurada ou implementada. | `src/Cap02/SPIProtocol.tex` |
| Conversão das listas do TMC5160, TMCS-28 e hardware para ambientes `itemize`, evitando renderização como texto corrido. | Driver TMC5160 / Arquitetura de Hardware: comentários sobre problema de formatação e sugestão de tópicos. | `src/Cap02/DriversTMC.tex`, `src/Cap03/ArquiteturaHardware.tex` |
| Identificação da Raspberry Pi 3 A+ e explicação do papel da base de prototipagem na montagem de bancada. | Arquitetura de Hardware: comentários "Que Raspberry é essa?" e observação sobre protoboard, folgas e corrente. | `src/Cap03/ArquiteturaHardware.tex` |
| Inclusão de um diagrama em blocos para conectar Raspberry Pi, SPI/DMA, camadas do firmware, temporizadores, drivers e encoders. | Metodologia / Arquitetura de software: comentário pedindo figura da arquitetura com blocos conectados e sub-blocos. | `src/Cap03/Capitulo3.tex` |
| Correção da referência bibliográfica do datasheet TMC5160 para evitar citação sem ano visível. Foi usado `s.d.` por não haver ano confirmado no arquivo fonte. | Bibliografia / tabelas de registros: citação aparecia como autor sem ano. | `src/Ref/SampleReferences.bib` |
| Correção de "embarcardo" para "embarcado" e substituição de "jitter reduzido" por "baixa variação temporal". | Conclusão: ajuste textual derivado da padronização de termos. | `src/Cap05/Capitulo5.tex` |

## Revisão adicional usando os dois PDFs

Depois de comparar `versao_enviada/TCC_Valdir_Dias_Silva_Junior.pdf`,
`indicação_correção/TCC_Valdir_Dias_Silva_Junior_revisao_glauber.pdf` e
os arquivos `.tex` atuais, ficou confirmado que a versão enviada à banca
continha trechos que não estão presentes como fonte editável no diretório
`src`. Portanto, a frase anterior "não existem nos arquivos `.tex` atuais"
continua correta, mas agora ela está detalhada abaixo por tópico.

| Tópico anotado no PDF revisado | Situação no PDF enviado | Situação nos fontes atuais | Encaminhamento |
|---|---|---|---|
| Seção 2.8, Sincronização Cruzada de Eixos | Existe no PDF enviado. | Não há seção equivalente no `.tex` atual; há apenas uma menção curta a controle cruzado na seção PID. | A linguagem foi suavizada na fundamentação atual, mas a seção completa não foi recuperada por falta de fonte editável. |
| Seção 2.11, Motores de Passo NEMA 23, passo de 0,9° e seleção de corrente | Existe no PDF enviado com várias subseções. | Não existe no `.tex` atual. | Parte conceitual foi coberta pela nova modelagem de motor de passo, mas a seção NEMA 23 completa não foi recriada. |
| Figura 3.1 e seção 3.2.1, Fluxo do comando `cnc-cli` | Existe no PDF enviado; a figura aparece numerada como Figura 3.1. | Não existe no `.tex` atual, nem foi encontrado arquivo de figura correspondente. | Foi criado um diagrama em blocos geral da arquitetura no Capítulo 3 atual. A figura específica do `cnc-cli` não foi recuperada. |
| Comentário sobre `frameId` no `MOVE_QUEUE_STATUS` | O fluxo `cnc-cli` aparece no PDF enviado e o comentário questiona a apresentação dos campos. | A seção não existe no `.tex`, mas o código confirma a estrutura. | Confirmado no código: requisição `MOVE_QUEUE_STATUS` tem 4 bytes e usa `raw[2]` como `frameId`; resposta tem 12 bytes e também usa `raw[2]` como `frameId`. Ver `CNC_Controller/App/Src/Protocol/Requests/move_queue_status_request.c` e `CNC_Controller/App/Src/Protocol/Responses/move_queue_status_response.c`. |
| Seção 3.4, Identificação experimental do modelo FOPDT | Existe no PDF enviado com subseções 3.4.1 a 3.4.5. | Não existe no `.tex` atual. | A fundamentação FOPDT e a explicação de Heaviside foram adicionadas, mas o procedimento experimental completo não foi reconstituído. |
| Figuras 3.2, 3.3 e 3.4 de validação FOPDT | Existem no PDF enviado. | Não foram encontrados arquivos de imagem correspondentes no repositório. | Não recuperadas; dependem de imagens ou scripts que não aparecem no fonte atual. |
| Seção 3.6, Arquitetura de Controle de Movimento | Existe no PDF enviado. | Não existe no `.tex` atual. | Foi adicionado um diagrama geral em `src/Cap03/Capitulo3.tex`; a seção completa de controle de movimento não foi recuperada. |
| Comentário pedindo diagrama em estilo Simulink | Aparece na seção 3.6 do PDF revisado. | A seção 3.6 não existe no `.tex` atual. | Atendido parcialmente com o diagrama em blocos geral da arquitetura. |
| Seção 3.7, DDA em ponto fixo, e algoritmo associado | Existe no PDF enviado. | Não existe no `.tex` atual. | Não foi possível mover ou reformatar o algoritmo sem fonte editável; a fundamentação do DDA foi mantida e revisada. |
| Seção 3.8, Rampa Trapezoidal, e comentário sobre notação de piso | Existe no PDF enviado. | Não existe no `.tex` atual. | Não aplicado no texto atual porque a equação e a notação comentada não existem nos fontes. |
| Seções 3.9 a 3.15, incluindo PI de posição, fila de movimentos, mapeamento TMC5160, parâmetros, coordenação multi-eixos, simulador Python e boas práticas | Existem no PDF enviado. | Não existem no `.tex` atual. | Não recriadas a partir do PDF para evitar gerar um fonte LaTeX incompleto e sem as figuras/tabelas originais. |
| Figura 3.12 do simulador e Figura 4.1 de comparação simulação/experimento | Existem no PDF enviado. | Não foram encontrados arquivos de imagem correspondentes no repositório. | Não recuperadas. |
| Seção 4.3, Comparação entre simulação e experimento com carga | Existe no PDF enviado. | O `.tex` atual possui outra seção 4.3, chamada "Integração com a Raspberry Pi". | Identificada divergência real entre PDF enviado e fonte atual; a seção experimental do PDF não tem fonte editável no diretório `src`. |

## Itens que não encontrei

Não encontrei, em formato editável no repositório, os fontes LaTeX ou assets
originais dos seguintes itens da versão enviada:

- Seção 2.8, Sincronização Cruzada de Eixos.
- Seção 2.11 e subseções sobre NEMA 23, passo 0,9° e corrente `IRMS`.
- Seção 3.2.1, Fluxo do comando `cnc-cli`, incluindo a Figura 3.1.
- Seção 3.4 completa sobre identificação experimental FOPDT e as Figuras 3.2 a 3.4.
- Seções 3.6 a 3.15 completas, incluindo algoritmos, tabelas e figuras do DDA, rampa, coordenação multi-eixos, simulador e boas práticas.
- Figura 3.5 a Figura 3.12 como arquivos externos.
- Seção 4.3 do PDF enviado, "Comparação entre simulação e experimento com carga", e a Figura 4.1.

O único arquivo de imagem encontrado dentro de `tcc/src` foi `Cap00/IC.jpg`,
que pertence aos elementos pré-textuais e não às figuras técnicas do corpo
do trabalho.

## Classificação detalhada das faltas

Legenda usada abaixo:

- `Comentado diretamente`: há uma anotação do Prof. Glauber no PDF revisado sobre aquele ponto ou sobre o trecho imediatamente associado.
- `Referenciado no PDF`: aparece no sumário, lista de figuras, legenda, corpo do PDF enviado ou em uma seção vizinha, mas não encontrei anotação direta do Glauber sobre o item.
- `Fonte atual`: situação nos arquivos `.tex` dentro de `tcc/src`.
- `Falta exata`: o que está ausente para aplicar a correção de forma limpa no LaTeX.

| Item | Caso | Evidência no PDF/revisão | Fonte atual | Falta exata |
|---|---|---|---|---|
| Seção 2.8, Sincronização Cruzada de Eixos | Comentado diretamente e também referenciado no sumário do PDF enviado. | O PDF revisado mostra anotação próxima ao texto de sincronização cruzada, extraída de forma imperfeita pelo `pdftotext` como comentário de linguagem/LaTeX; o sumário do PDF enviado lista a seção 2.8. | Não existe uma `\section{Sincronização Cruzada de Eixos}` em `tcc/src`. Há apenas uma menção curta a `cross-coupled` na seção de PID. | Falta o fonte completo da seção 2.8 para corrigir a linguagem exatamente no trecho anotado. A correção atual só suaviza a menção conceitual que ainda existe no `.tex`. |
| Seção 2.11, Motores de Passo NEMA 23, passo de 0,9° e seleção de corrente `IRMS` | Referenciado no PDF, sem comentário direto encontrado. | O sumário do PDF enviado lista a seção 2.11 e as subseções 2.11.1 a 2.11.7. | Não existe seção equivalente em `tcc/src`; o fonte atual pula de TMC5160/TMCS-28 para Registros. | Falta o fonte completo dessa seção. Como não encontrei comentário direto do Glauber, isto é divergência entre PDF enviado e fontes atuais, não uma correção solicitada pontualmente. |
| Seção 3.2.1, Fluxo do comando `cnc-cli` | Comentado diretamente. | O PDF revisado traz comentários "Consertar referência" na figura do fluxo e questionamento sobre `frameId`/`MOVE_QUEUE_STATUS`. O PDF enviado contém a Figura 3.1 e a descrição do fluxo. | Não existe `\subsection{Fluxo do comando cnc-cli}` no fonte atual. | Falta o fonte da subseção 3.2.1, o código LaTeX da Figura 3.1 e o arquivo/asset da figura. Sem isso, não dá para corrigir a referência da figura no local original. |
| Campo `frameId` no `MOVE_QUEUE_STATUS` | Comentado diretamente, mas verificável no código. | O comentário do PDF revisado pergunta se `frameId` está apresentado corretamente no monitoramento do `MOVE_QUEUE_STATUS`. | A subseção do TCC não existe no `.tex`, mas o código existe em `CNC_Controller/App/Src/Protocol/Requests/move_queue_status_request.c` e `CNC_Controller/App/Src/Protocol/Responses/move_queue_status_response.c`. | Falta o trecho editável do TCC onde isso era explicado. A resposta técnica foi registrada: requisição tem 4 bytes e usa `raw[2]` como `frameId`; resposta tem 12 bytes e também usa `raw[2]` como `frameId`. |
| Seção 3.4, Identificação experimental do modelo FOPDT | Referenciado no PDF, sem comentário direto encontrado nessa seção. | O sumário do PDF enviado lista a seção 3.4 e as subseções 3.4.1 a 3.4.5; o corpo do PDF enviado contém o procedimento experimental. | Não existe seção 3.4 no fonte atual; há apenas uma nova seção teórica FOPDT no Capítulo 2. | Falta o fonte experimental completo: pré-processamento, extração de marcos temporais, estimativas `K`, `L`, `\tau`, síntese PD e validação gráfica. |
| Figuras 3.2, 3.3 e 3.4, validação FOPDT por eixo | Referenciado no PDF, sem comentário direto encontrado. | A lista de figuras do PDF enviado lista as três figuras de sobreposição velocidade medida/FOPDT. | Não há `\includegraphics` correspondente e não encontrei arquivos de imagem dessas figuras no repositório. | Faltam os assets das figuras e o código LaTeX que as inseria. |
| Seção 3.6, Arquitetura de Controle de Movimento | Comentado diretamente. | O PDF revisado traz o comentário "Aqui deveria ter um diagrama de blocos, no estilo simulink" nessa seção; o PDF enviado lista a seção 3.6 e subseções 3.6.1 e 3.6.2. | Não existe seção 3.6 no fonte atual. Foi inserido um diagrama geral na seção atual de arquitetura de software. | Falta o fonte completo da seção 3.6 para colocar o diagrama exatamente no local comentado e com o nível de detalhe do controle de movimento. |
| Seção 3.7, DDA em Ponto Fixo, e algoritmo DDA | Comentado diretamente. | O PDF revisado comenta que o algoritmo parece solto/código de implementação e sugere pseudocódigo ou apêndice. | Não existe seção 3.7 no fonte atual. | Falta o fonte da seção, o ambiente do algoritmo e o contexto textual que ligava o algoritmo ao DDA. Sem isso, não há local original para reformatar/mover o algoritmo. |
| Seção 3.8, Rampa Trapezoidal, e notação de piso | Comentado diretamente. | O PDF revisado comenta a notação de menor inteiro/floor e sugere colocar na lista de símbolos. | Não existe seção 3.8 no fonte atual. A Lista de Símbolos foi ampliada, mas não há equação de rampa no `.tex` atual. | Falta o fonte da equação e da explicação da rampa trapezoidal; a notação específica não pode ser corrigida no local original porque o trecho não existe. |
| Seção 3.9, Controle PI de Posição com Encoder | Referenciado no PDF, sem comentário direto encontrado. | O sumário do PDF enviado lista a seção 3.9. | Não existe seção equivalente no fonte atual. | Falta o fonte da seção; não foi possível avaliar correções porque não há anotação direta nem texto editável. |
| Seção 3.10, Fila de Movimentos e Segmentação | Referenciado no PDF, sem comentário direto encontrado. | O sumário do PDF enviado lista a seção 3.10. | Não existe seção equivalente no fonte atual. | Falta o fonte da seção; o conteúdo só aparece no PDF renderizado. |
| Seção 3.11, Mapeamento para o TMC5160 | Referenciado no PDF, sem comentário direto encontrado. | O sumário do PDF enviado lista a seção 3.11 e subseções 3.11.1 e 3.11.2. | O fonte atual tem uma seção sobre TMC5160 na fundamentação e uma seção de registros, mas não a seção metodológica 3.11. | Falta o fonte metodológico que explicava geração STEP/DIR e MicroPlyer no Capítulo 3. |
| Seção 3.12, Parâmetros-chave do Projeto | Referenciado no PDF, sem comentário direto encontrado. | O sumário do PDF enviado lista a seção 3.12. | Não existe seção equivalente no fonte atual. | Falta a tabela/lista de parâmetros do projeto usada na versão enviada. |
| Seção 3.13, Coordenação Multi-eixos | Referenciado no PDF, sem comentário direto encontrado. | O sumário do PDF enviado lista a seção 3.13. | Não existe seção equivalente no fonte atual. | Falta o texto completo de coordenação multi-eixos e seus algoritmos/expressões. |
| Seção 3.14, Simulador Interativo em Python | Referenciado no PDF, sem comentário direto encontrado. | O sumário do PDF enviado lista a seção 3.14 e a lista de figuras inclui a Figura 3.12 do simulador. | Não existe seção equivalente no fonte atual e não encontrei imagem do simulador. | Falta o fonte da seção, a descrição do simulador e o asset da Figura 3.12. |
| Seção 3.15, Boas práticas e Diagnóstico | Referenciado no PDF, sem comentário direto encontrado. | O sumário do PDF enviado lista a seção 3.15. | Não existe seção equivalente no fonte atual. | Falta o fonte da seção com recomendações de diagnóstico, temporização e TMC5160. |
| Figuras 3.5 a 3.12 | Algumas comentadas diretamente, outras apenas referenciadas. | Figura 3.5 tem comentários sobre "Que Raspberry é essa?" e protoboard/folgas; Figura 3.12 aparece na lista de figuras. As demais aparecem na lista de figuras do PDF enviado. | Não há arquivos de imagem correspondentes em `tcc/src`; o único asset encontrado lá é `Cap00/IC.jpg`. | Faltam os arquivos das figuras e os comandos `\includegraphics`. Corrigi textualmente Raspberry/protoboard na seção de hardware atual, mas as figuras originais não foram recuperadas. |
| Seção 4.3, Comparação entre simulação e experimento com carga | Referenciado no PDF, sem comentário direto encontrado. | O sumário do PDF enviado lista essa seção; o corpo do PDF traz o texto e a Figura 4.1. | No fonte atual, a seção 4.3 existe com outro tema: `Integração com a Raspberry Pi`. | Falta o fonte da seção experimental da versão enviada. Este é um caso de divergência real entre PDF enviado e `.tex` atual, não apenas uma anotação não aplicada. |
| Figura 4.1, comparação entre velocidade medida e simulada | Referenciado no PDF, sem comentário direto encontrado. | A lista de figuras do PDF enviado e a seção 4.3 referenciam a Figura 4.1. | Não encontrei arquivo de imagem correspondente no repositório. | Falta o asset da figura e o trecho LaTeX que a inseria. |

## O que dá para completar a partir do que já foi escrito

Nesta etapa, verifiquei também se o conteúdo ausente poderia ser buscado
em outras branches do Git. Não encontrei essas seções como LaTeX em
branches locais ou remotas. A fonte recuperável, portanto, é o próprio
PDF enviado. Como o PDF contém texto pesquisável e imagens embutidas, dá
para reconstruir parte relevante do material.

| Item | Dá para completar? | Fonte de recuperação | Observação de fidelidade |
|---|---|---|---|
| Seção 2.8, Sincronização Cruzada de Eixos | Sim, com boa fidelidade. | Texto do PDF enviado via `pdftotext`. | Precisa revisar acentos, símbolos e citações depois da conversão. |
| Seção 2.11, NEMA 23, passo 0,9° e seleção de `IRMS` | Sim, com boa fidelidade. | Texto do PDF enviado via `pdftotext`. | Como não houve comentário direto do Glauber, entraria como restauração de divergência entre PDF e fonte. |
| Seção 3.2.1, Fluxo do comando `cnc-cli` | Sim para o texto; parcialmente para a figura. | Texto do PDF enviado; figura pode ser recriada em LaTeX simples ou recuperada por crop do PDF. | A figura original não apareceu como arquivo separado no repositório. Dá para refazer o diagrama, mas não recuperar o código original dele. |
| Comentário do `frameId` em `MOVE_QUEUE_STATUS` | Sim. | Código C do protocolo e texto do PDF. | A parte técnica já foi confirmada no código; falta inserir a explicação no trecho restaurado do `cnc-cli`. |
| Seção 3.4, Identificação experimental FOPDT | Sim, com boa fidelidade textual. | Texto do PDF enviado via `pdftotext`. | As equações precisam ser reescritas manualmente em LaTeX para evitar símbolos quebrados. |
| Figuras 3.2, 3.3 e 3.4 | Sim como imagens recuperadas; não como script/origem dos gráficos. | `pdfimages` mostra imagens embutidas no PDF enviado. | Dá para extrair e reinserir como figuras, mas não encontrei dados CSV ou scripts que geraram os gráficos. |
| Seção 3.6, Arquitetura de Controle de Movimento | Sim, com boa fidelidade textual. | Texto do PDF enviado via `pdftotext`. | O comentário do diagrama estilo Simulink pode ser atendido com um diagrama novo ou crop/recriação. |
| Seção 3.7, DDA em Ponto Fixo | Sim, com boa fidelidade textual. | Texto do PDF enviado via `pdftotext`. | O algoritmo precisa ser refeito como pseudocódigo LaTeX, porque o PDF só fornece a renderização. |
| Seção 3.8, Rampa Trapezoidal | Sim, com boa fidelidade textual. | Texto do PDF enviado via `pdftotext`. | A notação de piso/floor deve ser corrigida manualmente e adicionada à Lista de Símbolos se mantida. |
| Seções 3.9 a 3.13 | Sim, com boa fidelidade textual. | Texto do PDF enviado via `pdftotext`. | Equações, tabelas e algoritmos exigem reescrita manual em LaTeX. |
| Seção 3.14, Simulador Interativo em Python | Sim para o texto; sim para a imagem como figura recuperada. | Texto do PDF enviado e imagem embutida extraível. | Não encontrei o script `interactive_sim.py` ou fonte equivalente; o texto pode ser restaurado, mas o código do simulador não. |
| Seção 3.15, Boas práticas e Diagnóstico | Sim, com boa fidelidade textual. | Texto do PDF enviado via `pdftotext`. | Precisa revisar citações e termos técnicos após conversão. |
| Figuras 3.5 a 3.12 | Sim como imagens recuperadas do PDF. | `pdfimages` lista imagens embutidas nas páginas correspondentes. | Dá para extrair imagens finais; não encontrei modelos 3D, arquivos de plotagem ou código original que gerou cada figura. |
| Seção 4.3, Comparação entre simulação e experimento com carga | Sim, com boa fidelidade textual. | Texto do PDF enviado via `pdftotext`. | Entraria substituindo ou deslocando a atual seção 4.3 do fonte, que hoje fala de Raspberry Pi. |
| Figura 4.1 | Sim como imagem recuperada do PDF. | `pdfimages` lista uma imagem na página correspondente aos resultados. | Dá para reinserir como figura final; não encontrei dados brutos nem script que gerou o gráfico. |

Resumo prático: dá para reconstruir quase todo o conteúdo ausente porque
ele está escrito no PDF enviado. O que não dá para recuperar como original
são os arquivos-fonte que geraram figuras, gráficos, diagramas e scripts
experimentais. Para esses itens, a recuperação viável é inserir a imagem
final extraída do PDF ou recriar um diagrama equivalente em LaTeX.

## Busca por PDF idêntico em outras branches

Arquivo usado como referência:
`tcc/versao_enviada/TCC_Valdir_Dias_Silva_Junior.pdf`.

- SHA-256: `CE0373B1EAC1FACFE88688C7D2C93B31DBD1EF00851B71E8D4C1E5B5F664DED0`.
- Tamanho: `8545248` bytes.
- Páginas: 60.

Verifiquei primeiro os PDFs presentes nas pontas das branches locais e remotas
já disponíveis no repositório. Foram encontrados 167 caminhos de PDF nas refs
analisadas, mas nenhum tinha o mesmo tamanho do PDF enviado e nenhum tinha o
mesmo hash.

Depois verifiquei todos os objetos PDF alcançáveis no histórico Git local,
não apenas as pontas das branches. Foram encontrados 12 blobs PDF únicos no
histórico, também sem nenhum arquivo com o mesmo tamanho ou hash do PDF
enviado.

Também comparei o texto extraído do PDF enviado com os `tcc/src/main.pdf`
históricos encontrados no repositório. O PDF enviado tem 60 páginas; os
`main.pdf` históricos encontrados tinham entre 25 e 35 páginas. Nenhum deles
foi idêntico no texto extraído nem no texto normalizado.

Conclusão: nas branches e no histórico Git atualmente disponíveis localmente,
não encontrei um PDF idêntico ao arquivo enviado. Portanto, não apareceu uma
branch correta para copiar diretamente a partir de identidade binária ou
textual do PDF. Se essa branch existir, ela não está presente/fetchada neste
clone ou o PDF enviado nunca foi commitado nela.
