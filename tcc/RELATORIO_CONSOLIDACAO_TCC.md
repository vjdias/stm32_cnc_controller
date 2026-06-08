# Relatorio de consolidacao do TCC

Data da consolidacao: 2026-06-06

Branch de trabalho: `codex/ultima-correcao-tcc`

PDF usado como fonte de recuperacao:
`tcc/versao_enviada/TCC_Valdir_Dias_Silva_Junior.pdf`

## Objetivo

Consolidar os fontes LaTeX para que o TCC fique completo, congruente e
organizado apos a recuperacao do conteudo textual do PDF entregue e a
aplicacao das correcoes indicadas na revisao do Prof. Glauber.

O foco desta etapa foi fechar os pontos que ainda estavam incompletos:
figuras tecnicas ausentes, chamadas de figuras sem material visual,
explicacoes de hardware/controle que estavam resumidas demais e uma
incongruencia de temporizacao no Capitulo 4.

## Historico verificado

Foi verificado o historico alcancavel da branch `codex/ultima-correcao-tcc`
em busca dos arquivos de imagem originais. Eles nao apareceram como assets
versionados separados. No historico do `tcc`, os unicos arquivos de imagem
ou PDF tecnicamente relevantes encontrados foram:

| Arquivo | Situacao |
|---|---|
| `tcc/src/Cap00/IC.jpg` | Imagem institucional dos elementos pre-textuais. |
| `tcc/src/main.pdf` | PDF compilado historico, com diferentes versoes. |

Conclusao: as figuras tecnicas nao existiam nessa branch como arquivos
editaveis ou exportados separadamente. A fonte recuperavel era o proprio
PDF entregue.

## Estrategia aplicada

1. Extrair as imagens embutidas no PDF entregue.
2. Renomear os arquivos com nomes semanticos.
3. Versionar as imagens usadas em `tcc/src/figures/pdf_recovered/`.
4. Inserir as figuras no ponto correto do LaTeX, com `\label` e chamadas
   textuais por `\ref`.
5. Melhorar a explicacao dos trechos em que a figura e essencial para
   entendimento.
6. Corrigir incongruencias tecnicas encontradas durante a integracao.
7. Registrar as mudancas neste documento para avaliacao.

## Figuras recuperadas e inseridas

| Tema | Arquivos inseridos | Local no LaTeX |
|---|---|---|
| Validacao FOPDT eixo X | `fopdt_x_1_4.png`, `fopdt_x_1_16.png`, `fopdt_x_1_256.png` | `Cap03/IdentificacaoFopdt.tex`, `fig:fopdt-x` |
| Validacao FOPDT eixo Y | `fopdt_y_1_4.png`, `fopdt_y_1_16.png`, `fopdt_y_1_256.png` | `Cap03/IdentificacaoFopdt.tex`, `fig:fopdt-y` |
| Validacao FOPDT eixo Z | `fopdt_z_1_4.png`, `fopdt_z_1_16.png`, `fopdt_z_1_256.png` | `Cap03/IdentificacaoFopdt.tex`, `fig:fopdt-z` |
| Modelo 3D do controlador | `hardware_controller_perspective.png`, `hardware_controller_top.png` | `Cap03/ArquiteturaHardware.tex`, `fig:hardware-controlador-cnc` |
| Suporte do encoder | `tmcs28_support_cad.png` | `Cap03/ArquiteturaHardware.tex`, `fig:tmcs28-suporte-cad` |
| Motores com encoders | `prototype_motors_tmcs28.jpg` | `Cap03/ArquiteturaHardware.tex`, `fig:motores-tmcs28-prototipo` |
| Prototipo em bancada | `prototype_bench_complete.jpg` | `Cap03/ArquiteturaHardware.tex`, `fig:prototipo-bancada` |
| Temporizacao STEP/TMC5160 | `step_timing_tmc5160.png` | `Cap03/ControleMovimento.tex`, `fig:step-timing-tmc5160` |
| Simulacao DDA | `dda_xy_simulation.png` | `Cap03/ControleMovimento.tex`, `fig:dda-xy-simulation` |
| Rampa trapezoidal | `trapezoidal_ramp_simulation.png` | `Cap03/ControleMovimento.tex`, `fig:rampa-trapezoidal-simulada` |
| Simulador Python | `python_interactive_simulator.png` | `Cap03/ControleMovimento.tex`, `fig:simulador-python` |
| Simulacao versus experimento | `simulation_experiment_load_comparison.png` | `Cap04/Capitulo4.tex`, `fig:simulacao-experimento-carga` |

Foi mantida uma folha de revisao visual em
`tcc/revisao_consolidacao_assets/pdf_recovered_contact_sheet.jpg`, junto
com o metadado de extracao em
`tcc/revisao_consolidacao_assets/pdf_recovered_images.json`.

## Mudancas textuais relevantes

| Arquivo | Mudanca |
|---|---|
| `tcc/src/Cap03/IdentificacaoFopdt.tex` | A nota de que os graficos nao estavam disponiveis foi removida. Entraram os tres conjuntos de graficos FOPDT, agrupados por eixo e microstepping, com uma interpretacao sintetica dos resultados. |
| `tcc/src/Cap03/ArquiteturaHardware.tex` | A secao passou a incluir as imagens de CAD, suporte do encoder, motores com encoders e prototipo em bancada. Tambem foram reforcadas as explicacoes sobre Raspberry Pi 3 A+, protoboard, separacao fisica entre logica e potencia, e rotina de seguranca baseada em `GSTAT`, `DRV_STATUS`, `IHOLD`, `IRUN`, `TOFF` e `FREEWHEEL`. |
| `tcc/src/Cap03/ControleMovimento.tex` | Foram inseridas as figuras de temporizacao `STEP`, operacao do DDA, rampa trapezoidal e simulador Python. O texto foi ajustado para que cada imagem complemente uma explicacao tecnica especifica. |
| `tcc/src/Cap04/Capitulo4.tex` | Foi inserida a figura de comparacao entre velocidade medida e simulada. Tambem foi corrigida a temporizacao do pulso `STEP`: o texto agora usa `20 us` em nivel alto e `20 us` em nivel baixo, coerente com `TIM6 @ 50 kHz` e com a tabela do TMC5160. |
| `tcc/src/Cap02/RegTables.tex` | As tabelas de registros passaram de posicionamento rigido `[H]` para `[htbp]`, permitindo melhor organizacao de pagina e removendo o `overfull vbox` grande visto na compilacao inicial. |

## Congruencia tecnica

Durante a consolidacao, foram mantidos os seguintes alinhamentos com o
firmware atual:

| Topico | Estado consolidado |
|---|---|
| SPI com Raspberry Pi | Texto usa `SPI2` em modo escravo com DMA circular. |
| Geracao de passos | Texto usa `TIM6 @ 50 kHz`, com tick de `20 us`. |
| Controle/rampa/telemetria | Texto usa `TIM7 @ 1 kHz`. |
| Encoders | Texto usa `TIM3`, `TIM5` e `LPTIM1` em modo encoder. |
| Logs/telemetria | Texto usa `USART1`. |
| Protocolo `MOVE_QUEUE_STATUS` | Texto preserva a explicacao de `frameId` em `raw[2]` na requisicao e na resposta. |

## Validacoes executadas

| Validacao | Resultado |
|---|---|
| Caminhos de `\includegraphics` | 0 imagens faltantes. |
| `\label` duplicado | 0 duplicidades. |
| `\ref`, `\autoref` e `\eqref` quebrados | 0 referencias quebradas. |
| Placeholders (`TODO`, `FIXME`, `Figura??`, `Conteudo pendente`) | Nenhum encontrado em `tcc/src`. |
| Referencias antigas a `SPI1` e `TIM2` no texto do TCC | Nenhuma encontrada em `tcc/src`. |
| Menções remanescentes de assets indisponiveis | Nenhuma encontrada em `tcc/src`. |
| Compilacao com MiKTeX `pdflatex` + `bibtex` + passadas finais de `pdflatex` | Sucesso. PDF gerado em `tcc/src/main.pdf` com 63 paginas. |
| Log final de compilacao | Sem erro LaTeX, sem citacao indefinida e sem referencia indefinida. |

## Limitacoes

O `latexmk` do MiKTeX nao executou porque o MiKTeX nao encontrou o
interpretador Perl exigido por esse script. A compilacao foi feita
diretamente com os executaveis do MiKTeX encontrados no Windows:
`pdflatex`, `bibtex`, `pdflatex` e `pdflatex`.

O log final ainda registra avisos de diagramacao esperados em documentos
LaTeX, como `underfull hbox`, pequenos `overfull hbox`, avisos de
`hyperref` em titulos com matematica e duplicidade de destinos nos
elementos pre-textuais. Esses avisos nao impediram a geracao do PDF nem
indicaram referencias quebradas.

As figuras foram recuperadas como imagens finais embutidas no PDF
entregue. Isso resolve a completude visual do TCC, mas nao recupera os
arquivos-fonte originais dos graficos, modelos CAD, scripts de plotagem
ou dados experimentais.

## Arquivos principais alterados nesta etapa

| Caminho | Tipo de alteracao |
|---|---|
| `tcc/src/Cap03/IdentificacaoFopdt.tex` | Insercao de figuras FOPDT e revisao da interpretacao. |
| `tcc/src/Cap03/ArquiteturaHardware.tex` | Insercao de figuras de hardware e ampliacao da explicacao tecnica. |
| `tcc/src/Cap03/ControleMovimento.tex` | Insercao de figuras de controle, DDA, rampa e simulador. |
| `tcc/src/Cap04/Capitulo4.tex` | Insercao da figura de resultados e correcao de temporizacao. |
| `tcc/src/Cap02/RegTables.tex` | Ajuste de posicionamento das tabelas de registros. |
| `tcc/src/figures/pdf_recovered/` | Pasta com as imagens tecnicas recuperadas do PDF entregue. |
| `tcc/revisao_consolidacao_assets/` | Pasta auxiliar com folha de revisao visual e metadados de extracao. |
