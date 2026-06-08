# Relatorio de validacao de banca e orientacao

Data: 2026-06-06

## Pedido

Validar, a partir do conjunto de e-mails indicado pelo link do Gmail e de
fontes disponiveis, os nomes e cargos relacionados ao TCC:

- Andressa e Glauber como banca avaliadora.
- Icaro como orientador.
- Icaro tambem participando da banca e da nota.

Link informado:

`https://mail.google.com/mail/u/0/?tab=rm&ogbl#search/tcc/FMfcgzQcqtcPbbSbtFwcMQqnGrqPcBCR`

## Acesso ao Gmail

Nao foi possivel baixar o conteudo do Gmail diretamente por este ambiente.
O link redirecionou para a tela de login do Google, e nao ha conector de
Gmail autenticado disponivel nesta sessao.

Por isso, nenhum e-mail privado foi baixado. A pasta foi criada para
registrar a tentativa, guardar fontes publicas de apoio e deixar claro o
que foi validado.

## Fontes locais recebidas depois

Depois da tentativa de acesso ao Gmail, foram analisados os PDFs locais
informados em `C:\Users\Valdir\Downloads`. Os documentos proprios do aluno
e o fluxograma administrativo foram copiados para:

`tcc/documentos_pos_defesa/fontes_usuario/`

| Arquivo arquivado | Uso |
|---|---|
| `avaliacao_tcc_valdir_dias_assinado.pdf` | Formulario de avaliacao do TCC. |
| `avaliacao_tcc_valdir_dias_assinado_assinado.pdf` | Formulario de avaliacao do TCC com assinatura adicional. |
| `avaliacao_tcc_valdir_dias_assinado_assinado_copia1.pdf` | Copia identica do formulario assinado/assinado. |
| `avaliacao_tcc_valdir_dias_assinado_assinado_assinado_copia1.pdf` | Versao mais completa pelo nome e pelos campos de assinatura do PDF; usada como ata/anexo no TCC. |
| `fluxograma_colacao_grau_expedicao_diploma.pdf` | Fonte para os proximos passos de colacao de grau e diploma. |
| `nada_consta_biblioteca_valdir_dias.pdf` | Declaracao de nada consta da biblioteca. |

Dados confirmados pelo formulario de avaliacao:

| Campo | Valor confirmado |
|---|---|
| Aluno | Valdir Dias Silva Junior |
| Matricula | 14114090 |
| Titulo | Controle deterministico para CNC baseado em STM32L475 com DDA de alta frequencia |
| Data da defesa | 26/11/2025 |
| Nota obtida | 9 (Nove) |
| Orientador | Prof. Dr. Icaro Bezerra Queiroz de Araujo |
| Banca | Prof. Dr. Glauber Rodrigues Leite |
| Banca | M.e. Andressa Martins Oliveira |

## Fontes publicas usadas

| Fonte | Evidencia usada |
|---|---|
| Pagina institucional do IC/UFAL de Icaro Bezerra Queiroz de Araujo | Confirma nome completo, vinculo ao Instituto de Computacao da UFAL, titulo de doutor e cargo de Professor Adjunto. |
| PDF publico do repositorio UFAL `Controle PID aplicado a um sistema de geracao eletrico` | Usa uma banca de TCC recente com os mesmos nomes: Icaro Bezerra Queiroz de Araujo, Glauber Rodrigues Leite e Andressa Martins Oliveira. |
| Resultado de busca publico do repositorio UFAL | Indica explicitamente: Icaro como orientador e banca; Glauber Rodrigues Leite como `Prof. Dr., IC-UFAL`; Andressa Martins Oliveira como `MsC, IC-UFAL`. |

Arquivo publico baixado para esta pasta:

`fontes_publicas/tcc_referencia_banca_icaro_glauber_andressa.pdf`

## Resultado recomendado para o TCC

Com base no pedido e nas fontes publicas encontradas, a identificacao do
TCC foi ajustada para:

| Papel | Nome | Titulo/Afiliacao |
|---|---|---|
| Orientador | Icaro Bezerra Queiroz de Araujo | Prof. Dr. |
| Banca avaliadora | Icaro Bezerra Queiroz de Araujo | Prof. Dr., IC-UFAL |
| Banca avaliadora | Glauber Rodrigues Leite | Prof. Dr., IC-UFAL |
| Banca avaliadora | Andressa Martins Oliveira | M.e., IC-UFAL |
| Data de aprovacao/defesa | 26/11/2025 | Confirmada pelo formulario assinado |
| Nota | 9 (Nove) | Confirmada pelo formulario assinado |

## Arquivo alterado

`tcc/src/Identificacao.tex`

Alteracoes aplicadas:

- `\orientador{Ícaro Bezerra Queiroz de Araújo}` foi mantido.
- `\memberA` passou a ser `Ícaro Bezerra Queiroz de Araújo`.
- `\filiationA` passou a ser `Prof.\ Dr., IC-UFAL`.
- `\memberB` passou a ser `Glauber Rodrigues Leite`.
- `\filiationB` passou a ser `Prof.\ Dr., IC-UFAL`.
- `\memberC` passou a ser `Andressa Martins Oliveira`.
- `\filiationC` passou a ser `M.e., IC-UFAL`.
- A data do TCC foi ajustada para `26 de Novembro de 2025`.
- A pagina placeholder de ata/aprovacao foi substituida pelo formulario de avaliacao assinado arquivado no repositorio.

## Observacoes

O TCC agora registra Icaro duas vezes: como orientador e como membro da
banca. Isso foi feito porque o pedido informa que ele participou tambem
da banca e da nota, e ha exemplo publico recente da UFAL com essa mesma
estrutura.

As notas em si nao foram validadas pelo Gmail, porque o conteudo privado
nao ficou acessivel sem autenticacao. A nota foi validada posteriormente
pelos formularios de avaliacao assinados disponiveis localmente.
