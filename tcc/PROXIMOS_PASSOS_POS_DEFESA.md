# Proximos passos pos-defesa, colacao de grau e diploma

Data: 2026-06-06

## Situacao do TCC

O TCC tecnico foi consolidado e o PDF final foi recompilado em
`tcc/src/main.pdf`. Depois da analise dos documentos administrativos em
`Downloads`, foram aplicados estes ajustes no projeto:

- A data oficial do TCC passou para `26 de Novembro de 2025`, conforme o
  formulario de avaliacao assinado.
- A banca foi mantida com Icaro, Glauber e Andressa, agora validada por
  documento local assinado.
- A titulacao da Andressa foi ajustada para `M.e.`, seguindo o formulario
  de avaliacao.
- A pagina placeholder de ata/aprovacao foi substituida pelo formulario de
  avaliacao assinado arquivado no repositorio.

## Dados confirmados pelo formulario de avaliacao

| Campo | Valor |
|---|---|
| Aluno | Valdir Dias Silva Junior |
| Matricula | 14114090 |
| Titulo | Controle deterministico para CNC baseado em STM32L475 com DDA de alta frequencia |
| Data da defesa | 26/11/2025 |
| Nota | 9 (Nove) |
| Orientador | Prof. Dr. Icaro Bezerra Queiroz de Araujo |
| Banca | Prof. Dr. Glauber Rodrigues Leite |
| Banca | M.e. Andressa Martins Oliveira |

## Documentos arquivados no repositorio

Os documentos do aluno e o fluxograma administrativo foram copiados para:

`tcc/documentos_pos_defesa/fontes_usuario/`

| Arquivo | Observacao |
|---|---|
| `avaliacao_tcc_valdir_dias_assinado.pdf` | Formulario de avaliacao do TCC. |
| `avaliacao_tcc_valdir_dias_assinado_assinado.pdf` | Versao assinada/assinada do formulario. |
| `avaliacao_tcc_valdir_dias_assinado_assinado_copia1.pdf` | Copia identica da versao assinada/assinada. |
| `avaliacao_tcc_valdir_dias_assinado_assinado_assinado_copia1.pdf` | Versao usada no TCC como pagina de ata/aprovacao. |
| `fluxograma_colacao_grau_expedicao_diploma.pdf` | Fluxograma da colacao de grau por ato administrativo e expedicao do diploma. |
| `nada_consta_biblioteca_valdir_dias.pdf` | Declaracao de nada consta da biblioteca, emitida em 01/06/2026. |

Os arquivos de exemplo de terceiros foram consultados apenas como referencia
de estrutura. Eles nao foram copiados para o repositorio para evitar manter
documentos pessoais de outra pessoa dentro deste projeto.

## O que o fluxograma da UFAL indica

Pelo fluxograma de colacao de grau por ato administrativo e expedicao de
diploma de graduacao, a sequencia pratica e:

1. O aluno junta os documentos necessarios e submete a solicitacao no SIGAA.
2. O DRCA analisa a documentacao.
3. Se a resposta for `VALIDADO`, o DRCA disponibiliza o certificado de
   conclusao de curso no proprio SIGAA em ate 15 dias uteis.
4. Se a resposta for `NEGADO`, o aluno deve abrir nova solicitacao anexando
   os documentos exigidos ou corrigidos.
5. O DRCA encaminha ao Gabinete do Reitor a lista de alunos para outorga de
   grau por ato administrativo.
6. O Gabinete do Reitor publica a lista em boletim.
7. O DRCA inicia a expedicao do diploma, com prazo de 120 dias uteis conforme
   a Portaria n. 1095/2018.
8. O aluno acessa o SIGAA e baixa o diploma digital quando ele estiver
   disponivel.

Links indicados no fluxograma:

- SIGAA: `https://sigaa.sig.ufal.br/sigaa/logar.do?dispatch=logOff`
- Boletins SIPAC: `https://sipac.sig.ufal.br/public/visualizaBoletins.do?aba=p-boletins&publico=true`

## Checklist imediato

1. Conferir visualmente `tcc/src/main.pdf`, principalmente capa, folha de
   rosto, ficha catalografica, pagina de avaliacao/ata, resumo, abstract,
   listas, sumario e referencias.
2. Solicitar ou localizar a ficha catalografica oficial do seu TCC. A ficha
   da Rafaela serve apenas como exemplo de formato; nao deve ser adaptada
   manualmente como se fosse oficial.
3. Substituir o placeholder de `tcc/src/Cap00/FichaCatalografica.tex` pela
   ficha catalografica oficial quando ela for emitida.
4. Preencher e assinar o Termo de Autorizacao do Repositorio Institucional
   da UFAL.
5. Separar o PDF final do TCC corrigido, o formulario de avaliacao assinado,
   o nada consta da biblioteca, o termo de autorizacao e a ficha
   catalografica oficial.
6. Submeter a solicitacao no SIGAA conforme o fluxo de colacao por ato
   administrativo.
7. Acompanhar a resposta do DRCA: se validado, aguardar certificado em ate
   15 dias uteis; se negado, corrigir exatamente os documentos apontados.
8. Apos a publicacao da outorga, acompanhar o diploma digital no SIGAA. O
   prazo indicado para expedicao e de 120 dias uteis.

## Dados para preencher o Termo de Autorizacao RI/UFAL

Use estes dados como base, conferindo no formulario oficial antes de enviar:

| Campo | Preenchimento sugerido |
|---|---|
| Tipo do trabalho | Trabalho de Conclusao de Curso (graduacao) |
| 1o Autor | Valdir Dias Silva Junior |
| Orientador | Icaro Bezerra Queiroz de Araujo |
| 1o Membro da banca | Icaro Bezerra Queiroz de Araujo |
| 2o Membro da banca | Glauber Rodrigues Leite |
| 3o Membro da banca | Andressa Martins Oliveira |
| Data da defesa | 26/11/2025 |
| Titulo original | Controle deterministico para CNC baseado em STM32L475 com DDA de alta frequencia |
| Palavras-chave | controle CNC; STM32L475; DDA; controlador PID; SPI deterministico |
| Titulo em outro idioma | Deterministic CNC control based on STM32L475 with high-frequency DDA |
| Keywords | CNC control; STM32L475; DDA; PID controller; deterministic SPI |
| Curso/unidade | Engenharia de Computacao / Instituto de Computacao |
| Acesso ao documento | Total, salvo se houver motivo formal para embargo |
| Agencia de fomento | Preencher como inexistente se nao houve bolsa ou financiamento |

Campos que precisam ser preenchidos pelo aluno e nao devem ser inferidos:

- CPF, endereco, telefone e e-mail de contato.
- Links Lattes exigidos pelo formulario, especialmente do orientador.
- Data de entrega do documento ao repositorio.
- Assinaturas exigidas no fluxo oficial.

## Pendencias reais

- A ficha catalografica oficial do Valdir ainda nao foi encontrada entre os
  arquivos enviados; existe apenas um exemplo de outra aluna.
- O Termo de Autorizacao do RI/UFAL enviado como exemplo nao contem os dados
  completos do seu TCC na extracao textual; ele deve ser preenchido com os
  dados acima e assinado.
- O nada consta da biblioteca esta disponivel e indica ausencia de pendencias
  em 01/06/2026.
- A nota 9 foi confirmada pelos formularios de avaliacao assinados locais,
  nao pelo Gmail.
