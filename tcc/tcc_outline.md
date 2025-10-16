# Estrutura Sugerida para o TCC

## Summary

- Relacionei os elementos pré-textuais (capa, dedicatória, agradecimentos) do exemplo de TCC às suas necessidades para guiar a montagem do *front matter* do trabalho.
- Estruturei os tópicos da Introdução e Fundamentação Teórica conectando-os aos aspectos centrais do projeto, como hardware STM32L475, DDA de 50 kHz, controle PID a 1 kHz e comunicação SPI/USART.
- Propus como documentar Metodologia e Resultados utilizando o roteiro de configuração e o estudo do *pipeline* SPI, deixando claras as evidências práticas a serem discutidas no TCC.

## Testing

- ⚠️ Não aplicável (tarefa informativa).

## Orientação detalhada (em português)

### Elementos pré-textuais inspirados no exemplo
O TCC de referência apresenta capa institucional, folha de aprovação, dedicatória e agradecimentos logo no início. Reproduzir essa sequência ajuda a manter o padrão acadêmico da banca e já encaminha a diagramação do seu trabalho.

### Introdução
- **Contexto e motivação** – Descreva a necessidade de um controlador CNC determinístico embarcado, apresentando a plataforma STM32L475 (kit B-L475E-IOT01A), os encoders de alta resolução, a meta de sincronismo entre eixos e a integração com Raspberry Pi.
- **Objetivos específicos** – Liste metas como gerar passos DDA a 50 kHz (TIM6), fechar o laço PI/PID a 1 kHz (TIM7) e manter logs via USART1, ressaltando a importância da baixa latência e previsibilidade.
- **Organização do trabalho** – Antecipe que a Fundamentação aborda timers, DDA e PID; a Metodologia explica o roteiro de *bring-up*; os Resultados apresentam evidências do *pipeline* SPI e serviços.

### Fundamentação Teórica
- **Definição de CNC** – Apresente máquinas de Controle Numérico Computadorizado (CNC) como sistemas capazes de interpretar instruções digitais (G-code, protocolos proprietários) e converter esses comandos em movimentos coordenados de múltiplos eixos por meio de motores e atuadores, garantindo repetibilidade e precisão industriais.
- **Plataforma STM32 e temporizadores** – Detalhe a arquitetura de clock (80 MHz, APB1/APB2) e as fórmulas de PSC/ARR para gerar as bases de tempo exigidas pelo DDA e pelo controle.
- **Definição de DDA e utilidade** – Explique o Digital Differential Analyzer como um integrador digital que discretiza equações diferenciais para gerar trajetórias ponto a ponto; destaque que, no projeto, ele produz pulsos STEP/DIR/EN a 50 kHz via TIM6 para alimentar os drivers TMC5160 com determinismo temporal.
- **Definição de PID e utilidade** – Apresente o controlador Proporcional-Integral-Derivativo (PID) como técnica de controle retroalimentado que combina correção proporcional, integral e derivativa para reduzir erro e oscilações; relacione com o laço de 1 kHz implementado no projeto para acompanhar a posição lida pelos encoders e ajustar o movimento em tempo real.
- **Controle PI/PID em 1 kHz** – Aborde a leitura incremental dos encoders (TIM2/TIM5/TIM3) e o laço de 1 ms, mencionando a necessidade de cálculo modular para tratar *wrap* e garantir precisão.
- **Comunicação SPI/USART** – Discuta o modo escravo com DMA circular, prioridades de NVIC e o papel do console USB para observabilidade, alinhando com a literatura sobre protocolos determinísticos.
- **Integração DDA + PID para sincronismo multieixo** – Explique que o DDA atua como gerador determinístico de setpoints incrementais (contagem de passos ou posição alvo) para cada eixo, enquanto os PIDs fecham o laço de posição/velocidade comparando o setpoint com a leitura dos encoders. Descreva que todos os eixos compartilham a mesma base de tempo do DDA, o que permite pausar o avanço global quando um PID detecta erro sustentado acima de limites (por exemplo, saturação da ação integral ou disparo de monitor de esforço). Ao acionar essa condição de *feed hold*, o firmware inibe os pulsos STEP (ou reduz o *feedrate*) de todos os eixos, preservando o sincronismo e evitando deformações quando um eixo é bloqueado por força externa.

### Metodologia
- **Roteiro incremental de *bring-up*** – Utilize o checklist que parte da fundação de clock, passa por interrupções de segurança, configuração dos timers DDA/encoders/controle e encerra com SPI e logging; isso evidencia o planejamento experimental adotado.
- **Arquitetura de software** – Descreva a divisão entre código CubeMX (Core) e camadas autorais (App/Services/Protocol), mostrando como cada módulo participa do ciclo de controle e da comunicação.
- **Procedimentos de teste** – Cite medições de frequência via osciloscópio, validações de contagem de encoder, aferição de jitter no loop de 1 kHz e testes de integridade SPI conforme listado no roteiro.

### Resultados
- **Desempenho dos serviços principais** – Documente a geração de passos estável, o controle PI/PID executando em 1 kHz e o comportamento do homing/safety descritos no README.
- **Análise do *pipeline* SPI** – Apresente o fluxo de polls, a janela de *defer* e os três ciclos necessários para o ACK de LED, usando o detalhamento do documento específico como base para gráficos ou tabelas.
- **Interação com o cliente Raspberry Pi** – Explique como o `cnc_spi_client.py` decide repetir polls, relacionando timeouts e políticas de rearmamento do DMA.

### Conclusão
- **Síntese das contribuições** – Reforce que o sistema entrega geração determinística de movimento, controle fechado em hardware dedicado e protocolo SPI robusto, integrando os requisitos iniciais.
- **Limitações e trabalhos futuros** – Aponte possíveis otimizações do *pipeline* SPI (como reinicialização imediata do DMA ou reutilização de buffer) e ajustes de prescaler para serviços auxiliares, conforme sugerido na análise técnica.

Essa estrutura, alinhada ao exemplo fornecido, permite transformar o material já documentado no repositório em capítulos coerentes de um TCC, mantendo a profundidade técnica e destacando as evidências de validação do controlador CNC.
