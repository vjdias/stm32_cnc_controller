import graphviz

def generate_flowchart():
    """
    Generates a flowchart of the cnc-cli command.
    """
    dot = graphviz.Digraph('cnc_cli_flow', comment='Fluxo do comando cnc-cli')
    dot.attr('node', shape='box', style='rounded,filled', fillcolor='lightblue', fontname='helvetica')
    dot.attr('edge', fontname='helvetica')
    dot.attr(rankdir='TB')

    # Nodes
    dot.node('start', 'Ler application.cfg
e arquivo de sequência', shape='trapezium')
    dot.node('validate', 'Validar limites (PID, vel, TMC)
conforme cfg.limits')
    dot.node('ok', 'Válido?', shape='diamond', style='filled', fillcolor='lightcyan')
    dot.node('abort', 'Abortar execução
Log de motivos')
    dot.node('connect', 'Abrir SPI
STM32 (dev 0)')
    dot.node('ledblink', 'LED pisca (execução)')
    dot.node('steps', 'Iterar passos da sequência:')
    dot.node('tmc', 'tmc: aplica patch
(TOFF/HSTRT/HEND/TBL/...)' ) 
    dot.node('pid', 'pid: atualizar ganhos
default (x,y,z)')
    dot.node('move', 'move: enfileirar REQ_MOVE_QUEUE_ADD
(pos/vel/dir, PID atual)
frameId interno incremental')
    dot.node('startmv', 'REQ_START_MOVE')
    dot.node('monitor', 'Loop de monitoramento:
QueueStatus + TMC DRV_STATUS/GSTAT')
    dot.node('tmcerr', 'TMC erro?
(OT/OTPW/DRV_ERR)', shape='diamond', style='filled', fillcolor='lightcyan')
    dot.node('done', 'STM32 concluiu?', shape='diamond', style='filled', fillcolor='lightcyan')
    dot.node('tmcstop', 'Segurança: TOFF=0, IHOLD/IRUN=0
LED aceso (falha)')
    dot.node('finish', 'LED apagado
TOFF=0 (pós-execução)')

    # Edges
    dot.edge('start', 'validate')
    dot.edge('validate', 'ok')
    dot.edge('ok', 'abort', label='não')
    dot.edge('ok', 'connect', label='sim')
    dot.edge('connect', 'ledblink')
    dot.edge('ledblink', 'steps')

    with dot.subgraph() as s:
        s.attr(rank='same')
        s.edge('steps', 'tmc')
        s.edge('steps', 'pid')

    dot.edge('tmc', 'move')
    dot.edge('pid', 'move')
    dot.edge('move', 'startmv')
    dot.edge('startmv', 'monitor')
    dot.edge('monitor', 'tmcerr')
    dot.edge('monitor', 'done')
    dot.edge('tmcerr', 'tmcstop', label='sim')
    dot.edge('done', 'finish', label='sim')

    # Loops
    dot.edge('tmcstop', 'monitor')
    dot.edge('done', 'monitor', label='não')
    
    dot.render('cnc_cli_flow', format='png', view=False, cleanup=True)
    print("Flowchart generated as cnc_cli_flow.png")

if __name__ == '__main__':
    generate_flowchart()
