; Exemplo simples para testar raspberry_spi.cnc_cli com velocidades baixas
; Observação: este CLI interpreta F como mm/s e usa mm como unidade.

G90            ; posicionamento absoluto
G1 X0 Y0 Z0 F1 ; garante origem corrente (movimento nulo)

; Quadrado 5x5 mm no plano XY em baixa velocidade (1 mm/s)
G1 X5.0 Y0.0 F1.0
G1 X5.0 Y5.0 F1.0
G1 X0.0 Y5.0 F1.0
G1 X0.0 Y0.0 F1.0

; Pequeno movimento em Z com velocidade ainda menor (0.5 mm/s)
G1 Z1.0 F0.5
G1 Z0.0 F0.5

; Fim
