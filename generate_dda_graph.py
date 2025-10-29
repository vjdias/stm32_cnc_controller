
import matplotlib.pyplot as plt
import numpy as np
import os

# --- Função para o algoritmo DDA ---
def dda_line(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1

    steps = max(abs(dx), abs(dy))

    x_increment = dx / steps
    y_increment = dy / steps

    x, y = float(x1), float(y1)
    
    path_points = [(round(x), round(y))]

    for _ in range(steps):
        x += x_increment
        y += y_increment
        path_points.append((round(x), round(y)))
        
    return path_points

# --- Parâmetros da Linha ---
P1 = (1, 1)
P2 = (8, 5)

# Gera os pontos da linha DDA
dda_path = dda_line(P1[0], P1[1], P2[0], P2[1])
x_path, y_path = zip(*dda_path)

# --- Plot ---
fig, ax = plt.subplots(figsize=(10, 7))

# Desenha a grade
ax.set_xticks(np.arange(0, 11, 1))
ax.set_yticks(np.arange(0, 8, 1))
ax.grid(True, which='both', color='gray', linestyle='--', linewidth=0.5)
ax.set_aspect('equal', adjustable='box')
ax.set_xlim(0, 10)
ax.set_ylim(0, 7)

# Plota a linha ideal
ax.plot([P1[0], P2[0]], [P1[1], P2[1]], 'r--', label='Linha Ideal')

# Plota o caminho DDA (pixel a pixel)
# Usamos 'steps-post' para criar o efeito de escada
ax.step(x_path, y_path, where='post', label='Caminho DDA (Pixels)', color='blue', linewidth=2)
# Adiciona marcadores nos centros dos pixels
ax.plot(np.array(x_path) + 0.0, np.array(y_path) + 0.0, 'bo', markersize=8) 

# Estilo do Gráfico
ax.set_title('Ilustração do Algoritmo DDA para Rasterização de Reta')
ax.set_xlabel('Eixo X (passos)')
ax.set_ylabel('Eixo Y (passos)')
ax.legend()

# --- Salvar Figura ---
output_path = os.path.join('tcc', 'src', 'Cap03', 'dda_line_rasterization.png')
plt.savefig(output_path)

print(f"Gráfico do DDA salvo em: {output_path}")
