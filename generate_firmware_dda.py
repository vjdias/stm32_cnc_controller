
import matplotlib.pyplot as plt
import numpy as np
import os

# --- Parâmetros da Simulação (baseado no firmware e exemplo)

# Movimento: 7 passos em X, 4 em Y
TOTAL_STEPS_X = 7
TOTAL_STEPS_Y = 4

# Parâmetros do Timer e DDA
TIM6_HZ = 50000
Q16_1 = 1 << 16

# Define a velocidade do eixo dominante (X) e calcula a do outro
# A velocidade real não importa tanto quanto a razão entre elas
DOMINANT_AXIS_VEL = 25000 # steps/s

V_X = DOMINANT_AXIS_VEL
V_Y = DOMINANT_AXIS_VEL * (TOTAL_STEPS_Y / TOTAL_STEPS_X)

# Calcula os incrementos do DDA, como no firmware
DDA_INC_X = int((V_X / TIM6_HZ) * Q16_1)
DDA_INC_Y = int((V_Y / TIM6_HZ) * Q16_1)

# --- Simulação Tick a Tick ---
accum_x, accum_y = 0, 0
steps_x, steps_y = 0, 0
tick = 0

# Listas para guardar o estado a cada tick
tick_list = [0]
accum_x_list = [0]
accum_y_list = [0]
pulse_x_times = []
pulse_y_times = []

# O loop continua até o eixo dominante completar seus passos
while steps_x < TOTAL_STEPS_X:
    tick += 1
    
    accum_x += DDA_INC_X
    accum_y += DDA_INC_Y
    
    tick_list.append(tick)
    accum_x_list.append(accum_x)
    accum_y_list.append(accum_y)
    
    if accum_x >= Q16_1:
        accum_x -= Q16_1
        steps_x += 1
        pulse_x_times.append(tick)
        
    if accum_y >= Q16_1:
        accum_y -= Q16_1
        steps_y += 1
        pulse_y_times.append(tick)

# --- Plot ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
fig.suptitle(f'Simulação do DDA para um Movimento (dX={TOTAL_STEPS_X}, dY={TOTAL_STEPS_Y})', fontsize=16)

# --- Subplot 1: Acumuladores ---
ax1.plot(tick_list, accum_x_list, label=f'Acumulador X (inc: {DDA_INC_X})', drawstyle='steps-post')
ax1.plot(tick_list, accum_y_list, label=f'Acumulador Y (inc: {DDA_INC_Y})', drawstyle='steps-post')
ax1.axhline(y=Q16_1, color='r', linestyle='--', label=f'Limiar de Overflow ({Q16_1})')
ax1.set_title('Evolução dos Acumuladores DDA a cada Tick')
ax1.set_ylabel('Valor do Acumulador')
ax1.legend()
ax1.grid(True, linestyle=':')

# --- Subplot 2: Pulsos Gerados ---
ax2.eventplot(pulse_x_times, lineoffsets=1.0, linelengths=0.8, label='Pulsos Eixo X', color='C0')
ax2.eventplot(pulse_y_times, lineoffsets=0.0, linelengths=0.8, label='Pulsos Eixo Y', color='C1')
ax2.set_title('Trem de Pulsos STEP Gerado')
ax2.set_ylabel('Eixo')
ax2.set_xlabel('Tempo (ticks do TIM6 @ 50 kHz)')
ax2.set_yticks([0, 1], labels=['Y', 'X'])
ax2.set_ylim(-0.7, 1.7)
ax2.grid(True, axis='x', linestyle=':')
ax2.legend()

# --- Finalização ---
plt.tight_layout(rect=[0, 0, 1, 0.95])
output_path = os.path.join('tcc', 'src', 'Cap03', 'firmware_dda_simulation.png')
plt.savefig(output_path)

print(f"Gráfico da simulação do DDA do firmware salvo em: {output_path}")
