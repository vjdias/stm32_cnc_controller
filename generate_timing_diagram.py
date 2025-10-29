
import matplotlib.pyplot as plt
import numpy as np
import os

# --- Parâmetros ---
# Firmware (em µs)
T_STEP_HIGH = 20
T_STEP_LOW = 20
T_DIR_SETUP = 20

# Datasheet (em µs)
T_STEP_MIN = 1.0
T_DIR_SETUP_MIN = 0.5

# --- Geração dos Sinais ---
# Tempo total para 2 pulsos
total_time = T_DIR_SETUP + 2 * (T_STEP_HIGH + T_STEP_LOW) + 10

# Pontos de tempo e valores dos sinais
time_points = [0, T_DIR_SETUP, T_DIR_SETUP, T_DIR_SETUP + T_STEP_HIGH, T_DIR_SETUP + T_STEP_HIGH, 
               T_DIR_SETUP + T_STEP_HIGH + T_STEP_LOW, T_DIR_SETUP + T_STEP_HIGH + T_STEP_LOW, 
               T_DIR_SETUP + 2*T_STEP_HIGH + T_STEP_LOW, T_DIR_SETUP + 2*T_STEP_HIGH + T_STEP_LOW, total_time]

step_signal = [0, 0, 1, 1, 0, 0, 1, 1, 0, 0]
dir_signal  = [1, 1, 1, 1, 1, 1, 1, 1, 1, 1] # DIR é estável antes e durante os pulsos

# --- Plot ---
fig, ax = plt.subplots(figsize=(12, 6))

# Plot dos sinais
ax.step(time_points, np.array(step_signal) * 0.8 + 1.5, where='post', label='STEP', color='b')
ax.step(time_points, np.array(dir_signal) * 0.8, where='post', label='DIR', color='g')

# --- Anotações ---
# t_DSU (DIR Setup Time)
ax.annotate('', xy=(0, 0.85), xytext=(T_DIR_SETUP, 0.85), 
            arrowprops=dict(arrowstyle='<->', color='purple'))
ax.text(T_DIR_SETUP/2, 0.9, f't_DSU = {T_DIR_SETUP} µs\n(min: {T_DIR_SETUP_MIN} µs)', 
        ha='center', va='bottom', color='purple')

# t_SH (STEP High Time)
t_sh_start = T_DIR_SETUP
ax.annotate('', xy=(t_sh_start, 1.55), xytext=(t_sh_start + T_STEP_HIGH, 1.55), 
            arrowprops=dict(arrowstyle='<->', color='r'))
ax.text(t_sh_start + T_STEP_HIGH/2, 1.6, f't_SH = {T_STEP_HIGH} µs\n(min: {T_STEP_MIN} µs)', 
        ha='center', va='bottom', color='r')

# t_SL (STEP Low Time)
t_sl_start = T_DIR_SETUP + T_STEP_HIGH
ax.annotate('', xy=(t_sl_start, 1.55), xytext=(t_sl_start + T_STEP_LOW, 1.55), 
            arrowprops=dict(arrowstyle='<->', color='orange'))
ax.text(t_sl_start + T_STEP_LOW/2, 1.6, f't_SL = {T_STEP_LOW} µs\n(min: {T_STEP_MIN} µs)', 
        ha='center', va='bottom', color='orange')

# --- Estilo do Gráfico ---
ax.set_ylim(-0.5, 3)
ax.set_xlim(-5, total_time)
ax.set_title('Diagrama de Tempo: Firmware vs. Datasheet TMC5160')
ax.set_xlabel('Tempo (µs)')
ax.set_yticks([0.4, 1.9])
ax.set_yticklabels(['Low', 'High'])
ax.grid(True, which='both', linestyle='--', linewidth=0.5)
ax.legend(loc='upper right')

# --- Salvar Figura ---
output_path = os.path.join('tcc', 'src', 'Cap03', 'tmc5160_timing.png')
plt.savefig(output_path)

print(f"Diagrama de tempo salvo em: {output_path}")
