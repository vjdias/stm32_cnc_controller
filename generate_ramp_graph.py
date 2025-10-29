import matplotlib.pyplot as plt
import numpy as np
import os
import matplotlib.ticker as mticker

# Parameters from the firmware
TOTAL_STEPS = 150000
ACCEL_SPS2 = 20000  # steps/s^2
MAX_VELOCITY_SPS = 25000  # steps/s
TIME_STEP_S = 0.001  # 1ms from TIM7

# --- Simulation ---
time_points = [0]
velocity_points = [0]
distance_points = [0]
current_velocity_sps = 0
total_distance_steps = 0
time_s = 0

while total_distance_steps < TOTAL_STEPS:
    time_s += TIME_STEP_S
    braking_distance = (current_velocity_sps ** 2) / (2 * ACCEL_SPS2) if ACCEL_SPS2 > 0 else float('inf')
    remaining_steps = TOTAL_STEPS - total_distance_steps
    if remaining_steps <= braking_distance:
        current_velocity_sps -= ACCEL_SPS2 * TIME_STEP_S
        current_velocity_sps = max(0, current_velocity_sps)
    else:
        current_velocity_sps += ACCEL_SPS2 * TIME_STEP_S
        current_velocity_sps = min(MAX_VELOCITY_SPS, current_velocity_sps)
    total_distance_steps += current_velocity_sps * TIME_STEP_S
    time_points.append(time_s)
    velocity_points.append(current_velocity_sps)
    distance_points.append(total_distance_steps)

# --- Plotting ---
fig = plt.figure(figsize=(15, 10))
gs = fig.add_gridspec(2, 3)
ax1 = fig.add_subplot(gs[0, :])
ax_zoom1 = fig.add_subplot(gs[1, 0])
ax_zoom2 = fig.add_subplot(gs[1, 1])
ax_zoom3 = fig.add_subplot(gs[1, 2])

fig.suptitle('Análise do Perfil de Movimento e Densidade de Pulsos', fontsize=16)

# --- Subplot 1: Perfil de Velocidade ---
ax1.plot(time_points, velocity_points, label='Velocidade (steps/s)', color='dodgerblue')
ax1.set_title('Perfil de Velocidade Trapezoidal')
ax1.set_ylabel('Velocidade (steps/s)')
ax1.grid(True, linestyle=':')
ax1.axhline(y=MAX_VELOCITY_SPS, color='r', linestyle='--', label=f'Velocidade Máxima ({MAX_VELOCITY_SPS} steps/s)')
ax1.legend()

# --- Subplots de Zoom: Trem de Pulsos ---
pulse_times = []
last_step_count = 0
distance_points_floor = np.floor(distance_points)
for i in range(1, len(distance_points)):
    if distance_points_floor[i] > distance_points_floor[i-1]:
        num_new_pulses = int(distance_points_floor[i] - distance_points_floor[i-1])
        for j in range(num_new_pulses):
            den = distance_points[i] - distance_points[i-1]
            frac = ((distance_points_floor[i-1] + 1 + j) - distance_points[i-1]) / den if den > 0 else 0
            t_pulse = time_points[i-1] + frac * TIME_STEP_S
            if t_pulse <= time_points[i]:
                 pulse_times.append(t_pulse)

# Define as janelas de tempo para o zoom (4ms de duração)
win_duration = 0.004

# Calcula tempos da rampa para posicionar as janelas
time_to_accel = MAX_VELOCITY_SPS / ACCEL_SPS2
steps_for_accel = ACCEL_SPS2 * (time_to_accel**2) / 2
steps_on_plateau = TOTAL_STEPS - (2 * steps_for_accel)
time_on_plateau = steps_on_plateau / MAX_VELOCITY_SPS if MAX_VELOCITY_SPS > 0 else 0

windows = [
    (0.396, 'Aceleração', 'green', ax_zoom1),
    (time_to_accel + time_on_plateau / 2, 'Vel. Máxima', 'blue', ax_zoom2),
    (time_to_accel + time_on_plateau + time_to_accel / 2, 'Desaceleração', 'red', ax_zoom3)
]

for start, label, color, ax in windows:
    end = start + win_duration
    win_pulses = [t for t in pulse_times if start <= t < end]
    ax.eventplot(win_pulses, color=color, linelengths=0.8)
    ax.set_title(f'{label} ({len(win_pulses)} pulsos)')
    ax.set_xlabel('Tempo (s)')
    ax.set_yticks([])
    ax.set_xlim(start, end)
    ax.grid(True, axis='x', linestyle=':')
    
    # Formata o eixo X para mostrar milissegundos com 3 casas decimais
    ax.xaxis.set_major_formatter(mticker.FormatStrFormatter('%.3f'))
    ax.xaxis.set_major_locator(mticker.MaxNLocator(nbins=4, prune='both'))
    plt.setp(ax.get_xticklabels(), rotation=30, ha='right')

    # Adiciona a região sombreada no gráfico de velocidade
    ax1.axvspan(start, end, color=color, alpha=0.2)
    ax1.text(start + win_duration/2, 1000, label.replace(' ', '\n'), ha='center', va='bottom', color=color, fontsize=9)

# --- Finalização ---
plt.tight_layout(rect=[0, 0, 1, 0.96])
output_path = os.path.join('tcc', 'src', 'Cap03', 'rampa_trapezoidal.png')
plt.savefig(output_path)
print(f"Gráfico salvo em: {output_path}")
