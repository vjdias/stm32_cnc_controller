
import matplotlib.pyplot as plt
import numpy as np
import os

# Parameters from the firmware
TOTAL_STEPS = 150000
ACCEL_SPS2 = 20000  # steps/s^2
MAX_VELOCITY_SPS = 25000  # steps/s
TIME_STEP_S = 0.001  # 1ms from TIM7

# Simulation variables
time_points = [0]
velocity_points = [0]
distance_points = [0]
current_velocity_sps = 0
total_distance_steps = 0
time_s = 0

while total_distance_steps < TOTAL_STEPS:
    time_s += TIME_STEP_S

    # Braking distance: s = v^2 / (2*a)
    braking_distance = (current_velocity_sps ** 2) / (2 * ACCEL_SPS2)
    remaining_steps = TOTAL_STEPS - total_distance_steps

    if remaining_steps <= braking_distance:
        # Deceleration phase
        current_velocity_sps -= ACCEL_SPS2 * TIME_STEP_S
        current_velocity_sps = max(0, current_velocity_sps)
    else:
        # Acceleration/Constant velocity phase
        current_velocity_sps += ACCEL_SPS2 * TIME_STEP_S
        current_velocity_sps = min(MAX_VELOCITY_SPS, current_velocity_sps)

    # Update distance moved in this time step
    total_distance_steps += current_velocity_sps * TIME_STEP_S

    time_points.append(time_s)
    velocity_points.append(current_velocity_sps)
    distance_points.append(total_distance_steps)

# --- Plotting ---
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10), sharex=True)
fig.suptitle('Análise do Perfil de Movimento', fontsize=16)

# --- Subplot 1: Perfil de Velocidade ---
ax1.plot(time_points, velocity_points, label='Velocidade (steps/s)', color='dodgerblue')
ax1.set_title('Perfil de Velocidade Trapezoidal')
ax1.set_ylabel('Velocidade (steps/s)')
ax1.grid(True, linestyle=':')
ax1.axhline(y=MAX_VELOCITY_SPS, color='r', linestyle='--', label=f'Velocidade Máxima ({MAX_VELOCITY_SPS} steps/s)')

# Anotações de Aceleração
time_to_accel = MAX_VELOCITY_SPS / ACCEL_SPS2
steps_for_accel = ACCEL_SPS2 * (time_to_accel**2) / 2
steps_for_decel = steps_for_accel
steps_on_plateau = TOTAL_STEPS - (steps_for_accel + steps_for_decel)
time_on_plateau = steps_on_plateau / MAX_VELOCITY_SPS if MAX_VELOCITY_SPS > 0 else 0
accel_time_annotation = time_to_accel * 0.5
accel_vel_annotation = MAX_VELOCITY_SPS * 0.5
ax1.text(accel_time_annotation, accel_vel_annotation + 2000, f'Aceleração\n+{ACCEL_SPS2} steps/s²', ha='center', va='center', color='green', bbox=dict(facecolor='white', alpha=0.5, edgecolor='none'))
decel_time_annotation = time_to_accel + time_on_plateau + time_to_accel * 0.5
decel_vel_annotation = MAX_VELOCITY_SPS * 0.5
ax1.text(decel_time_annotation, decel_vel_annotation + 2000, f'Desaceleração\n-{ACCEL_SPS2} steps/s²', ha='center', va='center', color='red', bbox=dict(facecolor='white', alpha=0.5, edgecolor='none'))
ax1.legend()

# --- Subplot 2: Trem de Pulsos STEP (Amostrado) ---
# Gera os timestamps exatos de cada pulso por interpolação
pulse_times = []
last_step_count = 0
for i in range(1, len(distance_points)):
    current_step_count = np.floor(distance_points[i])
    if current_step_count > last_step_count:
        num_new_pulses = int(current_step_count - last_step_count)
        # Interpola para encontrar o tempo de cada pulso
        for j in range(num_new_pulses):
            frac = ((last_step_count + 1 + j) - distance_points[i-1]) / (distance_points[i] - distance_points[i-1])
            t_pulse = time_points[i-1] + frac * TIME_STEP_S
            if t_pulse < time_points[i]:
                 pulse_times.append(t_pulse)
        last_step_count = current_step_count

# Amostragem dos pulsos para visualização
sample_rate = 250
sampled_pulse_times = pulse_times[::sample_rate]

ax2.eventplot(sampled_pulse_times, color='purple', linelengths=0.75)
ax2.set_title(f'Trem de Pulsos STEP (1 a cada {sample_rate} pulsos)')
ax2.set_ylabel('Pulso STEP')
ax2.set_xlabel('Tempo (s)')
ax2.set_yticks([]) # O eixo Y não tem valor numérico, apenas representa o evento
ax2.grid(True, axis='x', linestyle=':')

# --- Finalização ---
plt.tight_layout(rect=[0, 0, 1, 0.96])
# Save the plot
output_path = os.path.join('tcc', 'src', 'Cap03', 'rampa_trapezoidal.png')
plt.savefig(output_path)

print(f"Gráfico salvo em: {output_path}")
