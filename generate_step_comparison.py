
import matplotlib.pyplot as plt
import numpy as np
import os

# --- Cria a figura com 2 subplots ---
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
fig.suptitle('Comparação do Pulso de STEP: Firmware vs. Datasheet', fontsize=16)

# --- Subplot 1: Pulso do Firmware ---
ax1.set_title('Pulso STEP Implementado')
T_HIGH_FW = 20  # µs
T_LOW_FW = 20   # µs

# Dados do sinal
time_fw = [0, 5, 5, 5 + T_HIGH_FW, 5 + T_HIGH_FW, 5 + T_HIGH_FW + T_LOW_FW]
step_fw = [0, 0, 1, 1, 0, 0]

ax1.plot(time_fw, step_fw, drawstyle='steps-post', color='b')
ax1.set_xlabel('Tempo (µs)')
ax1.set_ylabel('Nível Lógico')
ax1.set_ylim(-0.1, 1.4)
ax1.set_xlim(0, 50)
ax1.set_yticks([0, 1])
ax1.grid(True, linestyle=':')

# Anotações para o firmware
ax1.annotate('', xy=(5, 1.05), xytext=(25, 1.05), arrowprops=dict(arrowstyle='<->', color='r'))
ax1.text(15, 1.1, f't_SH = {T_HIGH_FW} µs', ha='center', color='r')

ax1.annotate('', xy=(25, 1.05), xytext=(45, 1.05), arrowprops=dict(arrowstyle='<->', color='orange'))
ax1.text(35, 1.1, f't_SL = {T_LOW_FW} µs', ha='center', color='orange')

# --- Subplot 2: Pulso Mínimo do Datasheet ---
ax2.set_title('Pulso STEP Mínimo (Datasheet)')
T_HIGH_DS = 1.0  # µs
T_LOW_DS = 1.0   # µs

# Dados do sinal
time_ds = [0, 0.5, 0.5, 0.5 + T_HIGH_DS, 0.5 + T_HIGH_DS, 0.5 + T_HIGH_DS + T_LOW_DS]
step_ds = [0, 0, 1, 1, 0, 0]

ax2.plot(time_ds, step_ds, drawstyle='steps-post', color='b')
ax2.set_xlabel('Tempo (µs)')
ax2.set_ylabel('Nível Lógico')
ax2.set_ylim(-0.1, 1.4)
ax2.set_xlim(0, 5)
ax2.set_yticks([0, 1])
ax2.grid(True, linestyle=':')

# Anotações para o datasheet
ax2.annotate('', xy=(0.5, 1.05), xytext=(1.5, 1.05), arrowprops=dict(arrowstyle='<->', color='r'))
ax2.text(1.0, 1.1, f't_SH >= {T_HIGH_DS} µs', ha='center', color='r')

ax2.annotate('', xy=(1.5, 1.05), xytext=(2.5, 1.05), arrowprops=dict(arrowstyle='<->', color='orange'))
ax2.text(2.0, 1.1, f't_SL >= {T_LOW_DS} µs', ha='center', color='orange')

# --- Finalização ---
plt.tight_layout(rect=[0, 0, 1, 0.96])

# Salvar a figura
output_path = os.path.join('tcc', 'src', 'Cap03', 'step_pulse_comparison.png')
plt.savefig(output_path)

print(f"Gráfico de comparação de pulsos salvo em: {output_path}")
