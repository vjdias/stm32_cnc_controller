import numpy as np
import matplotlib.pyplot as plt

"""
Análise da Velocidade e Corrente do Motor (Contexto da Discussão)

O cálculo da corrente RMS real no motor é:
I_rms = ((IRUN + 1) / 32) * (GLOBAL_SCALER / 256) * (0.325 / Rsense) * (1 / 1.414)

Onde:
- IRUN: 1
- GLOBAL_SCALER: 128
- Rsense: 0.022 Ω (do manual do driver BIGTREETECH TMC5160T Plus)

Cálculo passo a passo:
1. Termo IRUN: (1 + 1) / 32 = 0.0625
2. Termo GLOBAL_SCALER: 128 / 256 = 0.5
3. Corrente de Pico Máxima (teórica com Rsense=0.022): 0.325 V / 0.022 Ω ≈ 14.77 A
4. I_pico_real = 14.77 A * 0.0625 * 0.5 ≈ 0.46 A
5. I_rms_real = 0.46 A / 1.414 ≈ 0.327 A

Conclusão da Análise Teórica:
- Corrente Fornecida: ~0.33 A
- Corrente Nominal do Motor (NEMA 23): 2.8 A a 3.0 A
- A corrente configurada é ~11% da nominal, o que teoricamente não seria suficiente
  para vencer a inércia e atrito, levando a uma velocidade de 0 RPM.

Hipótese Prática (porque o motor GIRA):
- O driver opera em modo StealthChop™, que ajusta a corrente dinamicamente
  com base na carga, usando o valor de IRUN mais como uma "dica" do que
  um valor absoluto em baixas velocidades.

Cálculo da Velocidade Comandada:
- Parâmetro de velocidade `velTick` = 5
- v_target_sps = 5 * 1000 = 5000 SPS
- Passos por Rotação = 400 (base) * 256 (microstep) = 102400
- RPM = (5000 / 102400) * 60 ≈ 2.93 RPM
"""

def simulate_pid_tracking():
    # Parâmetros do controlador PID (firmware)
    KP = 10
    KI = 2
    KD = 5
    PI_SHIFT = 8

    # Parâmetros da simulação baseados em dados reais
    dt = 0.001  # Passo de tempo (~1kHz)
    total_distance = 102400  # pulsos (1 volta)
    move_duration = 12.8     # segundos
    target_velocity = total_distance / move_duration # 8000 passos/s
    simulation_time = 14.0 # segundos (para ver o final)
    n_points = int(simulation_time / dt)

    # Variáveis de estado
    process_variable = 0.0
    integral_error = 0.0
    previous_error = 0.0
    d_filt = 0.0
    alpha = 8

    # Histórico para o gráfico
    time_history = np.linspace(0, simulation_time, n_points)
    pv_history = []
    setpoint_history = []
    velocity_history = []

    # Loop de simulação
    for t in time_history:
        # Calcula o setpoint móvel (rampa de posição)
        if t <= move_duration:
            current_setpoint = t * target_velocity
        else:
            current_setpoint = total_distance # Mantém o alvo no final

        # Cálculo do erro de rastreamento (tracking error)
        error = current_setpoint - process_variable

        # Termo Integral
        integral_error += error

        # Termo Derivativo com filtro
        draw = error - previous_error
        d_filt = d_filt + ((draw - d_filt) / (2**alpha))
        previous_error = error

        # Saída do controlador (velocidade de CORREÇÃO em passos/s)
        p_term = (KP / (2**PI_SHIFT)) * error
        i_term = (KI / (2**PI_SHIFT)) * (integral_error * dt)
        d_term = (KD / (2**PI_SHIFT)) * (d_filt / dt)
        correction_velocity = p_term + i_term + d_term

        # A velocidade final é a velocidade alvo + a correção do PID
        final_velocity = target_velocity + correction_velocity
        if t > move_duration:
            final_velocity = correction_velocity # Após o fim do movimento, a vel alvo é 0

        # Modelo do sistema
        process_variable += final_velocity * dt

        # Armazena histórico
        pv_history.append(process_variable)
        setpoint_history.append(current_setpoint)
        velocity_history.append(final_velocity)

    # Plotagem
    plt.style.use('seaborn-v0_8-whitegrid')
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 9), sharex=True)
    
    # Gráfico 1: Posição vs. Tempo
    ax1.plot(time_history, setpoint_history, 'r--', label='Setpoint (Alvo Móvel)')
    ax1.plot(time_history, pv_history, 'b-', label='Posição Real do Motor')
    ax1.set_title('Simulação de Rastreamento de Trajetória com PID', fontsize=16)
    ax1.set_ylabel('Posição (passos)', fontsize=12)
    ax1.legend(fontsize=10)
    ax1.grid(True)

    # Gráfico 2: Velocidade vs. Tempo
    ax2.plot(time_history, velocity_history, 'g-', label='Velocidade Real do Motor')
    ax2.axhline(y=target_velocity, color='orange', linestyle='--', label=f'Velocidade Alvo ({int(target_velocity)} passos/s)')
    ax2.set_xlabel('Tempo (s)', fontsize=12)
    ax2.set_ylabel('Velocidade (passos/s)', fontsize=12)
    ax2.legend(fontsize=10)
    ax2.grid(True)
    ax2.set_ylim(bottom=0, top=target_velocity * 1.2) # Ajuste de eixo

    fig.tight_layout()

    # Salva a figura
    output_path = 'tcc/images/pid_tracking_simulation.png'
    plt.savefig(output_path)
    print(f"Gráfico salvo em: {output_path}")

if __name__ == '__main__':
    simulate_pid_tracking()