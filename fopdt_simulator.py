#!/usr/bin/env python3
"""
Simulador FOPDT + PID alimentado pelos dados de análise SWV.

Permite testar rapidamente os ganhos (Kp, Ki, Kd) calculados em
analysis_summary.txt sem depender da GUI interativa original.

Uso:
    python fopdt_simulator.py --axes X:256 --duration 0.3 --plot

Também é possível simular vários eixos de uma só vez:
    python fopdt_simulator.py --axes X:16,Y:4,Z:256 --kp-scale 0.5
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

try:
    import numpy as np
except ImportError as exc:  # pragma: no cover
    raise SystemExit("numpy é necessário para rodar fopdt_simulator.py (pip install numpy)") from exc

try:
    import matplotlib.pyplot as plt
except ImportError:  # pragma: no cover
    plt = None

from tuning_profiles import AnalysisCatalog, AxisKey


TS_DEFAULT = 0.001  # 1 kHz loop (TIM7)
MAX_SPEED_MARGIN = 1.2  # 20% headroom acima do setpoint base


@dataclass
class SimulationResult:
    axis_label: str
    time: np.ndarray
    setpoint: np.ndarray
    plant_output: np.ndarray
    control_output: np.ndarray
    position: np.ndarray


class FOPDTPlant:
    """Modelo discreto First-Order Plus Dead-Time."""

    def __init__(self, gain: float, delay_s: float, tau_s: float, ts: float):
        self.gain = gain
        self.ts = ts
        self.delay_steps = max(0, int(round(delay_s / ts)))
        tau = max(1e-9, tau_s)
        self.alpha = 1.0 - math.exp(-ts / tau)
        self.delay_buffer: List[float] = [0.0] * self.delay_steps
        self.state = 0.0

    def reset(self) -> None:
        self.delay_buffer = [0.0] * self.delay_steps
        self.state = 0.0

    def step(self, u: float) -> float:
        if self.delay_steps:
            self.delay_buffer.append(u)
            u_delayed = self.delay_buffer.pop(0)
        else:
            u_delayed = u
        target = self.gain * u_delayed
        self.state += self.alpha * (target - self.state)
        return self.state


class VelocityPD:
    """Controlador PD discreto com saturação."""

    def __init__(self, kp: float, kd: float, ts: float, vmax: float):
        self.kp = kp
        self.kd = kd
        self.ts = ts
        self.vmax = vmax
        self.prev_error = 0.0

    def reset(self) -> None:
        self.prev_error = 0.0

    def compute(self, base_setpoint: float, measurement: float) -> float:
        error = base_setpoint - measurement
        derivative = (error - self.prev_error) / self.ts
        self.prev_error = error
        correction = self.kp * error + self.kd * derivative
        cmd = base_setpoint + correction
        return float(np.clip(cmd, 0.0, self.vmax))


def parse_axes(arg: str) -> List[Tuple[str, int]]:
    result = []
    for token in arg.split(","):
        token = token.strip()
        if not token:
            continue
        if ":" not in token:
            raise ValueError(f"Formato inválido para eixo: '{token}' (esperado X:256)")
        name, ms = token.split(":", 1)
        name = name.strip().upper()
        if name not in ("X", "Y", "Z"):
            raise ValueError(f"Eixo desconhecido: {name}")
        result.append((name, int(ms)))
    return result


def axis_to_id(letter: str) -> int:
    return {"X": 0, "Y": 1, "Z": 2}[letter.upper()]


def simulate_axis(
    profile,
    duration: float,
    ts: float,
    kp_scale: float,
    kd_scale: float,
    setpoint_override: float | None,
) -> SimulationResult:
    n_steps = int(round(duration / ts))
    time = np.arange(n_steps) * ts
    setpoint_value = setpoint_override or profile.steady_cmd
    setpoint = np.full(n_steps, setpoint_value, dtype=float)

    gain = profile.steady_enc / profile.steady_cmd if profile.steady_cmd else 1.0
    plant = FOPDTPlant(gain, profile.l_s, profile.tau_s, ts)
    vmax = MAX_SPEED_MARGIN * setpoint_value
    controller = VelocityPD(profile.kp * kp_scale, profile.kd * kd_scale, ts, vmax)
    plant.reset()
    controller.reset()

    plant_out = np.zeros(n_steps)
    control_out = np.zeros(n_steps)
    position = np.zeros(n_steps)

    for k in range(n_steps):
        measurement = plant_out[k - 1] if k > 0 else 0.0
        cmd = controller.compute(setpoint[k], measurement)
        control_out[k] = cmd
        plant_out[k] = plant.step(cmd)
        position[k] = position[k - 1] + plant_out[k] * ts if k > 0 else plant_out[k] * ts

    label = AxisKey(profile.axis, profile.microstep).label()
    return SimulationResult(label, time, setpoint, plant_out, control_out, position)


def summarize(result: SimulationResult, steady_enc: float) -> Dict[str, float]:
    peak = float(np.max(result.plant_output))
    overshoot = max(0.0, (peak - steady_enc) / steady_enc * 100.0) if steady_enc else 0.0
    rise_idx = np.argmax(result.plant_output >= 0.9 * steady_enc) if steady_enc else 0
    rise_time = result.time[rise_idx] if rise_idx > 0 else float("nan")
    return {"peak": peak, "overshoot_pct": overshoot, "rise_time_s": rise_time}


def plot_results(results: Sequence[SimulationResult]) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    ax_vel, ax_ctrl = axes
    for res in results:
        ax_vel.plot(res.time, res.setpoint, "--", alpha=0.4, label=f"{res.axis_label} setpoint")
        ax_vel.plot(res.time, res.plant_output, label=f"{res.axis_label} output")
        ax_ctrl.plot(res.time, res.control_output, label=f"{res.axis_label} comando")
    ax_vel.set_ylabel("velocidade (pulsos/s)")
    ax_vel.grid(True, alpha=0.3)
    ax_vel.legend()
    ax_ctrl.set_ylabel("comando (pulsos/s)")
    ax_ctrl.set_xlabel("tempo (s)")
    ax_ctrl.grid(True, alpha=0.3)
    ax_ctrl.legend()
    fig.tight_layout()
    plt.show()


def main() -> int:
    parser = argparse.ArgumentParser(description="Simulador FOPDT + PD com dados da análise SWV.")
    parser.add_argument(
        "--axes",
        required=True,
        help="Lista de eixos no formato X:256,Y:16 (default: nenhum)",
    )
    parser.add_argument("--duration", type=float, default=0.3, help="Duração da simulação (s)")
    parser.add_argument("--ts", type=float, default=TS_DEFAULT, help="Passo de controle (s)")
    parser.add_argument("--kp-scale", type=float, default=1.0, help="Escala multiplicativa para Kp")
    parser.add_argument("--kd-scale", type=float, default=1.0, help="Escala multiplicativa para Kd")
    parser.add_argument("--setpoint", type=float, default=None, help="Override de setpoint (pulsos/s)")
    parser.add_argument("--plot", action="store_true", help="Exibe gráficos com matplotlib")
    parser.add_argument(
        "--emit-fw",
        action="store_true",
        help="Ao final, imprime os ganhos inteiros (kp_xyz/ki_xyz/kd_xyz) "
             "para colar no simulador interativo.",
    )
    args = parser.parse_args()
    if args.plot and plt is None:
        print("[WARN] matplotlib não está disponível; executando sem gráficos.")
        args.plot = False

    catalog = AnalysisCatalog()
    axes = parse_axes(args.axes)
    results: List[SimulationResult] = []
    fw_kp = {"X": 0, "Y": 0, "Z": 0}
    fw_ki = {"X": 0, "Y": 0, "Z": 0}
    fw_kd = {"X": 0, "Y": 0, "Z": 0}
    for letter, mstep in axes:
        key = AxisKey(axis_to_id(letter), mstep)
        profile = catalog.get(key.axis, key.microstep)
        if not profile:
            print(f"[WARN] Sem perfil para {key.label()}, pulando.")
            continue
        result = simulate_axis(profile, args.duration, args.ts, args.kp_scale, args.kd_scale, args.setpoint)
        stats = summarize(result, profile.steady_enc)
        print(
            f"{result.axis_label}: pico={stats['peak']:.1f} pulsos/s "
            f"overshoot={stats['overshoot_pct']:.1f}% "
            f"rise≈{stats['rise_time_s']:.3f}s"
        )
        results.append(result)
        kp_i, ki_i, kd_i = profile.firmware_gains()
        fw_kp[letter] = kp_i
        fw_ki[letter] = ki_i
        fw_kd[letter] = kd_i

    if args.emit_fw:
        kp_tuple = (fw_kp["X"], fw_kp["Y"], fw_kp["Z"])
        ki_tuple = (fw_ki["X"], fw_ki["Y"], fw_ki["Z"])
        kd_tuple = (fw_kd["X"], fw_kd["Y"], fw_kd["Z"])
        print("\nGanhos prontos para o Scenario:")
        print(f"kp_xyz={kp_tuple}")
        print(f"ki_xyz={ki_tuple}")
        print(f"kd_xyz={kd_tuple}")

    if args.plot and results:
        plot_results(results)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
