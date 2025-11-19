#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gera uma simulação OLD para o cenário de 6 voltas
e compara a velocidade de passo com as curvas medidas via SWV.

Saída principal:
  - Figura PNG com sobreposição velocidade (steps/s) simulada vs. medida
    por eixo (X/Y/Z), salva na pasta de imagens do TCC.
"""
from __future__ import annotations

import os
import csv
import math
from pathlib import Path
from typing import Dict, List, Tuple
import re

# Força backend não interativo antes de qualquer import do matplotlib/interactive_sim
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent

import sys  # noqa: E402

if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from interactive_sim import (  # noqa: E402
    PlantConfig,
    Scenario,
    InteractiveSim,
)
SWV_PATH = REPO_ROOT / "CNC_Controller" / "SWV_export" / "SWV_ITM_Data_Console_sematrito.txt"
SIM_LOG_DIR = REPO_ROOT / "sim_logs"
OUT_IMG = REPO_ROOT / "tcc" / "src" / "Cap03" / "imagens" / "TCC_SIMULACAO_SEM_ATRITO.png"

# Encoders conforme firmware (motion_service.c)
ENC_COUNTS_PER_REV = {0: 40000.0, 1: 5000.0, 2: 40000.0}
STEPS_PER_REV_BASE = 400.0
MICROSTEP = 256.0
DDA_STEPS_PER_REV = STEPS_PER_REV_BASE * MICROSTEP

# Mesmo padrão usado em filter_swv_export.py
NUMERIC_ROW = re.compile(r"^\s*(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")


def _moving_average(values: List[float], window: int) -> List[float]:
    """Suaviza a série por média móvel simples."""
    if window <= 1 or len(values) <= 1:
        return list(values)
    out: List[float] = []
    acc = 0.0
    buf: List[float] = []
    for v in values:
        buf.append(v)
        acc += v
        if len(buf) > window:
            acc -= buf.pop(0)
        out.append(acc / len(buf))
    return out


def build_sim() -> InteractiveSim:
    """Configura InteractiveSim no modo OLD, sem atrito, para 6 voltas."""
    cfg = PlantConfig(
        Ts=0.001,
        tim6_hz=50_000.0,
        microstep_factor=int(MICROSTEP),
        enc_cpr_xyz=(40000, 5000, 40000),
        kd_alpha_bits=8,
        step_high_ticks=1,
        step_low_ticks=1,
        load_B_xyz=(0.0, 0.0, 0.0),
    )

    # 6 voltas completas em cada eixo: 6 * 400 * 256 = 614400 steps
    revolutions = 6.0
    total_steps = int(revolutions * DDA_STEPS_PER_REV)

    # Velocidade: vel=8 no firmware => 8 steps/ms => 8000 steps/s
    v_sps = 8000.0

    scn = Scenario(
        s_xyz=(total_steps, total_steps, total_steps),
        v_xyz=(v_sps, v_sps, v_sps),
        dir_xyz=(1, 1, 1),
        # Ganhos conforme experimento: Kp≈801/797, Ki=Kd=0
        kp_xyz=(801, 801, 797),
        ki_xyz=(0, 0, 0),
        kd_xyz=(0, 0, 0),
        sim_time_s=90.0,
        use_dda=True,
    )

    sim = InteractiveSim(
        cfg,
        scn,
        log_dir=SIM_LOG_DIR,
        enable_logging=True,
        auto_analyze=False,
        headless=False,  # força criação de timer/figura, mas sem GUI (Agg)
        show_friction_band=False,
    )

    # Modo OLD (mesmas flags do interactive_old_sim_y5000)
    sim.master_select_strategy = "progress"
    sim.prefer_loaded_master = False
    sim.master_switch_margin_steps = 0.0
    sim.sync_err_feed_threshold = 200.0
    sim.sync_err_feed_min_fraction = 0.25
    sim.sync_hold_enabled = False
    sim.finish_all_axes = False
    sim.ramp_use_worst_remaining = False
    sim.global_stop_all_axes = True
    sim.finish_window_steps = 0.0
    sim.finish_disable_stall = False
    sim.finish_extra_budget_steps = 0

    return sim


def run_simulation() -> Path:
    """Roda a simulação até o término lógico e retorna o caminho do CSV."""
    SIM_LOG_DIR.mkdir(parents=True, exist_ok=True)

    os.environ.setdefault("MPLBACKEND", "Agg")

    sim = build_sim()
    sim.reset(None)

    # Zera atrito (C/B) e janelas
    try:
        sim.C_load_values[:] = 0.0
        sim.B_load[:] = 0.0
        sim.load_start_times[:] = 0.0
        sim.load_end_times[:] = 0.0
    except Exception:
        pass

    sim.is_paused = False
    sim._start_log_session()

    max_steps = 200_000
    for _ in range(max_steps):
        sim._step()
        if sim.is_paused:
            break

    sim._stop_log_session()
    if not sim.last_log_path:
        raise RuntimeError("Simulação não gerou arquivo de log.")
    return sim.last_log_path


def load_sim_log(path: Path) -> Dict[int, Tuple[List[float], List[float]]]:
    """Carrega velocidade de passo (steps/s) por eixo a partir do CSV da simulação."""
    result: Dict[int, Tuple[List[float], List[float]]] = {0: ([], []), 1: ([], []), 2: ([], [])}
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row["t_s"])
                v_x = float(row["vel_sps_x"])
                v_y = float(row["vel_sps_y"])
                v_z = float(row["vel_sps_z"])
            except Exception:
                continue
            for axis, vel in enumerate((v_x, v_y, v_z)):
                t_list, y_list = result[axis]
                t_list.append(t)
                y_list.append(vel)
    return result


def load_swv_per_axis(path: Path) -> Dict[int, Tuple[List[float], List[float]]]:
    """
    Extrai curvas de velocidade por eixo a partir do dump SWV de console.

    Reaproveita as mesmas regras do filtro de SWV:
      - aceita apenas linhas com 5 inteiros axis,id,time_ms,encoder,pulses;
      - exige ids estritamente crescentes e time/pulses não decrescentes (por eixo);
      - usa valor absoluto do encoder;
      - deriva velocidade de passo a partir da contagem acumulada de pulsos.
    """
    per_axis: Dict[int, Tuple[List[float], List[float]]] = {}
    last_id: Dict[int, int] = {}
    last_time: Dict[int, int] = {}
    last_pulses: Dict[int, int] = {}

    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for raw in f:
            m = NUMERIC_ROW.match(raw.strip())
            if not m:
                continue
            axis = int(m.group(1), 10)
            seq = int(m.group(2), 10)
            t_ms = int(m.group(3), 10)
            enc = int(m.group(4), 10)
            pulses = int(m.group(5), 10)
            if axis not in (0, 1, 2):
                continue
            # Monotonia por eixo
            if axis in last_id:
                if seq <= last_id[axis] or t_ms < last_time[axis] or pulses < last_pulses[axis]:
                    continue
            # Deriva velocidade de passo a partir de pulses acumulados
            if axis in last_time:
                dt_ms = t_ms - last_time[axis]
                dpulses = pulses - last_pulses[axis]
                if dt_ms > 0 and dpulses >= 0:
                    dt_s = dt_ms / 1000.0
                    v_sps = dpulses / dt_s
                    t_s = t_ms / 1000.0
                    t_list, y_list = per_axis.setdefault(axis, ([], []))
                    t_list.append(t_s)
                    y_list.append(v_sps)

            last_id[axis] = seq
            last_time[axis] = t_ms
            last_pulses[axis] = pulses

    return per_axis


def make_comparison_plot(
    swv: Dict[int, Tuple[List[float], List[float]]],
    sim: Dict[int, Tuple[List[float], List[float]]],
    out_path: Path,
) -> Dict[int, Dict[str, float]]:
    """Gera gráfico comparativo e retorna métricas simples por eixo."""
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(3, 1, figsize=(7.5, 8.5), sharex=True)
    axis_labels = {0: "X", 1: "Y", 2: "Z"}
    metrics: Dict[int, Dict[str, float]] = {}

    for axis in (0, 1, 2):
        ax = axes[axis]
        t_meas, y_meas = swv.get(axis, ([], []))
        t_sim, y_sim = sim.get(axis, ([], []))

        if not t_meas or not t_sim:
            continue

        # Limita a simulação ao mesmo intervalo de tempo da medição
        t_max = t_meas[-1]
        t_sim_clip = [t for t in t_sim if t <= t_max]
        y_sim_clip = y_sim[: len(t_sim_clip)]

        # Suaviza ambas as séries com média móvel (janela ~0,5% do total)
        win_meas = max(1, len(y_meas) // 200)
        win_sim = max(1, len(y_sim_clip) // 200)
        y_meas_s = _moving_average(y_meas, win_meas)
        y_sim_s = _moving_average(y_sim_clip, win_sim)

        ax.plot(t_meas, y_meas_s, label="medido (pulsos/s)", color="tab:blue")
        ax.plot(t_sim_clip, y_sim_s, label="simulação", color="tab:red", linestyle="--")
        ax.set_ylabel(f"eixo {axis_labels[axis]} (steps/s)")
        ax.grid(True, alpha=0.3)

        # Métricas simples: velocidade média de regime e erro relativo
        n_tail = max(1, int(len(y_meas_s) * 0.1))
        v_meas = sum(y_meas_s[-n_tail:]) / n_tail
        n_tail_sim = max(1, int(len(y_sim_s) * 0.1))
        v_sim = sum(y_sim_s[-n_tail_sim:]) / n_tail_sim
        err_abs = v_meas - v_sim
        err_pct = (err_abs / v_meas) * 100.0 if v_meas != 0 else math.nan
        metrics[axis] = {
            "v_meas": v_meas,
            "v_sim": v_sim,
            "err_abs": err_abs,
            "err_pct": err_pct,
        }

        ax.legend(loc="best", fontsize=8)

    axes[-1].set_xlabel("tempo (s)")
    fig.suptitle("Comparação de velocidade de passo: medido vs. simulado (6 voltas)")
    fig.tight_layout(rect=[0.03, 0.03, 0.97, 0.96])
    fig.savefig(out_path, dpi=130)
    plt.close(fig)
    return metrics


def main() -> int:
    if not SWV_PATH.exists():
        raise SystemExit(f"Arquivo SWV não encontrado: {SWV_PATH}")

    print("==> Rodando simulação OLD (6 voltas)...")
    sim_csv = run_simulation()
    print(f"Log da simulação: {sim_csv}")

    print("==> Carregando log da simulação...")
    sim_curves = load_sim_log(sim_csv)

    print("==> Extraindo curvas por eixo do SWV (com filtros)...")
    swv_curves = load_swv_per_axis(SWV_PATH)

    print("==> Gerando figura comparativa...")
    metrics = make_comparison_plot(swv_curves, sim_curves, OUT_IMG)
    print(f"Figura gerada em: {OUT_IMG}")
    for axis in (0, 1, 2):
        if axis not in metrics:
            continue
        m = metrics[axis]
        print(
            f"Eixo {axis}: medido={m['v_meas']:.1f} steps/s, "
            f"simulação={m['v_sim']:.1f} steps/s, "
            f"erro={m['err_abs']:.1f} steps/s ({m['err_pct']:.2f} %)."
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
