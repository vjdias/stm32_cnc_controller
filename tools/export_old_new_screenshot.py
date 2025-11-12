#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gera um print (screenshot) lado a lado do simulador interativo
para os modos "NEW" (estratégia atual) e "OLD" (compat progress),
sem abrir a janela gráfica (backend Agg), e salva em:
  tcc/src/Cap03/interactive_old_new.png

Requisitos: numpy, matplotlib
"""
import os
import numpy as np

# Força backend offscreen antes de importar matplotlib/interactive_sim
os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib
import matplotlib.pyplot as plt

from pathlib import Path

# Importações do simulador
from interactive_sim import PlantConfig, Scenario, InteractiveSim


def build_sim_new() -> InteractiveSim:
    """Simulador com estratégia NOVA (padrões do firmware)."""
    cfg = PlantConfig(
        microstep_factor=256,
        enc_cpr_xyz=(40000, 2500, 40000),
        kd_alpha_bits=8,
        step_high_ticks=1,
        step_low_ticks=1,
    )
    scn = Scenario(
        s_xyz=(40000, 32000, 24000),
        v_xyz=(10000, 8000, 6000),
        dir_xyz=(1, 1, 1),
        # Ganhos serão sobrescritos pelo reset() inicial com valores de scn
        kp_xyz=(800, 800, 800),
        ki_xyz=(40, 40, 40),
        kd_xyz=(120, 120, 120),
        sim_time_s=2.5,  # mais rápido para gerar o print
        use_dda=True,
    )
    return InteractiveSim(cfg, scn, headless=False, enable_logging=False)


def build_sim_old() -> InteractiveSim:
    """Simulador com estratégia ANTIGA (progress, compat do interactive_old_sim)."""
    sim = build_sim_new()
    # Parâmetros de compatibilidade "antigos"
    sim.master_select_strategy = 'progress'            # mestre por menor progresso
    sim.prefer_loaded_master = False                  # não prioriza eixo carregado
    sim.master_switch_margin_steps = 0.0              # troca imediata de mestre
    sim.sync_err_feed_threshold = 200.0               # throttle agressivo
    sim.sync_err_feed_min_fraction = 0.25             # piso baixo do feed
    sim.sync_hold_enabled = False                     # sem trava de sincronismo
    sim.finish_all_axes = False                       # termina quando mestre chega
    sim.ramp_use_worst_remaining = False              # rampa pelo mestre
    sim.global_stop_all_axes = True                   # stall/stop global
    sim.finish_window_steps = 0.0                     # sem fase final especial
    sim.finish_disable_stall = False
    sim.finish_extra_budget_steps = 0                 # sem extensão automática
    return sim


def run_to_end_and_render(sim: InteractiveSim):
    """Executa a simulação completa e traz a figura para um estado desenhável."""
    # Inicializa estados e limites dos gráficos
    sim.reset(None)
    # Avança a simulação (1ms por passo)
    while sim.k < sim.N_steps_total:
        sim._step()
        # Atualiza artistas de tempos em tempos para manter o buffer coerente
        if (sim.k % 20) == 0:
            sim._update_artists_data()
    # Atualiza uma última vez e força desenho síncrono
    sim._update_artists_data()
    if hasattr(sim.fig.canvas, 'draw'):
        sim.fig.canvas.draw()


def save_fig(fig, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)


def main():
    out_dir = Path("tcc/src/Cap03")
    out_old = out_dir / "interactive_old.png"
    out_new = out_dir / "interactive_new.png"
    out_combo = out_dir / "interactive_old_new.png"

    # Constrói e executa NEW
    sim_new = build_sim_new()
    run_to_end_and_render(sim_new)
    save_fig(sim_new.fig, out_new)

    # Constrói e executa OLD
    sim_old = build_sim_old()
    run_to_end_and_render(sim_old)
    save_fig(sim_old.fig, out_old)

    # Carrega as imagens e monta um mosaico lado a lado via numpy
    img_new = plt.imread(out_new)
    img_old = plt.imread(out_old)

    # Normaliza alturas: se diferentes, faz padding simples
    h = max(img_new.shape[0], img_old.shape[0])
    w_new = img_new.shape[1]
    w_old = img_old.shape[1]

    def pad_to_h(img, h):
        if img.shape[0] == h:
            return img
        pad = h - img.shape[0]
        # Pad inferior de zeros (branco ao salvar)
        if img.ndim == 3:
            pad_block = np.ones((pad, img.shape[1], img.shape[2]), dtype=img.dtype)
        else:
            pad_block = np.ones((pad, img.shape[1]), dtype=img.dtype)
        return np.vstack([img, pad_block])

    img_new_p = pad_to_h(img_new, h)
    img_old_p = pad_to_h(img_old, h)
    combo = np.hstack([img_old_p, img_new_p])

    # Salva o mosaico final
    plt.imsave(out_combo, combo)
    print(f"Figura composta salva em: {out_combo}")


if __name__ == "__main__":
    main()
