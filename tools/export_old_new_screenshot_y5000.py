#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gera prints lado a lado do simulador interativo (NEW vs OLD)
com encoder Y=5000 (X/Z=40000) e salva em tcc/src/Cap03/interactive_old_new_y5000.png
"""
import os
import numpy as np

os.environ.setdefault("MPLBACKEND", "Agg")

import matplotlib.pyplot as plt
from pathlib import Path

from interactive_sim import PlantConfig, Scenario, InteractiveSim


def build_sim_new() -> InteractiveSim:
    cfg = PlantConfig(
        microstep_factor=256,
        enc_cpr_xyz=(40000, 5000, 40000),
        kd_alpha_bits=8,
        step_high_ticks=1,
        step_low_ticks=1,
    )
    scn = Scenario(
        s_xyz=(40000, 32000, 24000),
        v_xyz=(10000, 8000, 6000),
        dir_xyz=(1, 1, 1),
        kp_xyz=(800, 800, 800),
        ki_xyz=(40, 40, 40),
        kd_xyz=(120, 120, 120),
        sim_time_s=2.5,
        use_dda=True,
    )
    return InteractiveSim(cfg, scn, headless=False, enable_logging=False)


def build_sim_old() -> InteractiveSim:
    sim = build_sim_new()
    sim.master_select_strategy = 'progress'
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


def run_to_end_and_render(sim: InteractiveSim):
    sim.reset(None)
    while sim.k < sim.N_steps_total:
        sim._step()
        if (sim.k % 20) == 0:
            sim._update_artists_data()
    sim._update_artists_data()
    if hasattr(sim.fig.canvas, 'draw'):
        sim.fig.canvas.draw()


def save_fig(fig, path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=120)


def main():
    out_dir = Path("tcc/src/Cap03")
    out_old = out_dir / "interactive_old_y5000.png"
    out_new = out_dir / "interactive_new_y5000.png"
    out_combo = out_dir / "interactive_old_new_y5000.png"

    sim_new = build_sim_new()
    run_to_end_and_render(sim_new)
    save_fig(sim_new.fig, out_new)

    sim_old = build_sim_old()
    run_to_end_and_render(sim_old)
    save_fig(sim_old.fig, out_old)

    img_new = plt.imread(out_new)
    img_old = plt.imread(out_old)

    h = max(img_new.shape[0], img_old.shape[0])
    def pad_to_h(img, h):
        if img.shape[0] == h:
            return img
        pad = h - img.shape[0]
        if img.ndim == 3:
            pad_block = np.ones((pad, img.shape[1], img.shape[2]), dtype=img.dtype)
        else:
            pad_block = np.ones((pad, img.shape[1]), dtype=img.dtype)
        return np.vstack([img, pad_block])

    combo = np.hstack([pad_to_h(img_old, h), pad_to_h(img_new, h)])
    plt.imsave(out_combo, combo)
    print(f"Figura composta salva em: {out_combo}")


if __name__ == "__main__":
    main()

