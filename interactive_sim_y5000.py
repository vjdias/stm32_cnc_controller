#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulador CASC/PID (Y=5000 CPR)
Versão do entrypoint do interactive_sim com encoder Y=5000 (X/Z=40000).
"""
from __future__ import annotations
import argparse
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

from interactive_sim import (
    PlantConfig,
    Scenario,
    parse_axis_map,
    parse_pid_triple,
    gains_from_catalog,
    InteractiveSim,
)


def main() -> None:
    parser = argparse.ArgumentParser(description="Simulador CASC/PID (Y=5000 CPR)")
    parser.add_argument("--axes", default="X:256,Y:256,Z:256")
    parser.add_argument("--log-dir", default="sim_logs")
    parser.add_argument("--no-log", action="store_true")
    parser.add_argument("--auto-analyze", action="store_true")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--kp",
        default=None,
        help=(
            "Ganhos Kp por eixo (inteiros do firmware). Formatos: "
            "'X:800,Y:800,Z:800' ou '800,800,800' ou '800'."
        ),
    )
    parser.add_argument(
        "--ki",
        default=None,
        help=(
            "Ganhos Ki por eixo (inteiros do firmware). Formatos: "
            "'X:40,Y:40,Z:40' ou '40,40,40' ou '40'."
        ),
    )
    parser.add_argument(
        "--kd",
        default=None,
        help=(
            "Ganhos Kd por eixo (inteiros do firmware). Formatos: "
            "'X:120,Y:120,Z:120' ou '120,120,120' ou '120'."
        ),
    )
    args = parser.parse_args()

    axis_map = parse_axis_map(args.axes)
    kp_xyz, ki_xyz, kd_xyz = gains_from_catalog(axis_map)
    # Overrides opcionais via CLI
    if args.kp is not None:
        kp_xyz = parse_pid_triple(args.kp)
    if args.ki is not None:
        ki_xyz = parse_pid_triple(args.ki)
    if args.kd is not None:
        kd_xyz = parse_pid_triple(args.kd)

    cfg = PlantConfig(
        microstep_factor=axis_map[0][1],
        enc_cpr_xyz=(40000, 5000, 40000),
        kd_alpha_bits=8,
        step_high_ticks=1,
        step_low_ticks=1,
    )

    scn = Scenario(
        s_xyz=(40000, 32000, 24000),
        v_xyz=(10000, 8000, 6000),
        dir_xyz=(1, 1, 1),
        kp_xyz=kp_xyz,
        ki_xyz=ki_xyz,
        kd_xyz=kd_xyz,
        sim_time_s=5.0,
        use_dda=True,
    )

    sim_app = InteractiveSim(
        cfg, scn,
        log_dir=Path(args.log_dir),
        enable_logging=not args.no_log,
        auto_analyze=args.auto_analyze,
        headless=args.headless,
    )
    if args.headless:
        sim_app._start_log_session()
        sim_app._log_state(
            t=sim_app.t,
            pos_steps=sim_app.pos_real.copy(),
            pos_enc=sim_app._encoder_rel_dda(),
            vel_sps=sim_app.v_real.copy(),
            casc_err=sim_app.g_casc_err_s32.copy(),
            load_c=sim_app.active_C_load.copy(),
            load_timer=sim_app.load_timer_xyz.copy(),
            span_steps=float(np.max(sim_app.pos_real) - np.min(sim_app.pos_real)),
            global_stop=False,
        )
        while sim_app.k < sim_app.N_steps_total:
            sim_app._step()
        sim_app._stop_log_session()
        if args.auto_analyze:
            sim_app._auto_analyze_last_log()
        print("Headless (Y=5000 CPR) concluído.")
    else:
        plt.show()
    print("Simulação interativa (Y=5000 CPR) encerrada.")


if __name__ == "__main__":
    main()
