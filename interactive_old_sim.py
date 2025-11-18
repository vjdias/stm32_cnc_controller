#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Interactive OLD Sim
Redireciona para InteractiveSim com parâmetros de compatibilidade "antigos",
usando os MESMOS ganhos/tempos/cargas do sim atual para comparação justa.
"""
from __future__ import annotations
import argparse
from pathlib import Path

from tuning_profiles import AnalysisCatalog
from interactive_sim import (
    PlantConfig,
    Scenario,
    parse_axis_map,
    parse_pid_triple,
    gains_from_catalog,
    InteractiveSim,
)


def main():
    parser = argparse.ArgumentParser(description="Interactive OLD Sim (compat mode)")
    parser.add_argument(
        "--axes",
        default="X:256,Y:256,Z:256",
        help="Mapa de eixos X:micro,Y:micro (default: 1/256 para X/Y/Z).",
    )
    parser.add_argument(
        "--log-dir",
        default="sim_logs",
        help="Diretório de logs CSV (default: sim_logs).",
    )
    parser.add_argument(
        "--no-log",
        action="store_true",
        help="Desativa o log em arquivo",
    )
    parser.add_argument(
        "--auto-analyze",
        action="store_true",
        help="Analisa automaticamente o CSV ao finalizar",
    )
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
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Roda sem GUI (usa backend Agg)",
    )
    parser.add_argument(
        "--show-friction-band",
        action="store_true",
        help="Mostra faixa vertical vermelha durante aplicação de atrito nos gráficos.",
    )
    parser.add_argument(
        "--no-friction",
        action="store_true",
        help="Desabilita atrito (C e B) na simulação.",
    )
    args = parser.parse_args()

    axis_map = parse_axis_map(args.axes)
    kp_xyz, ki_xyz, kd_xyz = gains_from_catalog(axis_map)
    if args.kp is not None:
        kp_xyz = parse_pid_triple(args.kp)
    if args.ki is not None:
        ki_xyz = parse_pid_triple(args.ki)
    if args.kd is not None:
        kd_xyz = parse_pid_triple(args.kd)

    cfg = PlantConfig(
        microstep_factor=axis_map[0][1],
        enc_cpr_xyz=(40000, 2500, 40000),
        kd_alpha_bits=8,
        step_high_ticks=1,
        step_low_ticks=1,
    )
    # Mesmos parâmetros de cenário que o sim atual
    scn = Scenario(
        s_xyz=(20000, 16000, 12000),
        v_xyz=(10000, 8000, 6000),
        dir_xyz=(1, 1, 1),
        kp_xyz=kp_xyz,
        ki_xyz=ki_xyz,
        kd_xyz=kd_xyz,
        sim_time_s=5.0,
        use_dda=True,
    )

    sim = InteractiveSim(
        cfg,
        scn,
        log_dir=Path(args.log_dir),
        enable_logging=not args.no_log,
        auto_analyze=args.auto_analyze,
        headless=args.headless,
        show_friction_band=bool(args.show_friction_band),
    )

    if args.no_friction:
        try:
            # Zera atrito interno
            sim.C_load_values[:] = 0.0
            sim.B_load[:] = 0.0
            sim.load_start_times[:] = 0.0
            sim.load_end_times[:] = 0.0
            # Se houver GUI, limpa também os inputs de atrito e tempos
            if not args.headless:
                sim.txt_c_x.set_val("0.0")
                sim.txt_c_y.set_val("0.0")
                sim.txt_c_z.set_val("0.0")
                sim.txt_t_start_x.set_val("0.0")
                sim.txt_t_start_y.set_val("0.0")
                sim.txt_t_start_z.set_val("0.0")
                sim.txt_t_end_x.set_val("0.0")
                sim.txt_t_end_y.set_val("0.0")
                sim.txt_t_end_z.set_val("0.0")
        except Exception:
            pass

    # Configuração de compatibilidade "antiga"
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

    if args.headless:
        # Headless: roda direto (mesma rotina do interactive_sim headless)
        sim._start_log_session()
        # Snapshot inicial
        sim._log_state(
            t=sim.t,
            pos_steps=sim.pos_real.copy(),
            pos_enc=sim._encoder_rel_dda(),
            vel_sps=sim.v_real.copy(),
            casc_err=sim.g_casc_err_s32.copy(),
            load_c=sim.active_C_load.copy(),
            load_timer=sim.load_timer_xyz.copy(),
            span_steps=0.0,
            global_stop=False,
        )
        while sim.k < sim.N_steps_total:
            sim._step()
        sim._stop_log_session()
        if args.auto_analyze:
            sim._auto_analyze_last_log()
        print("Interactive OLD (headless) concluído.")
    else:
        # Usa a GUI do InteractiveSim
        import matplotlib.pyplot as plt
        plt.show()
        print("Interactive OLD (GUI) encerrado.")


if __name__ == '__main__':
    main()

