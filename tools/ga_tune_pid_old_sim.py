#!/usr/bin/env python3
"""
Busca (via algoritmo genético simples) os melhores valores inteiros de
Kp, Ki e Kd para um eixo do OLD Sim, tentando:

- Minimizar IAE (integral do erro absoluto, em steps·s).
- Minimizar o tempo com velocidade saturada em 25000 steps/s.
- Manter a maior parte do tempo dentro de uma "zona" em torno do feed alvo.

O script usa o InteractiveSim em modo headless com a configuração
de compatibilidade do interactive_old_sim.py.

Exemplo de uso (otimizando eixo Y @ 1/256, Y=5000 CPR):

    python tools/ga_tune_pid_old_sim.py --axis Y --axes X:256,Y:256,Z:256 \\
        --generations 20 --pop-size 24

Ao final ele imprime o melhor indivíduo encontrado e os kp/ki/kd inteiros
para uso direto no OLD Sim (via --kp/--ki/--kd) ou no firmware.
"""
from __future__ import annotations

import argparse
import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

# Garante que o diretório raiz do repositório esteja no sys.path
import sys

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from interactive_sim import (  # type: ignore
    PlantConfig,
    Scenario,
    parse_axis_map,
    gains_from_catalog,
    InteractiveSim,
)


AXIS_ID = {"X": 0, "Y": 1, "Z": 2}


@dataclass
class GAParams:
    pop_size: int = 24
    generations: int = 20
    cx_prob: float = 0.7
    mut_prob: float = 0.3
    mut_sigma: float = 0.25
    scale_min: float = 0.3
    scale_max: float = 3.0
    zone_low_factor: float = 0.5
    zone_high_factor: float = 1.5
    sat_frac: float = 0.96
    w_iae: float = 1.0
    w_sat: float = 5.0
    w_zone: float = 1.0
    seed: int = 1234


def build_sim(
    axis_map: List[Tuple[str, int]],
    kp_xyz: Tuple[int, int, int],
    ki_xyz: Tuple[int, int, int],
    kd_xyz: Tuple[int, int, int],
    *,
    enc_cpr_y: int,
    sim_time_s: float,
) -> InteractiveSim:
    """Constrói um InteractiveSim headless configurado como OLD Sim."""
    cfg = PlantConfig(
        microstep_factor=axis_map[0][1],
        enc_cpr_xyz=(40000, enc_cpr_y, 40000),
        kd_alpha_bits=8,
        step_high_ticks=1,
        step_low_ticks=1,
    )
    scn = Scenario(
        s_xyz=(20000, 16000, 12000),
        v_xyz=(10000, 8000, 6000),
        dir_xyz=(1, 1, 1),
        kp_xyz=kp_xyz,
        ki_xyz=ki_xyz,
        kd_xyz=kd_xyz,
        sim_time_s=sim_time_s,
        use_dda=True,
    )

    sim = InteractiveSim(
        cfg,
        scn,
        log_dir=Path("sim_logs"),
        enable_logging=False,
        auto_analyze=False,
        headless=True,
        show_friction_band=False,
    )

    # Mesma configuração de compatibilidade do interactive_old_sim.py
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


def evaluate_candidate(
    axis_focus: int,
    axis_map: List[Tuple[str, int]],
    base_kp_xyz: Tuple[int, int, int],
    base_ki_xyz: Tuple[int, int, int],
    base_kd_xyz: Tuple[int, int, int],
    scales: Tuple[float, float, float],
    ga: GAParams,
    *,
    enc_cpr_y: int,
    sim_time_s: float,
) -> Tuple[float, dict]:
    """
    Roda o OLD Sim para um indivíduo (scales -> kp/ki/kd) e retorna o custo.

    scales: (scale_kp, scale_ki, scale_kd) aplicados sobre os ganhos base
            do eixo em foco.
    """
    # Decodifica ganhos inteiros (apenas para o eixo de interesse)
    kp_xyz = list(base_kp_xyz)
    ki_xyz = list(base_ki_xyz)
    kd_xyz = list(base_kd_xyz)

    def _clamp_int(v: float, base: int) -> int:
        # Limita para evitar valores absurdos, mas ainda permitindo exploração
        lo = max(0, int(round(0.1 * base)))
        hi = int(round(5.0 * max(1, base)))
        return max(lo, min(hi, int(round(v))))

    kp_xyz[axis_focus] = _clamp_int(base_kp_xyz[axis_focus] * scales[0], base_kp_xyz[axis_focus])
    ki_xyz[axis_focus] = _clamp_int(base_ki_xyz[axis_focus] * scales[1], max(1, base_ki_xyz[axis_focus]))
    kd_xyz[axis_focus] = _clamp_int(base_kd_xyz[axis_focus] * scales[2], max(1, base_kd_xyz[axis_focus]))

    sim = build_sim(
        axis_map,
        tuple(kp_xyz),
        tuple(ki_xyz),
        tuple(kd_xyz),
        enc_cpr_y=enc_cpr_y,
        sim_time_s=sim_time_s,
    )

    max_sps = sim.cfg.max_sps
    target_v = float(sim.v_target_sps[axis_focus])
    zone_low = ga.zone_low_factor * target_v
    zone_high = ga.zone_high_factor * target_v
    sat_threshold = ga.sat_frac * max_sps

    n_steps = sim.N_steps_total
    sat_count = 0
    zone_good = 0

    for _ in range(n_steps):
        sim._step()
        v = float(sim.v_real[axis_focus])
        if v >= sat_threshold:
            sat_count += 1
        if zone_low <= v <= zone_high:
            zone_good += 1

    iae = float(sim.err_accum_xyz[axis_focus])
    sat_frac = sat_count / float(max(1, n_steps))
    zone_frac = zone_good / float(max(1, n_steps))

    # Normaliza IAE usando o tamanho do movimento
    target_steps = float(abs(sim.target_s32[axis_focus]))
    norm_iae = iae / max(1.0, target_steps * sim.scn.sim_time_s)

    cost = (
        ga.w_iae * norm_iae
        + ga.w_sat * sat_frac
        + ga.w_zone * (1.0 - zone_frac)
    )

    metrics = {
        "iae": iae,
        "norm_iae": norm_iae,
        "sat_frac": sat_frac,
        "zone_frac": zone_frac,
        "kp_xyz": tuple(kp_xyz),
        "ki_xyz": tuple(ki_xyz),
        "kd_xyz": tuple(kd_xyz),
    }
    return cost, metrics


def random_individual(rng: random.Random, ga: GAParams) -> Tuple[float, float, float]:
    return (
        rng.uniform(ga.scale_min, ga.scale_max),
        rng.uniform(ga.scale_min, ga.scale_max),
        rng.uniform(ga.scale_min, ga.scale_max),
    )


def mutate(
    ind: Tuple[float, float, float],
    rng: random.Random,
    ga: GAParams,
) -> Tuple[float, float, float]:
    def _mutate_gene(g: float) -> float:
        if rng.random() < ga.mut_prob:
            g = g + rng.gauss(0.0, ga.mut_sigma)
        return float(min(ga.scale_max, max(ga.scale_min, g)))

    return (_mutate_gene(ind[0]), _mutate_gene(ind[1]), _mutate_gene(ind[2]))


def crossover(
    a: Tuple[float, float, float],
    b: Tuple[float, float, float],
    rng: random.Random,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
    if rng.random() > 0.5:
        alpha = rng.random()
        c0 = tuple(alpha * x + (1.0 - alpha) * y for x, y in zip(a, b))
        c1 = tuple(alpha * y + (1.0 - alpha) * x for x, y in zip(a, b))
        return c0, c1
    idx = rng.randint(0, 2)
    a_list = list(a)
    b_list = list(b)
    a_list[idx], b_list[idx] = b_list[idx], a_list[idx]
    return tuple(a_list), tuple(b_list)


def run_ga(
    axis_focus_letter: str,
    axis_map: List[Tuple[str, int]],
    ga: GAParams,
    *,
    enc_cpr_y: int,
    sim_time_s: float,
) -> None:
    axis_focus_letter = axis_focus_letter.upper()
    if axis_focus_letter not in AXIS_ID:
        raise SystemExit(f"Eixo inválido para --axis: {axis_focus_letter}")
    axis_focus = AXIS_ID[axis_focus_letter]

    kp_xyz_base, ki_xyz_base, kd_xyz_base = gains_from_catalog(axis_map)

    rng = random.Random(ga.seed)

    pop: List[Tuple[float, float, float]] = [
        random_individual(rng, ga) for _ in range(ga.pop_size)
    ]
    fitness_cache: dict = {}

    def _fitness(ind: Tuple[float, float, float]) -> Tuple[float, dict]:
        key = tuple(round(x, 4) for x in ind)
        if key not in fitness_cache:
            fitness_cache[key] = evaluate_candidate(
                axis_focus,
                axis_map,
                kp_xyz_base,
                ki_xyz_base,
                kd_xyz_base,
                ind,
                ga,
                enc_cpr_y=enc_cpr_y,
                sim_time_s=sim_time_s,
            )
        return fitness_cache[key]

    best_ind = None
    best_cost = math.inf
    best_metrics = {}

    for gen in range(ga.generations):
        evaluated = [(_fitness(ind)[0], ind) for ind in pop]
        evaluated.sort(key=lambda x: x[0])
        gen_best_cost, gen_best_ind = evaluated[0]
        gen_best_metrics = _fitness(gen_best_ind)[1]

        if gen_best_cost < best_cost:
            best_cost = gen_best_cost
            best_ind = gen_best_ind
            best_metrics = gen_best_metrics

        print(
            f"[GEN {gen+1:02d}] best_cost={gen_best_cost:.6f} "
            f"iae={gen_best_metrics['iae']:.2f} "
            f"sat={gen_best_metrics['sat_frac']*100:.1f}% "
            f"zone={gen_best_metrics['zone_frac']*100:.1f}%"
        )

        # Seleção por torneio simples
        new_pop: List[Tuple[float, float, float]] = []
        while len(new_pop) < ga.pop_size:
            a = rng.choice(pop)
            b = rng.choice(pop)
            fa, _ = _fitness(a)
            fb, _ = _fitness(b)
            parent1 = a if fa < fb else b

            c = rng.choice(pop)
            d = rng.choice(pop)
            fc, _ = _fitness(c)
            fd, _ = _fitness(d)
            parent2 = c if fc < fd else d

            if rng.random() < ga.cx_prob:
                child1, child2 = crossover(parent1, parent2, rng)
            else:
                child1, child2 = parent1, parent2

            child1 = mutate(child1, rng, ga)
            child2 = mutate(child2, rng, ga)
            new_pop.append(child1)
            if len(new_pop) < ga.pop_size:
                new_pop.append(child2)

        pop = new_pop

    if best_ind is None:
        print("Nenhum indivíduo avaliado, algo deu errado.")
        return

    print("\n==== Melhor indivíduo encontrado ====")
    print(f"Eixo foco: {axis_focus_letter}")
    print(f"Escalas (Kp, Ki, Kd): {best_ind}")
    print(
        f"Custo={best_cost:.6f} | IAE={best_metrics['iae']:.2f} "
        f"| sat={best_metrics['sat_frac']*100:.2f}% "
        f"| zona={best_metrics['zone_frac']*100:.2f}%"
    )

    kp_xyz = best_metrics["kp_xyz"]
    ki_xyz = best_metrics["ki_xyz"]
    kd_xyz = best_metrics["kd_xyz"]

    print("\nGanhos inteiros sugeridos (kp_xyz, ki_xyz, kd_xyz):")
    print(f"kp_xyz={kp_xyz}")
    print(f"ki_xyz={ki_xyz}")
    print(f"kd_xyz={kd_xyz}")

    print("\nStrings para usar no interactive_old_sim.py:")
    print(
        f"--kp \"X:{kp_xyz[0]},Y:{kp_xyz[1]},Z:{kp_xyz[2]}\" "
        f"--ki \"X:{ki_xyz[0]},Y:{ki_xyz[1]},Z:{ki_xyz[2]}\" "
        f"--kd \"X:{kd_xyz[0]},Y:{kd_xyz[1]},Z:{kd_xyz[2]}\""
    )


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Otimiza Kp/Ki/Kd para o OLD Sim via GA simples."
    )
    p.add_argument(
        "--axes",
        default="X:256,Y:256,Z:256",
        help="Mapa de eixos X:micro,Y:micro (default: X:256,Y:256,Z:256).",
    )
    p.add_argument(
        "--axis",
        default="Y",
        help="Eixo em foco (X, Y ou Z). Default: Y.",
    )
    p.add_argument(
        "--enc-cpr-y",
        type=int,
        default=5000,
        help="Encoder CPR do eixo Y (default: 5000, variante Y=5000).",
    )
    p.add_argument(
        "--sim-time",
        type=float,
        default=5.0,
        help="Tempo de simulação em segundos (default: 5.0).",
    )
    p.add_argument("--pop-size", type=int, default=24)
    p.add_argument("--generations", type=int, default=20)
    p.add_argument("--seed", type=int, default=1234)
    p.add_argument("--zone-low", type=float, default=0.5,
                   help="Fator inferior da zona relativa ao feed alvo (default: 0.5).")
    p.add_argument("--zone-high", type=float, default=1.5,
                   help="Fator superior da zona relativa ao feed alvo (default: 1.5).")
    p.add_argument("--sat-frac", type=float, default=0.96,
                   help="Fração de vmax considerada saturação (default: 0.96).")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    axis_map = parse_axis_map(args.axes)

    ga = GAParams(
        pop_size=args.pop_size,
        generations=args.generations,
        zone_low_factor=args.zone_low,
        zone_high_factor=args.zone_high,
        sat_frac=args.sat_frac,
        seed=args.seed,
    )

    run_ga(
        args.axis,
        axis_map,
        ga,
        enc_cpr_y=args.enc_cpr_y,
        sim_time_s=args.sim_time,
    )


if __name__ == "__main__":
    main()
