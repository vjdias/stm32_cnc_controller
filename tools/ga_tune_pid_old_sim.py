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

import concurrent.futures
import os

# Garante que o diretório raiz do repositório esteja no sys.path
import sys

_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

# Força modo headless/Agg para o simulador durante a otimização (evita Qt).
os.environ.setdefault("SIM_HEADLESS", "1")
os.environ.setdefault("MPLBACKEND", "Agg")

from interactive_sim import (  # type: ignore
    PlantConfig,
    Scenario,
    parse_axis_map,
    parse_pid_triple,
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
    # Pesos da função de custo:
    #   cost = w_iae*norm_iae + w_sat*sat_frac + w_zone*(1 - zone_frac) + w_smooth*norm_smooth
    # Prioriza fortemente ficar dentro da zona (w_zone maior), com suavidade de velocidade.
    w_iae: float = 1.0
    w_sat: float = 2.0
    w_zone: float = 5.0
    w_smooth: float = 1.0
    seed: int = 1234
    ls_every: int = 0        # 0 = só no final; 1 = toda geração; N = a cada N gerações
    ls_iters: int = 20       # iterações máximas da busca local
    workers: int = 0         # 0 = auto (todos núcleos), 1 = desabilita paralelismo
    friction_enabled: bool = True  # True = com atrito, False = sem atrito


def latin_hypercube_population(rng: random.Random, ga: GAParams) -> List[Tuple[float, float, float]]:
    """
    Gera uma população inicial bem espaçada em [scale_min, scale_max]^3
    usando amostragem em hipercubo latino (LHS).
    """
    dim = 3
    n = ga.pop_size
    # Para cada dimensão, criamos n intervalos e embaralhamos
    intervals = []
    for _ in range(dim):
        axis_vals = [(i + rng.random()) / n for i in range(n)]
        rng.shuffle(axis_vals)
        intervals.append(axis_vals)

    span = ga.scale_max - ga.scale_min
    pop: List[Tuple[float, float, float]] = []
    for i in range(n):
        genes = []
        for d in range(dim):
            u = intervals[d][i]
            val = ga.scale_min + u * span
            genes.append(val)
        pop.append(tuple(genes))  # type: ignore[arg-type]
    return pop


def _eval_candidate_star(args):
    """Wrapper para permitir uso com executor.map."""
    (
        axis_focus,
        axis_map,
        kp_xyz_base,
        ki_xyz_base,
        kd_xyz_base,
        ind,
        ga,
        enc_cpr_y,
        sim_time_s,
    ) = args
    return evaluate_candidate(
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


def evaluate_population_parallel(
    pop: List[Tuple[float, float, float]],
    axis_focus: int | None,
    axis_map: List[Tuple[str, int]],
    kp_xyz_base: Tuple[int, int, int],
    ki_xyz_base: Tuple[int, int, int],
    kd_xyz_base: Tuple[int, int, int],
    ga: GAParams,
    *,
    enc_cpr_y: int,
    sim_time_s: float,
) -> List[Tuple[float, dict]]:
    """
    Avalia toda a população em paralelo usando todos os núcleos disponíveis.

    Retorna lista alinhada com 'pop' contendo (cost, metrics) por indivíduo.
    """
    args_list = [
        (
            axis_focus,
            axis_map,
            kp_xyz_base,
            ki_xyz_base,
            kd_xyz_base,
            ind,
            ga,
            enc_cpr_y,
            sim_time_s,
        )
        for ind in pop
    ]

    # workers <= 1: modo sequencial (útil para debug ou ambientes limitados)
    if ga.workers <= 1:
        return [_eval_candidate_star(args) for args in args_list]

    max_workers = None if ga.workers <= 0 else ga.workers
    # Usa ProcessPoolExecutor para contornar o GIL e paralelizar CPU-bound
    with concurrent.futures.ProcessPoolExecutor(max_workers=max_workers) as ex:
        results = list(ex.map(_eval_candidate_star, args_list))
    return results


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
    axis_focus: int | None,
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

    - Se axis_focus for 0/1/2: aplica escalas apenas nesse eixo e calcula métricas
      (IAE/sat/zone) só para ele.
    - Se axis_focus for None: aplica escalas a TODOS os eixos e calcula métricas
      agregadas (IAE somado, saturação se QUALQUER eixo satura, zona se TODOS
      os eixos estão na faixa desejada).

    scales: (scale_kp, scale_ki, scale_kd) aplicados sobre os ganhos base.
    """
    # Decodifica ganhos inteiros
    kp_xyz = list(base_kp_xyz)
    ki_xyz = list(base_ki_xyz)
    kd_xyz = list(base_kd_xyz)

    def _clamp_int(v: float, base: int) -> int:
        # Limita para evitar valores absurdos, mas ainda permitindo exploração
        lo = max(0, int(round(0.1 * base)))
        hi = int(round(5.0 * max(1, base)))
        return max(lo, min(hi, int(round(v))))

    if axis_focus is None:
        # Aplica a MESMA escala em todos os eixos
        for ax in range(3):
            kp_xyz[ax] = _clamp_int(base_kp_xyz[ax] * scales[0], base_kp_xyz[ax])
            ki_xyz[ax] = _clamp_int(base_ki_xyz[ax] * scales[1], max(1, base_ki_xyz[ax]))
            kd_xyz[ax] = _clamp_int(base_kd_xyz[ax] * scales[2], max(1, base_kd_xyz[ax]))
    else:
        # Aplica escala apenas no eixo em foco
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

    # Opcionalmente desabilita atrito (estático + viscoso) na simulação
    if not ga.friction_enabled:
        try:
            sim.C_load_values[:] = 0.0
            sim.B_load[:] = 0.0
        except Exception:
            # Se por algum motivo os atributos não existirem, ignora silenciosamente
            pass

    max_sps = sim.cfg.max_sps
    target_v_all = [float(v) for v in sim.v_target_sps]
    if axis_focus is None:
        zone_low = [ga.zone_low_factor * v for v in target_v_all]
        zone_high = [ga.zone_high_factor * v for v in target_v_all]
    else:
        target_v = target_v_all[axis_focus]
        zone_low = ga.zone_low_factor * target_v
        zone_high = ga.zone_high_factor * target_v
    sat_threshold = ga.sat_frac * max_sps

    n_steps = sim.N_steps_total
    sat_count = 0
    zone_good = 0
    # Termo de suavidade: acumula variação absoluta de velocidade entre steps
    smooth_acc = 0.0
    if axis_focus is None:
        v_prev_vec = [0.0, 0.0, 0.0]
    else:
        v_prev = 0.0

    for _ in range(n_steps):
        sim._step()
        # --- Suavidade (variação de velocidade) ---
        if axis_focus is None:
            v_vec = [float(x) for x in sim.v_real]
            smooth_acc += sum(abs(v_vec[ax] - v_prev_vec[ax]) for ax in range(3))
            v_prev_vec = v_vec
        else:
            v_cur = float(sim.v_real[axis_focus])
            smooth_acc += abs(v_cur - v_prev)
            v_prev = v_cur

        if axis_focus is None:
            v_vec = [float(x) for x in sim.v_real]
            # Saturação se QUALQUER eixo saturar
            if any(v >= sat_threshold for v in v_vec):
                sat_count += 1
            # Zona "boa" apenas se TODOS os eixos estiverem na faixa
            in_zone = True
            for ax in range(3):
                if not (zone_low[ax] <= v_vec[ax] <= zone_high[ax]):
                    in_zone = False
                    break
            if in_zone:
                zone_good += 1
        else:
            v = float(sim.v_real[axis_focus])
            if v >= sat_threshold:
                sat_count += 1
            if zone_low <= v <= zone_high:
                zone_good += 1

    if axis_focus is None:
        iae = float(sum(float(x) for x in sim.err_accum_xyz))
        target_steps = float(
            sum(abs(int(s)) for s in sim.target_s32)
        )
    else:
        iae = float(sim.err_accum_xyz[axis_focus])
        target_steps = float(abs(sim.target_s32[axis_focus]))

    sat_frac = sat_count / float(max(1, n_steps))
    zone_frac = zone_good / float(max(1, n_steps))

    # Normaliza IAE usando o tamanho do movimento
    norm_iae = iae / max(1.0, target_steps * sim.scn.sim_time_s)

    # Normaliza suavidade aproximando pelo range típico de velocidade
    if axis_focus is None:
        smooth_den = max(1.0, max_sps * n_steps * 3)
    else:
        smooth_den = max(1.0, max_sps * n_steps)
    norm_smooth = smooth_acc / smooth_den

    cost = (
        ga.w_iae * norm_iae
        + ga.w_sat * sat_frac
        + ga.w_zone * (1.0 - zone_frac)
        + ga.w_smooth * norm_smooth
    )

    metrics = {
        "iae": iae,
        "norm_iae": norm_iae,
        "sat_frac": sat_frac,
        "zone_frac": zone_frac,
        "smooth": smooth_acc,
        "norm_smooth": norm_smooth,
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
    fixed_kp: Tuple[int, int, int] | None = None,
    fixed_ki: Tuple[int, int, int] | None = None,
    fixed_kd: Tuple[int, int, int] | None = None,
) -> None:
    axis_focus_letter = axis_focus_letter.upper()
    if axis_focus_letter == "ALL":
        axis_focus: int | None = None
    else:
        if axis_focus_letter not in AXIS_ID:
            raise SystemExit(f"Eixo inválido para --axis: {axis_focus_letter}")
        axis_focus = AXIS_ID[axis_focus_letter]

    kp_xyz_base, ki_xyz_base, kd_xyz_base = gains_from_catalog(axis_map)
    # Permite sobrepor ganhos fixos para eixos não focados
    kp_xyz_base = list(kp_xyz_base)
    ki_xyz_base = list(ki_xyz_base)
    kd_xyz_base = list(kd_xyz_base)
    if fixed_kp is not None:
        for ax in range(3):
            if axis_focus is None or ax != axis_focus:
                kp_xyz_base[ax] = fixed_kp[ax]
    if fixed_ki is not None:
        for ax in range(3):
            if axis_focus is None or ax != axis_focus:
                ki_xyz_base[ax] = fixed_ki[ax]
    if fixed_kd is not None:
        for ax in range(3):
            if axis_focus is None or ax != axis_focus:
                kd_xyz_base[ax] = fixed_kd[ax]

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
        # Avalia toda a população (potencialmente em paralelo) e pré-preenche o cache
        results = evaluate_population_parallel(
            pop,
            axis_focus,
            axis_map,
            kp_xyz_base,
            ki_xyz_base,
            kd_xyz_base,
            ga,
            enc_cpr_y=enc_cpr_y,
            sim_time_s=sim_time_s,
        )
        for ind, (cost, metrics) in zip(pop, results):
            key = tuple(round(x, 4) for x in ind)
            fitness_cache[key] = (cost, metrics)

        evaluated = [(_fitness(ind)[0], ind) for ind in pop]
        evaluated.sort(key=lambda x: x[0])
        gen_best_cost, gen_best_ind = evaluated[0]
        gen_best_metrics = _fitness(gen_best_ind)[1]
        if gen_best_cost < best_cost:
            best_cost = gen_best_cost
            best_ind = gen_best_ind
            best_metrics = gen_best_metrics

        if axis_focus is not None:
            kp_axis = gen_best_metrics["kp_xyz"][axis_focus]
            ki_axis = gen_best_metrics["ki_xyz"][axis_focus]
            kd_axis = gen_best_metrics["kd_xyz"][axis_focus]
            k_info = f"Kp={kp_axis} Ki={ki_axis} Kd={kd_axis}"
        else:
            k_info = (
                f"kp_xyz={gen_best_metrics['kp_xyz']} "
                f"ki_xyz={gen_best_metrics['ki_xyz']} "
                f"kd_xyz={gen_best_metrics['kd_xyz']}"
            )

        print(
            f"[GEN {gen+1:02d}] best_cost={gen_best_cost:.6f} "
            f"iae={gen_best_metrics['iae']:.2f} "
            f"sat={gen_best_metrics['sat_frac']*100:.1f}% "
            f"zone={gen_best_metrics['zone_frac']*100:.1f}% "
            f"smooth={gen_best_metrics['norm_smooth']:.3f} "
            f"{k_info}"
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


def local_search(
    start_ind: Tuple[float, float, float],
    fitness_fn,
    ga: GAParams,
    *,
    base_step: float = 0.2,
    max_iters: int = 20,
) -> Tuple[Tuple[float, float, float], float, dict]:
    """
    Busca local por coordenadas em torno do melhor indivíduo.

    Aproxima um "gradiente" discreto: testa variações positivas/negativas
    em cada gene e aceita apenas movimentos que reduzem o custo.
    """
    current = start_ind
    current_cost, current_metrics = fitness_fn(current)
    step = base_step

    for _ in range(max_iters):
        improved = False
        for g_idx in range(3):
            g_val = current[g_idx]
            for direction in (+1.0, -1.0):
                g_new = g_val * (1.0 + direction * step)
                g_new = max(ga.scale_min, min(ga.scale_max, g_new))
                cand = list(current)
                cand[g_idx] = g_new
                cand_t = tuple(cand)  # type: ignore[assignment]
                c_cost, c_metrics = fitness_fn(cand_t)
                if c_cost < current_cost:
                    current, current_cost, current_metrics = cand_t, c_cost, c_metrics
                    improved = True
        if not improved:
            step *= 0.5
            if step < 0.01:
                break

    return current, current_cost, current_metrics


def run_memetic(
    axis_focus_letter: str,
    axis_map: List[Tuple[str, int]],
    ga: GAParams,
    *,
    enc_cpr_y: int,
    sim_time_s: float,
    fixed_kp: Tuple[int, int, int] | None = None,
    fixed_ki: Tuple[int, int, int] | None = None,
    fixed_kd: Tuple[int, int, int] | None = None,
) -> None:
    """
    Variante "memética": população inicial bem espaçada (LHS)
    + GA global + refino local tipo gradiente no melhor indivíduo.
    """
    axis_focus_letter = axis_focus_letter.upper()
    if axis_focus_letter == "ALL":
        axis_focus: int | None = None
    else:
        if axis_focus_letter not in AXIS_ID:
            raise SystemExit(f"Eixo inválido para --axis: {axis_focus_letter}")
        axis_focus = AXIS_ID[axis_focus_letter]

    kp_xyz_base, ki_xyz_base, kd_xyz_base = gains_from_catalog(axis_map)
    # Permite sobrepor ganhos fixos para eixos não focados
    kp_xyz_base = list(kp_xyz_base)
    ki_xyz_base = list(ki_xyz_base)
    kd_xyz_base = list(kd_xyz_base)
    if fixed_kp is not None:
        for ax in range(3):
            if axis_focus is None or ax != axis_focus:
                kp_xyz_base[ax] = fixed_kp[ax]
    if fixed_ki is not None:
        for ax in range(3):
            if axis_focus is None or ax != axis_focus:
                ki_xyz_base[ax] = fixed_ki[ax]
    if fixed_kd is not None:
        for ax in range(3):
            if axis_focus is None or ax != axis_focus:
                kd_xyz_base[ax] = fixed_kd[ax]

    rng = random.Random(ga.seed)

    pop: List[Tuple[float, float, float]] = latin_hypercube_population(rng, ga)
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
        # Avalia toda a população (potencialmente em paralelo) e pré-preenche o cache
        results = evaluate_population_parallel(
            pop,
            axis_focus,
            axis_map,
            kp_xyz_base,
            ki_xyz_base,
            kd_xyz_base,
            ga,
            enc_cpr_y=enc_cpr_y,
            sim_time_s=sim_time_s,
        )
        for ind, (cost, metrics) in zip(pop, results):
            key = tuple(round(x, 4) for x in ind)
            fitness_cache[key] = (cost, metrics)

        evaluated = [(_fitness(ind)[0], ind) for ind in pop]
        evaluated.sort(key=lambda x: x[0])
        gen_best_cost, gen_best_ind = evaluated[0]
        gen_best_metrics = _fitness(gen_best_ind)[1]

        if gen_best_cost < best_cost:
            best_cost = gen_best_cost
            best_ind = gen_best_ind
            best_metrics = gen_best_metrics

        if axis_focus is not None:
            kp_axis = gen_best_metrics["kp_xyz"][axis_focus]
            ki_axis = gen_best_metrics["ki_xyz"][axis_focus]
            kd_axis = gen_best_metrics["kd_xyz"][axis_focus]
            k_info = f"Kp={kp_axis} Ki={ki_axis} Kd={kd_axis}"
        else:
            k_info = (
                f"kp_xyz={gen_best_metrics['kp_xyz']} "
                f"ki_xyz={gen_best_metrics['ki_xyz']} "
                f"kd_xyz={gen_best_metrics['kd_xyz']}"
            )

        print(
            f"[GEN {gen+1:02d}] best_cost={gen_best_cost:.6f} "
            f"iae={gen_best_metrics['iae']:.2f} "
            f"sat={gen_best_metrics['sat_frac']*100:.1f}% "
            f"zone={gen_best_metrics['zone_frac']*100:.1f}% "
            f"smooth={gen_best_metrics['norm_smooth']:.3f} "
            f"{k_info}"
        )

        # Busca local opcional ao final de algumas gerações
        if ga.ls_every > 0 and ((gen + 1) % ga.ls_every == 0):
            print(f"[LOCAL SEARCH] Refino em torno do melhor da geração {gen+1}...")
            ls_ind, ls_cost, ls_metrics = local_search(
                gen_best_ind,
                _fitness,
                ga,
                max_iters=ga.ls_iters,
            )
            # Atualiza melhor da geração / global se houver melhora
            if ls_cost < gen_best_cost:
                gen_best_cost, gen_best_ind, gen_best_metrics = ls_cost, ls_ind, ls_metrics
            if ls_cost < best_cost:
                best_cost, best_ind, best_metrics = ls_cost, ls_ind, ls_metrics
            # Injeta indivíduo refinado na população (substitui o pior)
            worst_idx = max(range(len(pop)), key=lambda i: _fitness(pop[i])[0])
            pop[worst_idx] = ls_ind

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

    # Refino final obrigatório, mesmo se ls_every==0 (só no final)
    print("\n[LOCAL SEARCH] Iniciando refino final em torno do melhor indivíduo global...")
    ls_ind, ls_cost, ls_metrics = local_search(
        best_ind,
        _fitness,
        ga,
        max_iters=ga.ls_iters,
    )

    # Se o refino melhorou, substitui
    if ls_cost < best_cost:
        best_cost, best_ind, best_metrics = ls_cost, ls_ind, ls_metrics

    print("\n==== Melhores resultados GA (antes do refino local) ====")
    print(f"Eixo foco: {axis_focus_letter}")
    print(f"Escalas (Kp, Ki, Kd): {best_ind}")
    print(
        f"Custo={best_cost:.6f} | IAE={best_metrics['iae']:.2f} "
        f"| sat={best_metrics['sat_frac']*100:.2f}% "
        f"| zona={best_metrics['zone_frac']*100:.2f}%"
    )

    print("\n==== Melhor indivíduo após refino local ====")
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
        help="Eixo em foco (X, Y, Z ou ALL para considerar os três). Default: Y.",
    )
    p.add_argument(
        "--fixed-kp",
        default=None,
        help="Ganhos Kp fixos para eixos não focados (formato X:800,Y:800,Z:800 ou 800,800,800).",
    )
    p.add_argument(
        "--fixed-ki",
        default=None,
        help="Ganhos Ki fixos para eixos não focados (formato X:40,Y:40,Z:40 ou 40,40,40).",
    )
    p.add_argument(
        "--fixed-kd",
        default=None,
        help="Ganhos Kd fixos para eixos não focados (formato X:120,Y:120,Z:120 ou 120,120,120).",
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
    p.add_argument(
        "--no-friction",
        action="store_true",
        help="Desabilita atrito (C e B) na simulação do InteractiveSim durante a otimização.",
    )
    p.add_argument(
        "--ls-every",
        type=int,
        default=0,
        help="Frequência da busca local no modo memetic: 0=só final, 1=toda geração, N=a cada N gerações.",
    )
    p.add_argument(
        "--ls-iters",
        type=int,
        default=20,
        help="Iterações máximas da busca local (default: 20).",
    )
    p.add_argument(
        "--workers",
        type=int,
        default=0,
        help=(
            "Número de processos em paralelo para avaliar a população: "
            "0=auto (todos núcleos), 1=sem paralelismo, N=força N workers."
        ),
    )
    p.add_argument(
        "--mode",
        choices=["ga", "memetic"],
        default="ga",
        help="Algoritmo: GA simples ou GA + busca local (memetic).",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    axis_map = parse_axis_map(args.axes)

    fixed_kp = fixed_ki = fixed_kd = None
    if args.fixed_kp:
        fixed_kp = parse_pid_triple(args.fixed_kp)
    if args.fixed_ki:
        fixed_ki = parse_pid_triple(args.fixed_ki)
    if args.fixed_kd:
        fixed_kd = parse_pid_triple(args.fixed_kd)

    ga = GAParams(
        pop_size=args.pop_size,
        generations=args.generations,
        zone_low_factor=args.zone_low,
        zone_high_factor=args.zone_high,
        sat_frac=args.sat_frac,
        seed=args.seed,
        ls_every=args.ls_every,
        ls_iters=args.ls_iters,
        workers=args.workers,
        friction_enabled=not args.no_friction,
    )
    if args.mode == "ga":
        run_ga(
            args.axis,
            axis_map,
            ga,
            enc_cpr_y=args.enc_cpr_y,
            sim_time_s=args.sim_time,
            fixed_kp=fixed_kp,
            fixed_ki=fixed_ki,
            fixed_kd=fixed_kd,
        )
    else:
        run_memetic(
            args.axis,
            axis_map,
            ga,
            enc_cpr_y=args.enc_cpr_y,
            sim_time_s=args.sim_time,
            fixed_kp=fixed_kp,
            fixed_ki=fixed_ki,
            fixed_kd=fixed_kd,
        )


if __name__ == "__main__":
    main()
