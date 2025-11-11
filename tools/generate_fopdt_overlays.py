#!/usr/bin/env python3
from __future__ import annotations

import re
import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import math

import matplotlib
try:
    matplotlib.use("Agg")  # backend não interativo
    import matplotlib.pyplot as plt
except Exception as exc:  # pragma: no cover
    raise SystemExit("matplotlib é necessário (pip install matplotlib)") from exc


SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parent
SUMMARY_PATH = REPO_ROOT / "CNC_Controller" / "SWV_export" / "analysis_summary.txt"
DERIVED_DIR = REPO_ROOT / "CNC_Controller" / "SWV_export" / "analysis_data"
OUT_DIR = REPO_ROOT / "tcc" / "src" / "Cap03" / "img"


@dataclass
class Profile:
    axis_letter: str
    microstep: int
    k: float
    L: float
    tau: float
    derived_csv: Path


def parse_summary(path: Path) -> List[Profile]:
    profiles: List[Profile] = []
    axis_letter = None
    micro = None
    K = L = tau = None
    csv_path: Optional[Path] = None
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            m = re.search(r"Axis\s+([XYZ])\s+\(id\s+\d+\),\s+microstep\s+(\d+)", line)
            if m:
                axis_letter = m.group(1)
                micro = int(m.group(2))
                continue
            if line.startswith("G_v(s) params:"):
                # Example: G_v(s) params: K=6.1718, L=0.001000 s, τ=0.019000 s (t63=0.020000 s).
                mk = re.search(r"K=([0-9.]+)", line)
                mL = re.search(r"L=([0-9.]+)\s*s", line)
                mt = re.search(r"[τt]=([0-9.]+)\s*s", line)
                if mk and mL and mt:
                    K = float(mk.group(1))
                    L = float(mL.group(1))
                    tau = float(mt.group(1))
                continue
            if line.startswith("Derived CSV:"):
                p = line.split(":", 1)[1].strip()
                csv_path = Path(p)
                # finalize current profile
                if axis_letter and micro and K is not None and L is not None and tau is not None:
                    profiles.append(Profile(axis_letter, micro, K, L, tau, csv_path))
                # reset
                axis_letter = None
                micro = None
                K = L = tau = None
                csv_path = None
    return profiles


def load_derived(csv_path: Path) -> Tuple[List[float], List[float], List[float]]:
    times: List[float] = []
    vcmd: List[float] = []
    venc: List[float] = []
    with csv_path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader, None)
        # expect header; proceed even if missing
        for row in reader:
            if len(row) < 9:
                continue
            try:
                t = float(row[3])  # time_s
                vc = float(row[7]) if row[7] else float("nan")
                ve = float(row[8]) if row[8] else float("nan")
            except ValueError:
                continue
            times.append(t)
            vcmd.append(vc)
            venc.append(ve)
    return times, vcmd, venc


def steady_value(values: List[float], frac: float = 0.2) -> Optional[float]:
    vals = [v for v in values if v == v and not math.isinf(v)]
    if not vals:
        return None
    n = max(1, int(len(vals) * frac))
    tail = vals[-n:]
    return sum(tail) / len(tail)


def fopdt_step_response(times: List[float], K: float, L: float, tau: float, U: float) -> List[float]:
    y: List[float] = []
    for t in times:
        if t < L:
            y.append(0.0)
        else:
            y.append(K * U * (1.0 - math.exp(-(t - L) / max(tau, 1e-12))))
    return y


def run() -> int:
    if not SUMMARY_PATH.exists():
        print(f"Summary não encontrado: {SUMMARY_PATH}")
        return 1
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    profiles = parse_summary(SUMMARY_PATH)
    if not profiles:
        print("Nenhum perfil encontrado no summary.")
        return 2
    # Organizar por eixo
    by_axis: Dict[str, List[Profile]] = {"X": [], "Y": [], "Z": []}
    for p in profiles:
        if p.axis_letter in by_axis:
            by_axis[p.axis_letter].append(p)
    # Ordenar por microstep asc
    for axis in by_axis:
        by_axis[axis].sort(key=lambda pr: pr.microstep)

    # Gerar um PNG por perfil
    for p in profiles:
        if not p.derived_csv.exists():
            # tente localizar pelo nome base
            candidates = list(DERIVED_DIR.glob(f"{p.derived_csv.stem}*.csv"))
            if candidates:
                p.derived_csv = candidates[0]
        times, vcmd, venc = load_derived(p.derived_csv)
        if not times:
            print(f"Sem dados em {p.derived_csv}")
            continue
        U = steady_value(vcmd) or (max(vcmd) if vcmd else 0.0)
        yhat = fopdt_step_response(times, p.k, p.L, p.tau, U)

        plt.figure(figsize=(6, 3.2))
        plt.plot(times, venc, label="medido (v_enc)")
        plt.plot(times, yhat, "r--", label=f"FOPDT (K={p.k:.3f}, L={p.L*1e3:.1f} ms, ")
        plt.plot([], [], alpha=0)  # layout helper
        plt.title(f"Eixo {p.axis_letter} @ 1/{p.microstep}")
        plt.xlabel("tempo (s)")
        plt.ylabel("velocidade (cont./s)")
        plt.grid(True, alpha=0.3)
        plt.legend()
        out_name = f"overlay_{p.axis_letter}_mstep{p.microstep}.png"
        out_path = OUT_DIR / out_name
        plt.tight_layout()
        plt.savefig(out_path, dpi=130)
        plt.close()
        print(f"Gerado: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run())
