#!/usr/bin/env python3
"""
Analyze SWV step-response logs and derive control gains.

Workflow implemented per the engineering plan:
  1. Pre-process each *_filtered.csv to compute Delta_Tempo (s),
     Velocidade_Comando (pulsos STM/s) e Velocidade_Encoder (pulsos enc/s).
  2. Model the plant assuming G_v(s) = K * e^(-Ls) / (τ s + 1) by extracting
     the static gain K, dead time L and time constant τ from the velocity data.
  3. Synthesize PD gains via the Ziegler–Nichols reaction-curve rules
     (K_i = 0, controller de posição já integra).
  4. Emit a consolidated TXT report under SWV_export plus per-run CSVs with
     the derived columns to inspect em Excel/Matplotlib se necessário.

As derivadas são calculadas com uma janela deslizante de 10 linhas (configurável
via --stride) para reduzir ruído sem perder resolução temporal.

The script also cross-references motion_service.c constants (steps per rev,
microstepping, encoder counts) to estimate the theoretical gain and comment
on friction/ramp effects.
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import textwrap
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
import re

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT_DIR = SCRIPT_DIR / "CNC_Controller" / "SWV_export"
SAMPLE_STRIDE = 10  # janela deslizante padrão (10 linhas) para suavizar amostras

AXIS_NAMES = {0: "X", 1: "Y", 2: "Z"}
# motion_service.c -> ENC_COUNTS_PER_REV
ENC_COUNTS_PER_REV = {0: 40000, 1: 2500, 2: 40000}
# motion_service.c -> STEPS_PER_REV_BASE / MICROSTEP_FACTOR
STEPS_PER_REV_BASE = 400
DEFAULT_MICROSTEP = 256

DERIVED_HEADER = [
    "axis",
    "id",
    "time_ms",
    "time_s",
    "encoder",
    "pulses",
    "delta_t_s",
    "vel_cmd_sps",
    "vel_enc_sps",
]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Compute K/L/tau and PD gains from SWV *_filtered.csv files."
    )
    p.add_argument(
        "paths",
        nargs="*",
        help="Files or directories to scan (defaults to CNC_Controller/SWV_export).",
    )
    p.add_argument(
        "--summary",
        type=Path,
        default=None,
        help="Path to the TXT report (default: SWV_export/analysis_summary.txt).",
    )
    p.add_argument(
        "--derived-dir",
        type=Path,
        default=None,
        help="Directory for per-run derived CSVs (default: SWV_export/analysis_data).",
    )
    p.add_argument(
        "--no-derived",
        action="store_true",
        help="Do not emit per-run derived CSVs (only the summary).",
    )
    p.add_argument(
        "--steady-window",
        type=float,
        default=0.2,
        help="Fraction of samples considered steady-state (default: 0.2).",
    )
    p.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Print per-file progress to stdout.",
    )
    p.add_argument(
        "--stride",
        type=int,
        default=SAMPLE_STRIDE,
        help="Tamanho da janela deslizante (em linhas) usada nas derivadas (default: 10).",
    )
    return p.parse_args()


def discover_files(candidates: Sequence[str]) -> List[Path]:
    search_roots = list(candidates) if candidates else [str(DEFAULT_INPUT_DIR)]
    found: List[Path] = []
    seen = set()
    for raw in search_roots:
        p = Path(raw).expanduser()
        if not p.exists():
            print(f"[WARN] Ignoring missing path: {p}", file=sys.stderr)
            continue
        if p.is_dir():
            for csv_file in sorted(p.rglob("*_filtered.csv")):
                if csv_file not in seen:
                    found.append(csv_file)
                    seen.add(csv_file)
        elif p.is_file():
            if p.suffix.lower() == ".csv" and p.name.endswith("_filtered.csv"):
                if p not in seen:
                    found.append(p)
                    seen.add(p)
    return found


def load_rows(csv_path: Path) -> List[Dict[str, int]]:
    rows: List[Dict[str, int]] = []
    with csv_path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.reader(fh)
        for cols in reader:
            if len(cols) < 5:
                continue
            try:
                axis = int(cols[0], 10)
                seq = int(cols[1], 10)
                t_ms = int(cols[2], 10)
                encoder = int(cols[3], 10)
                pulses = int(cols[4], 10)
            except ValueError:
                continue
            rows.append(
                {
                    "axis": axis,
                    "seq": seq,
                    "time_ms": t_ms,
                    "encoder": encoder,
                    "pulses": pulses,
                }
            )
    return rows


def parse_microstep_from_name(path: Path) -> int:
    m = re.search(r"mstep(\d+)", path.stem, re.IGNORECASE)
    if not m:
        return DEFAULT_MICROSTEP
    try:
        return int(m.group(1))
    except ValueError:
        return DEFAULT_MICROSTEP


def compute_derived_series(
    rows: List[Dict[str, int]],
    stride: int = SAMPLE_STRIDE,
) -> Tuple[List[Dict[str, float]], List[float], List[float], List[float], List[float]]:
    """
    Constrói amostras derivadas usando uma janela deslizante de `stride` linhas válidas.

    Ignora a primeira linha (sem referência) e qualquer linha cujo time_ms não
    avance. Cada nova amostra considera o delta entre a linha atual e a linha
    mais antiga da janela, preservando a resolução temporal sem atrasar a
    detecção do degrau, ao mesmo tempo em que suaviza o ruído.
    """
    if stride < 1:
        stride = 1
    derived: List[Dict[str, float]] = []
    window: List[Dict[str, int]] = []
    last_valid: Optional[Dict[str, int]] = None
    for row in rows:
        if last_valid is None:
            last_valid = row
            window = [row]
            continue
        dt_ms = row["time_ms"] - last_valid["time_ms"]
        if dt_ms <= 0:
            # mantém o baseline anterior; apenas descarta esta linha
            continue
        last_valid = row
        window.append(row)
        if len(window) > stride:
            window.pop(0)
        if len(window) < stride:
            continue
        first = window[0]
        last = window[-1]
        span_ms = last["time_ms"] - first["time_ms"]
        if span_ms <= 0:
            continue
        dt = span_ms / 1000.0
        dpulses = last["pulses"] - first["pulses"]
        denc = last["encoder"] - first["encoder"]
        derived.append(
            {
                "axis": last["axis"],
                "id": last["seq"],
                "time_ms": last["time_ms"],
                "time_s": last["time_ms"] / 1000.0,
                "encoder": last["encoder"],
                "pulses": last["pulses"],
                "delta_t_s": dt,
                "vel_cmd_sps": dpulses / dt if dt != 0 else math.nan,
                "vel_enc_sps": denc / dt if dt != 0 else math.nan,
            }
        )
    times_s = [row["time_s"] for row in derived]
    delta_t_s = [row["delta_t_s"] for row in derived]
    vel_cmd = [row["vel_cmd_sps"] for row in derived]
    vel_enc = [row["vel_enc_sps"] for row in derived]
    return derived, times_s, delta_t_s, vel_cmd, vel_enc


def compute_raw_velocity_series(
    rows: List[Dict[str, int]],
) -> Tuple[List[float], List[float], List[float], List[float]]:
    times_s: List[float] = []
    vel_cmd: List[float] = []
    vel_enc: List[float] = []
    dt_values: List[float] = []
    prev = None
    for row in rows:
        if prev is None:
            prev = row
            continue
        dt_ms = row["time_ms"] - prev["time_ms"]
        if dt_ms <= 0:
            prev = row
            continue
        dt = dt_ms / 1000.0
        dpulses = row["pulses"] - prev["pulses"]
        denc = row["encoder"] - prev["encoder"]
        vel_cmd.append(dpulses / dt if dt != 0 else math.nan)
        vel_enc.append(denc / dt if dt != 0 else math.nan)
        times_s.append(row["time_ms"] / 1000.0)
        dt_values.append(dt)
        prev = row
    return times_s, vel_cmd, vel_enc, dt_values


def _is_valid(value: Optional[float]) -> bool:
    return value is not None and not math.isnan(value)


def steady_value(values: Sequence[Optional[float]], window_fraction: float) -> Optional[float]:
    if not values:
        return None
    start = len(values) - max(1, int(len(values) * window_fraction))
    window = [v for v in values[start:] if _is_valid(v)]
    if not window:
        window = [v for v in values if _is_valid(v)]
    if not window:
        return None
    return statistics.fmean(window)


def first_crossing(times: Sequence[float], values: Sequence[Optional[float]], threshold: float) -> Optional[float]:
    if threshold is None:
        return None
    for t, v in zip(times, values):
        if v is None or math.isnan(v):
            continue
        if v >= threshold:
            return t
    return None


def classify_step(times: Sequence[float], values: Sequence[Optional[float]], steady: Optional[float], t_start: Optional[float]) -> Tuple[str, Optional[float], Optional[float]]:
    if steady is None or t_start is None or steady <= 0:
        return ("unknown", None, None)
    target = 0.9 * steady
    t_90 = first_crossing(times, values, target)
    if t_90 is None:
        return ("unknown", None, None)
    rise = t_90 - t_start
    total = times[-1] - t_start
    if total <= 0:
        return ("unknown", rise, None)
    ratio = rise / total
    mode = "step-like" if rise <= 0.15 * total else "ramp-like"
    return (mode, rise, ratio)


def theoretical_gain(axis: int, microstep: int) -> Optional[float]:
    counts = ENC_COUNTS_PER_REV.get(axis)
    if counts is None or microstep <= 0:
        return None
    steps_per_rev = STEPS_PER_REV_BASE * microstep
    return counts / steps_per_rev


def sanitize_float(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    if math.isnan(value) or math.isinf(value):
        return None
    return value


def analyze_file(
    csv_path: Path,
    derived_dir: Optional[Path],
    steady_window: float,
    stride: int,
) -> Dict[str, object]:
    rows = load_rows(csv_path)
    if not rows:
        raise ValueError("no valid rows")
    axis = rows[0]["axis"]
    microstep = parse_microstep_from_name(csv_path)
    raw_times_s, raw_vel_cmd, raw_vel_enc, raw_dt_values = compute_raw_velocity_series(rows)
    if not raw_times_s:
        raise ValueError("no valid raw samples (Δt<=0 everywhere)")
    derived_rows, times_s, delta_t_s, vel_cmd, vel_enc = compute_derived_series(rows, stride=stride)
    if not derived_rows:
        raise ValueError("no derived samples after applying stride/window")
    duration_s = (rows[-1]["time_ms"] - rows[0]["time_ms"]) / 1000.0 if len(rows) > 1 else 0.0
    steady_cmd = sanitize_float(steady_value(vel_cmd, steady_window))
    steady_enc = sanitize_float(steady_value(vel_enc, steady_window))
    cmd_thresh = steady_cmd * 0.05 if steady_cmd and steady_cmd > 0 else None
    enc_thresh = steady_enc * 0.05 if steady_enc and steady_enc > 0 else None
    t_cmd_start = sanitize_float(first_crossing(raw_times_s, raw_vel_cmd, cmd_thresh) if cmd_thresh else None)
    t_enc_start = sanitize_float(first_crossing(raw_times_s, raw_vel_enc, enc_thresh) if enc_thresh else None)
    L_s = sanitize_float((t_enc_start - t_cmd_start) if (t_cmd_start is not None and t_enc_start is not None) else None)
    if L_s is not None and L_s < 0:
        L_s = 0.0
    target_63 = steady_enc * 0.632 if steady_enc else None
    t_63 = sanitize_float(first_crossing(raw_times_s, raw_vel_enc, target_63) if target_63 else None)
    tau_s = None
    if t_63 is not None and L_s is not None:
        tau_s = sanitize_float(t_63 - L_s)
        if tau_s is not None and tau_s < 0:
            tau_s = None
    min_dt_raw = min(raw_dt_values) if raw_dt_values else None
    resolution_note = None
    tau_resolution_note = None
    if (L_s is None or L_s <= 0) and min_dt_raw:
        L_s = min_dt_raw
        if t_63 is not None:
            tau_s = sanitize_float(t_63 - L_s)
            if tau_s is not None and tau_s < 0:
                tau_s = None
        resolution_note = f"L ajustado para resolução mínima (~{L_s:.6f} s)."
    if (tau_s is None or tau_s <= 0) and min_dt_raw:
        tau_s = min_dt_raw
        tau_resolution_note = f"τ ajustado para resolução mínima (~{tau_s:.6f} s)."
    K_gain = None
    if steady_cmd and steady_cmd > 0 and steady_enc is not None:
        K_gain = sanitize_float(steady_enc / steady_cmd)
    Kp = Kd = Ki = None
    if K_gain and L_s and tau_s and L_s > 0 and tau_s > 0:
        Kp = sanitize_float(1.2 * (tau_s / (K_gain * L_s)))
        Td = 0.5 * L_s
        Kd = sanitize_float(Kp * Td) if Kp is not None else None
        Ki = 0.0
    elif K_gain:
        Ki = 0.0
    dt_vals = [v for v in delta_t_s if _is_valid(v)]
    dt_stats = {}
    if dt_vals:
        dt_stats = {
            "min": min(dt_vals),
            "max": max(dt_vals),
            "mean": sum(dt_vals) / len(dt_vals),
            "median": statistics.median(dt_vals),
        }
    mode, rise_time, rise_ratio = classify_step(raw_times_s, raw_vel_cmd, steady_cmd, t_cmd_start)
    theo_gain = theoretical_gain(axis, microstep)
    viscous_pct = None
    if K_gain and theo_gain:
        viscous_pct = (1.0 - (K_gain / theo_gain)) * 100.0
    derived_path = None
    if derived_dir:
        derived_dir.mkdir(parents=True, exist_ok=True)
        derived_path = derived_dir / f"{csv_path.stem}_derived.csv"
        with derived_path.open("w", encoding="utf-8", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(DERIVED_HEADER)
            for row in derived_rows:
                writer.writerow(
                    [
                        row["axis"],
                        row["id"],
                        row["time_ms"],
                        f"{row['time_s']:.6f}",
                        row["encoder"],
                        row["pulses"],
                        "" if math.isnan(row["delta_t_s"]) else f"{row['delta_t_s']:.9f}",
                        "" if math.isnan(row["vel_cmd_sps"]) else f"{row['vel_cmd_sps']:.3f}",
                        "" if math.isnan(row["vel_enc_sps"]) else f"{row['vel_enc_sps']:.3f}",
                    ]
                )
    return {
        "path": csv_path,
        "axis": axis,
        "axis_name": AXIS_NAMES.get(axis, f"axis{axis}"),
        "microstep": microstep,
        "samples": len(derived_rows),
        "duration_s": duration_s,
        "steady_cmd": steady_cmd,
        "steady_enc": steady_enc,
        "K": K_gain,
        "L_s": L_s,
        "tau_s": tau_s,
        "t63_s": t_63,
        "t_cmd_start": t_cmd_start,
        "t_enc_start": t_enc_start,
        "cmd_rise_s": rise_time,
        "cmd_rise_ratio": rise_ratio,
        "step_shape": mode,
        "viscous_pct": viscous_pct,
        "theoretical_K": theo_gain,
        "Kp": Kp,
        "Ki": Ki,
        "Kd": Kd,
        "safe_Kp": (Kp * 0.5) if Kp else None,
        "derived_csv": derived_path,
        "dt_stats": dt_stats,
        "target63": target_63,
        "resolution_note": resolution_note,
        "tau_note": tau_resolution_note,
    }


def fmt_num(value: Optional[float], unit: str = "", precision: int = 4) -> str:
    if value is None:
        return "n/a"
    if abs(value) >= 1000:
        text = f"{value:,.2f}"
    else:
        text = f"{value:.{precision}f}"
    return f"{text}{unit}"


def render_summary(results: List[Dict[str, object]]) -> str:
    lines: List[str] = []
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    lines.append("SWV Step Response Analysis")
    lines.append(f"Generated: {now}")
    lines.append(
        "Reference: motion_service.c (STEPS_PER_REV_BASE=400, default microstep=256, encoder counts X/Z=40000, Y=2500)."
    )
    lines.append("")
    for res in results:
        lines.append(f"=== {res['path'].name} ===")
        lines.append(
            f"Axis {res['axis_name']} (id {res['axis']}), microstep {res['microstep']}, samples {res['samples']}, duration {fmt_num(res['duration_s'], ' s', 3)}."
        )
        dt_stats = res.get("dt_stats", {})
        if dt_stats:
            lines.append(
                "Δt stats: min "
                f"{fmt_num(dt_stats.get('min'), ' s', 6)}, median {fmt_num(dt_stats.get('median'), ' s', 6)}, "
                f"mean {fmt_num(dt_stats.get('mean'), ' s', 6)}, max {fmt_num(dt_stats.get('max'), ' s', 6)}."
            )
        lines.append(
            "Steady velocities: comando "
            f"{fmt_num(res['steady_cmd'], ' pulsos/s', 2)}, encoder {fmt_num(res['steady_enc'], ' pulsos/s', 2)}."
        )
        lines.append(
            "G_v(s) params: K="
            f"{fmt_num(res['K'])}, L={fmt_num(res['L_s'], ' s', 6)}, τ={fmt_num(res['tau_s'], ' s', 6)} (t63={fmt_num(res['t63_s'], ' s', 6)})."
        )
        lines.append(
            "PD gains (Z-N reaction curve): "
            f"Kp={fmt_num(res['Kp'])}, Ki={fmt_num(res['Ki'])}, Kd={fmt_num(res['Kd'])}. "
            f"Sugestão segura inicial Kp={fmt_num(res['safe_Kp'])} (50%)."
        )
        lines.append(
            f"Step detection: {res['step_shape']} (t90 rise {fmt_num(res['cmd_rise_s'], ' s', 6)}; "
            f"fraction {fmt_num(res['cmd_rise_ratio'], '', 3)})."
        )
        lines.append(
            "Viscoso/teórico: ganho medido vs teórico "
            f"{fmt_num(res['K'])} / {fmt_num(res['theoretical_K'])} "
            f"→ variação {fmt_num(res['viscous_pct'], ' %', 3)}."
        )
        if res.get("derived_csv"):
            rel_path = res["derived_csv"]
            lines.append(f"Derived CSV: {rel_path}")
        notes: List[str] = []
        if res["step_shape"] == "ramp-like":
            notes.append(
                "Comando parece rampa; execute novo teste em degrau para aplicar Z-N com maior confiança."
            )
        if res.get("resolution_note"):
            notes.append(res["resolution_note"])
        if res.get("tau_note"):
            notes.append(res["tau_note"])
        missing_params: List[str] = []
        if res["K"] is None:
            missing_params.append("K (ganho estático)")
        if res["L_s"] is None:
            missing_params.append("L (tempo morto)")
        if res["tau_s"] is None:
            missing_params.append("τ (constante de tempo)")
        if res["Kp"] is None:
            missing_params.append("Kp")
        if res["Kd"] is None:
            missing_params.append("Kd")
        if missing_params:
            detail = ", ".join(missing_params)
            notes.append(f"Dados insuficientes para extrair: {detail}.")
        if notes:
            lines.append("Observações:")
            for note in notes:
                lines.append(f"  - {note}")
        lines.append("")
    lines.append(
        "Use estes números como ponto de partida. Ajuste Kp gradualmente (50% → 100%) com o Kd sugerido e Ki=0, conforme recomendado."
    )
    return "\n".join(lines)


def main() -> int:
    args = parse_args()
    if args.stride <= 0:
        print("Stride deve ser >= 1.", file=sys.stderr)
        return 1
    files = discover_files(args.paths)
    if not files:
        print("No *_filtered.csv files found.", file=sys.stderr)
        return 1
    base = DEFAULT_INPUT_DIR
    if args.paths:
        first = Path(args.paths[0]).expanduser()
        base = first if first.is_dir() else first.parent
    summary_path = args.summary if args.summary else (base / "analysis_summary.txt")
    derived_dir = None
    if not args.no_derived:
        derived_dir = args.derived_dir if args.derived_dir else (base / "analysis_data")
    results: List[Dict[str, object]] = []
    for csv_path in files:
        try:
            res = analyze_file(csv_path, derived_dir, args.steady_window, args.stride)
        except Exception as exc:  # noqa: BLE001
            print(f"[ERROR] {csv_path}: {exc}", file=sys.stderr)
            continue
        results.append(res)
        if args.verbose:
            print(f"Analyzed {csv_path.name}")
    if not results:
        print("No analyses completed successfully.", file=sys.stderr)
        return 2
    summary_text = render_summary(results)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(summary_text, encoding="utf-8")
    print(f"Wrote summary to {summary_path} ({len(results)} dataset(s)).")
    if derived_dir:
        print(f"Derived CSVs stored under {derived_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
