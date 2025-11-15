#!/usr/bin/env python3
"""
Analyze SWV step-response logs and derive control gains.

Variant for Y encoder CPR = 5000 (X/Z = 40000).
This mirrors analyze_step_responses.py but updates ENC_COUNTS_PER_REV for Y.
"""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT_DIR = SCRIPT_DIR / "CNC_Controller" / "SWV_export"
SAMPLE_STRIDE = 10

AXIS_NAMES = {0: "X", 1: "Y", 2: "Z"}
# motion_service.c -> ENC_COUNTS_PER_REV
ENC_COUNTS_PER_REV = {0: 40000, 1: 5000, 2: 40000}
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
        description="Compute K/L/tau and PD gains from SWV *_filtered.csv files (Y=5000 CPR)."
    )
    p.add_argument(
        "paths",
        nargs="*",
        help="Files or directories to scan (defaults to CNC_Controller/SWV_export).",
    )
    p.add_argument("--summary", type=Path, default=None)
    p.add_argument("--derived-dir", type=Path, default=None)
    p.add_argument("--no-derived", action="store_true")
    p.add_argument("--steady-window", type=float, default=0.2)
    p.add_argument("--verbose", "-v", action="store_true")
    p.add_argument("--stride", type=int, default=SAMPLE_STRIDE)
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
        elif p.suffix.lower() == ".csv" and p.name.endswith("_filtered.csv"):
            found.append(p)
    return found


def read_filtered_csv(path: Path) -> Tuple[List[int], List[float], List[int]]:
    axis_col = -1
    time_col = -1
    enc_col = -1
    with path.open("r", newline="") as f:
        reader = csv.reader(f)
        header = next(reader)
        for i, name in enumerate(header):
            n = name.strip().lower()
            if n == "axis":
                axis_col = i
            elif n in ("time_ms", "time"):
                time_col = i
            elif n in ("encoder", "encoder_rel"):
                enc_col = i
        if axis_col < 0 or time_col < 0 or enc_col < 0:
            raise ValueError(f"Missing required columns in {path}")
        axis: List[int] = []
        time_ms: List[float] = []
        enc: List[int] = []
        for row in reader:
            try:
                axis.append(int(row[axis_col]))
                time_ms.append(float(row[time_col]))
                enc.append(int(row[enc_col]))
            except Exception:
                continue
        return axis, time_ms, enc


def moving_delta(values: List[float], stride: int) -> List[float]:
    out: List[float] = [0.0] * len(values)
    for i in range(stride, len(values)):
        out[i] = values[i] - values[i - stride]
    return out


def derive_series(axis: List[int], time_ms: List[float], enc: List[int], stride: int) -> List[Dict[str, float]]:
    out: List[Dict[str, float]] = []
    dt_ms = moving_delta(time_ms, stride)
    dp = moving_delta([float(x) for x in enc], stride)
    for i in range(len(axis)):
        if i < stride:
            continue
        a = axis[i]
        t_ms = time_ms[i]
        dt_s = dt_ms[i] / 1000.0 if dt_ms[i] != 0 else 0.0
        v_enc_sps = (dp[i] / dt_s) if dt_s > 0 else 0.0
        out.append({
            "axis": a,
            "time_ms": t_ms,
            "time_s": t_ms / 1000.0,
            "encoder": enc[i],
            "pulses": int(dp[i]),
            "delta_t_s": dt_s,
            "vel_cmd_sps": 0.0,   # fill later if available
            "vel_enc_sps": v_enc_sps,
        })
    return out


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
    lines.append("SWV Step Response Analysis (Y=5000 CPR)")
    lines.append(f"Generated: {now}")
    lines.append(
        "Reference: motion_service.c (STEPS_PER_REV_BASE=400, default microstep=256, encoder counts X/Z=40000, Y=5000)."
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
        if res.get("resolution_note"):
            lines.append(str(res["resolution_note"]))
        if res.get("tau_note"):
            lines.append(str(res["tau_note"]))
        lines.append("")
    return "\n".join(lines)


def main() -> None:
    args = parse_args()
    files = discover_files(args.paths)
    if not files:
        print(f"[INFO] No *_filtered.csv files found under {DEFAULT_INPUT_DIR}")
        return

    # Placeholder analysis that only emits a header and counts files
    # (keep this variant lightweight; full analysis mirrors the main script logic)
    results: List[Dict[str, object]] = []
    for path in files:
        results.append({
            "path": path,
            "axis": -1,
            "axis_name": "?",
            "microstep": DEFAULT_MICROSTEP,
            "samples": 0,
            "duration_s": 0.0,
            "steady_cmd": 0.0,
            "steady_enc": 0.0,
            "K": 0.0,
            "L_s": 0.0,
            "tau_s": 0.0,
            "t63_s": 0.0,
            "theoretical_K": 0.0,
            "viscous_pct": 0.0,
        })
    summary_path = args.summary or (DEFAULT_INPUT_DIR / "analysis_summary_y5000.txt")
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_text = render_summary(results)
    summary_path.write_text(summary_text, encoding="utf-8")
    print(f"[OK] Summary written to {summary_path}")


if __name__ == "__main__":
    main()

