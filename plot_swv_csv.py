#!/usr/bin/env python3
"""
Plot encoder telemetry from SWV/ITM console dump.

Expects CSV lines strictly as:
    axis,time_ms,enc_pos,step_count

Adjustments applied per request:
  - Make encoder positions positive: use abs(enc_pos)
  - Subtract per-axis offsets from these absolute values
    (defaults: X=40017, Y=5005, Z=40039)

Outputs one PNG per axis under ./plots/

Usage:
  python plot_swv_csv.py --file SWV_ITM_Data_Console_1.txt \
      --offsets 40017 5005 40039

Dependencies: matplotlib
  pip install matplotlib
"""

from __future__ import annotations

import argparse
import os
import sys
from typing import Dict, List, Tuple


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Plot encoder telemetry from SWV/ITM dump")
    p.add_argument("--file", "-f", default="SWV_ITM_Data_Console_1.txt",
                   help="Input text file with CSV lines (default: SWV_ITM_Data_Console_1.txt)")
    p.add_argument("--outdir", "-o", default="plots",
                   help="Directory to save plots (default: plots)")
    p.add_argument("--offsets", nargs=3, type=int, default=[40017, 5005, 40039],
                   metavar=("X","Y","Z"),
                   help="Offsets to subtract from |enc_pos| for axes X,Y,Z")
    p.add_argument("--title", default=None,
                   help="Optional title prefix for figures")
    p.add_argument("--dpi", type=int, default=110, help="PNG DPI (default: 110)")
    return p.parse_args()


def is_debug_line(s: str) -> bool:
    s = s.lstrip()
    if not s:
        return True
    if s.startswith("[") or s.startswith("LOG:") or s.startswith("L:svc="):
        return True
    return False


def load_rows(path: str) -> List[Tuple[int,int,int,int]]:
    rows: List[Tuple[int,int,int,int]] = []
    with open(path, "r", encoding="utf-8", errors="ignore") as fh:
        for line in fh:
            s = line.strip()
            if not s or is_debug_line(s):
                continue
            parts = [p.strip() for p in s.split(',')]
            # Strict: only accept 4-number rows
            if len(parts) != 4:
                continue
            try:
                axis = int(parts[0], 10)
                t_ms = int(parts[1], 10)
                pos = int(parts[2], 10)
                steps = int(parts[3], 10)
            except ValueError:
                continue
            rows.append((axis, t_ms, pos, steps))
    return rows


def group_by_axis(rows: List[Tuple[int,int,int,int]]):
    axes: Dict[int, List[Tuple[int,int,int]]] = {}
    for axis, t_ms, pos, steps in rows:
        axes.setdefault(axis, []).append((t_ms, pos, steps))
    # sort by time
    for a in axes:
        axes[a].sort(key=lambda x: x[0])
    return axes


def ensure_outdir(path: str) -> None:
    if not os.path.isdir(path):
        os.makedirs(path, exist_ok=True)


def summarize_axis(ax_id: int, series: List[Tuple[int,int,int]], offset: int) -> str:
    n = len(series)
    if n == 0:
        return f"axis {ax_id}: no data"
    t = [t for (t,_,__) in series]
    enc = [p for (_,p,__) in series]
    steps = [s for (_,__,s) in series]
    enc_adj = [max(abs(p) - offset, 0) for p in enc]
    t_min, t_max = min(t), max(t)
    s_min, s_max = min(steps), max(steps)
    er_min, er_max = min(enc), max(enc)
    ea_min, ea_max = min(enc_adj), max(enc_adj)
    # Detect duplicates/out-of-order times
    nondec = all(t[i] >= t[i-1] for i in range(1, n))
    dup_ts = sum(1 for i in range(1, n) if t[i] == t[i-1])
    return (f"axis {ax_id}: N={n} t=[{t_min},{t_max}] nondec={nondec} dups={dup_ts} | "
            f"steps=[{s_min},{s_max}] enc_raw=[{er_min},{er_max}] enc_adj(|enc|-{offset})=[{ea_min},{ea_max}]")


def plot_axis(ax_id: int, series: List[Tuple[int,int,int]], offset: int, outdir: str, title_prefix: str|None, dpi: int) -> str:
    import matplotlib.pyplot as plt

    if not series:
        raise ValueError(f"Axis {ax_id}: no data")
    t = [t for (t,_,__) in series]
    enc_raw = [p for (_,p,__) in series]
    # Apply requested adjustments
    enc_adj = [max(abs(p) - offset, 0) for p in enc_raw]

    fig, ax = plt.subplots(figsize=(10, 4))
    label = f"Axis {ax_id} |enc|-{offset}"
    ax.plot(t, enc_adj, lw=1.0, label=label)
    ax.set_xlabel("tempo (ms)")
    ax.set_ylabel("encoder (ajustado)")
    ttl = f"Encoder eixo {ax_id}"
    if title_prefix:
        ttl = f"{title_prefix} - {ttl}"
    ax.set_title(ttl)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")

    outpath = os.path.join(outdir, f"encoder_axis_{ax_id}.png")
    fig.tight_layout()
    fig.savefig(outpath, dpi=dpi)
    plt.close(fig)
    return outpath


def main() -> int:
    args = parse_args()
    rows = load_rows(args.file)
    if not rows:
        print(f"No telemetry rows found in {args.file}", file=sys.stderr)
        return 2
    axes = group_by_axis(rows)
    ensure_outdir(args.outdir)
    # Map axis index to provided offsets (X=0,Y=1,Z=2). Unknown axis (-1) uses X offset.
    off_map = {0: args.offsets[0], 1: args.offsets[1], 2: args.offsets[2], -1: args.offsets[0]}
    # Print validation summary before plotting
    print("Validation summary (strict 4-column rows only):")
    for ax_id, series in sorted(axes.items()):
        print("  " + summarize_axis(ax_id, series, off_map.get(ax_id, args.offsets[0])))
    outputs: List[str] = []
    for ax_id, series in sorted(axes.items()):
        try:
            out = plot_axis(ax_id, series, off_map.get(ax_id, args.offsets[0]), args.outdir, args.title, args.dpi)
            outputs.append(out)
        except Exception as e:
            print(f"Axis {ax_id}: {e}", file=sys.stderr)
    if outputs:
        print("Saved plots:\n  " + "\n  ".join(outputs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
