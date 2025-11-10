#!/usr/bin/env python3
"""
Generate plots for every sanitized SWV CSV (files ending with *_filtered.csv).

Each filtered file is expected to contain lines in the format:
    axis,id,time_ms,encoder,pulses

The script loads all matching CSVs under CNC_Controller/SWV_export/
(or paths provided via CLI), plots encoder and/or pulse counts versus time,
and saves PNGs inside a subdirectory that lives under SWV_export.
Use --select PATTERN (e.g., --select "motor2*") to restrict which files
and --series to choose which data series get plotted.
"""

from __future__ import annotations

import argparse
import csv
import fnmatch
import sys
from pathlib import Path
from typing import List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT_DIR = SCRIPT_DIR / "CNC_Controller" / "SWV_export"
DEFAULT_OUTPUT_SUBDIR = "plots_filtered"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot all *_filtered.csv files generated from SWV exports."
    )
    parser.add_argument(
        "inputs",
        metavar="PATH",
        nargs="*",
        help="Files or directories to scan. Defaults to the SWV_export folder.",
    )
    parser.add_argument(
        "--outdir-name",
        default=DEFAULT_OUTPUT_SUBDIR,
        help="Subdirectory name (under SWV_export) where PNGs will be stored.",
    )
    parser.add_argument(
        "--dpi", type=int, default=110, help="DPI for the generated PNGs (default: 110)."
    )
    parser.add_argument(
        "--quiet",
        "-q",
        action="store_true",
        help="Suppress per-file summaries (errors still show up).",
    )
    parser.add_argument(
        "--select",
        "-s",
        action="append",
        metavar="PATTERN",
        help="Only plot files whose name matches the given glob pattern (repeatable).",
    )
    parser.add_argument(
        "--series",
        "-S",
        choices=("encoder", "pulses"),
        nargs="+",
        default=("encoder", "pulses"),
        help="Select which series to plot against time (default: encoder pulses).",
    )
    return parser.parse_args()


def discover_filtered_csvs(paths: Sequence[str]) -> List[Path]:
    candidates = list(paths) if paths else [str(DEFAULT_INPUT_DIR)]
    results: List[Path] = []
    seen = set()
    for raw in candidates:
        p = Path(raw).expanduser()
        if not p.exists():
            print(f"[WARN] Path not found: {p}", file=sys.stderr)
            continue
        if p.is_dir():
            for child in sorted(p.iterdir()):
                if (
                    child.is_file()
                    and child.suffix.lower() == ".csv"
                    and child.stem.endswith("_filtered")
                ):
                    if child not in seen:
                        results.append(child)
                        seen.add(child)
        else:
            if (
                p.is_file()
                and p.suffix.lower() == ".csv"
                and p.stem.endswith("_filtered")
                and p not in seen
            ):
                results.append(p)
                seen.add(p)
    return results


def apply_selection(files: List[Path], patterns: Sequence[str] | None) -> List[Path]:
    if not patterns:
        return files
    filtered: List[Path] = []
    for path in files:
        name = path.name
        stem = path.stem
        if any(fnmatch.fnmatch(name, pat) or fnmatch.fnmatch(stem, pat) for pat in patterns):
            filtered.append(path)
    return filtered


def ensure_outdir(csv_path: Path, subdir_name: str) -> Path:
    base = csv_path.parent
    outdir = base / subdir_name
    outdir.mkdir(parents=True, exist_ok=True)
    return outdir


def load_csv_rows(csv_path: Path) -> Tuple[int, List[int], List[int], List[int]]:
    times: List[int] = []
    encoder: List[int] = []
    pulses: List[int] = []
    axis_id = None
    with csv_path.open("r", encoding="utf-8", newline="") as fh:
        reader = csv.reader(fh)
        for fields in reader:
            if len(fields) < 5:
                continue
            try:
                axis = int(fields[0].strip(), 10)
                _seq = int(fields[1].strip(), 10)
                t_ms = int(fields[2].strip(), 10)
                enc = int(fields[3].strip(), 10)
                pulses_val = int(fields[4].strip(), 10)
            except ValueError:
                continue
            if axis_id is None:
                axis_id = axis
            times.append(t_ms)
            encoder.append(enc)
            pulses.append(pulses_val)
    if axis_id is None:
        raise ValueError("no valid rows found")
    return axis_id, times, encoder, pulses


def plot_csv(csv_path: Path, outdir: Path, series: Sequence[str], dpi: int) -> Path:
    unique_series: List[str] = []
    for item in series:
        if item not in unique_series:
            unique_series.append(item)
    if not unique_series:
        raise ValueError("no series selected to plot")
    axis_id, times, encoder, pulses = load_csv_rows(csv_path)
    if not times:
        raise ValueError("no samples to plot")
    fig, ax1 = plt.subplots(figsize=(10, 4))
    ax1.set_xlabel("tempo (ms)")
    ax1.grid(True, alpha=0.25)

    def plot_series(ax, kind: str) -> None:
        if kind == "encoder":
            ax.plot(times, encoder, color="tab:blue", label="Encoder")
            ax.set_ylabel("encoder (abs)", color="tab:blue")
            ax.tick_params(axis="y", labelcolor="tab:blue")
        elif kind == "pulses":
            ax.plot(times, pulses, color="tab:orange", label="Pulsos")
            ax.set_ylabel("pulsos acumulados", color="tab:orange")
            ax.tick_params(axis="y", labelcolor="tab:orange")

    plot_series(ax1, unique_series[0])
    if len(unique_series) > 1:
        ax_secondary = ax1.twinx()
        plot_series(ax_secondary, unique_series[1])

    title = f"{csv_path.stem} (axis {axis_id})"
    ax1.set_title(title)

    fig.tight_layout()
    outpng = outdir / f"{csv_path.stem}.png"
    fig.savefig(outpng, dpi=dpi)
    plt.close(fig)
    return outpng


def main() -> int:
    args = parse_args()
    csv_files = discover_filtered_csvs(args.inputs)
    csv_files = apply_selection(csv_files, args.select)
    if not csv_files:
        print("No *_filtered.csv files found.", file=sys.stderr)
        return 1
    generated = []
    for csv_path in csv_files:
        outdir = ensure_outdir(csv_path, args.outdir_name)
        try:
            png_path = plot_csv(csv_path, outdir, args.series, args.dpi)
        except Exception as exc:  # noqa: BLE001
            print(f"[ERROR] {csv_path}: {exc}", file=sys.stderr)
            continue
        generated.append(png_path)
        if not args.quiet:
            print(f"{csv_path.name} -> {png_path}")
    if not generated:
        print("No plots generated (all inputs failed).", file=sys.stderr)
        return 2
    print(f"Done. Generated {len(generated)} plot(s).")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
