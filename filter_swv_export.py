#!/usr/bin/env python3
"""
Normalize SWV export logs into clean CSV files.

The firmware can dump raw telemetry under CNC_Controller/SWV_export/.
Those dumps sometimes include spurious text, truncated lines or rows
with counters that move backwards.  This helper keeps only the safe
rows in the canonical format:

    axis,id,time_ms,encoder,pulses

Filtering rules (per user request):
  * Accept only rows that already look like number,number,number,number,number.
  * Require strictly increasing ids, while time and pulse counters must be
    non-decreasing (>=).
  * Convert encoder readings to their absolute value (always positive).
  * Enforce a single axis per file: once the first numeric row sets the axis,
    every subsequent row must keep the same value or it gets discarded.

By default the script scans the SWV_export directory next to this file,
but you can point it to any file or directory.
"""

from __future__ import annotations

import argparse
import sys
import re
from pathlib import Path
from typing import List, Sequence

NUMERIC_ROW = re.compile(r"^\s*(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)\s*$")
SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_INPUT_DIR = SCRIPT_DIR / "CNC_Controller" / "SWV_export"
SUPPORTED_EXTENSIONS = {".txt", ".csv", ".log"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate sanitized CSV files from SWV export dumps."
    )
    parser.add_argument(
        "inputs",
        metavar="PATH",
        nargs="*",
        help="Files or directories to process (defaults to SWV_export folder).",
    )
    parser.add_argument(
        "--outdir",
        "-o",
        type=Path,
        default=None,
        help="Optional directory for the sanitized CSV files (defaults to alongside each input).",
    )
    parser.add_argument(
        "--suffix",
        default="_filtered",
        help="Suffix appended to the stem when writing next to the input file (default: _filtered).",
    )
    parser.add_argument(
        "--encoding",
        default="utf-8",
        help="Text encoding used to read the SWV export files (default: utf-8).",
    )
    parser.add_argument(
        "--quiet",
        "-q",
        action="store_true",
        help="Suppress per-file summaries; still prints errors.",
    )
    return parser.parse_args()


def discover_sources(paths: Sequence[str]) -> List[Path]:
    candidates = list(paths) if paths else [str(DEFAULT_INPUT_DIR)]
    sources: List[Path] = []
    seen = set()
    for raw in candidates:
        p = Path(raw).expanduser()
        if not p.exists():
            print(f"[WARN] Path not found: {p}", file=sys.stderr)
            continue
        if p.is_dir():
            for child in sorted(p.iterdir()):
                if child.is_file() and child.suffix.lower() in SUPPORTED_EXTENSIONS:
                    if child not in seen:
                        sources.append(child)
                        seen.add(child)
        else:
            if p not in seen:
                sources.append(p)
                seen.add(p)
    return sources


def destination_for(src: Path, outdir: Path | None, suffix: str) -> Path:
    target_name = f"{src.stem}{suffix}.csv"
    if outdir:
        outdir.mkdir(parents=True, exist_ok=True)
        return outdir / target_name
    return src.with_name(target_name)


def sanitize_file(src: Path, dest: Path, encoding: str) -> tuple[int, int]:
    kept = 0
    skipped = 0
    last_id = last_time = last_pulses = None
    axis_ref = None
    with src.open("r", encoding=encoding, errors="ignore") as infile, dest.open(
        "w", encoding="utf-8"
    ) as outfile:
        for line_no, raw in enumerate(infile, 1):
            match = NUMERIC_ROW.match(raw.strip())
            if not match:
                skipped += 1
                continue
            axis, seq, t_ms, encoder, pulses = (int(group, 10) for group in match.groups())
            if axis_ref is None:
                axis_ref = axis
            elif axis != axis_ref:
                skipped += 1
                continue
            if last_id is not None:
                if seq <= last_id or t_ms < last_time or pulses < last_pulses:
                    skipped += 1
                    continue
            last_id, last_time, last_pulses = seq, t_ms, pulses
            outfile.write(f"{axis},{seq},{t_ms},{abs(encoder)},{pulses}\n")
            kept += 1
    if kept == 0:
        try:
            dest.unlink()
        except FileNotFoundError:
            pass
    return kept, skipped


def main() -> int:
    args = parse_args()
    sources = discover_sources(args.inputs)
    if not sources:
        print("No input files found to process.", file=sys.stderr)
        return 1
    results = []
    for src in sources:
        dest = destination_for(src, args.outdir, args.suffix)
        try:
            kept, skipped = sanitize_file(src, dest, args.encoding)
        except Exception as exc:  # noqa: BLE001
            print(f"[ERROR] {src}: {exc}", file=sys.stderr)
            continue
        results.append((src, dest, kept, skipped))
        if not args.quiet:
            outcome = "EMPTY" if kept == 0 else "OK"
            print(f"{outcome}: {src} -> {dest} | kept={kept} skipped={skipped}")
    if not results:
        return 2
    kept_total = sum(record[2] for record in results)
    skipped_total = sum(record[3] for record in results)
    print(f"Done. Files processed: {len(results)} | kept={kept_total} skipped={skipped_total}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
