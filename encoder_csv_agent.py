#!/usr/bin/env python3
"""
Encoder CSV receiver

Reads CSV lines "t_ms,enc_pos,step_count" from a serial port (UART) or stdin
and optionally writes them to a file for later analysis.

Usage examples:
  - Read from UART1 (115200) and print to screen:
      python encoder_csv_agent.py --port COM3 --baud 115200

  - Save to file:
      python encoder_csv_agent.py -p COM3 -b 115200 -o run.csv

  - Read from stdin (e.g., if you export SWV console to a file and pipe it):
      type swv_output.txt | python encoder_csv_agent.py --port stdin -o run.csv

Notes:
  - The firmware prints CSV only when the encoder changes, with columns:
        time_ms_since_start, encoder_position_relative, step_pulses_since_start
  - To receive via Python directly, either:
      a) Disable SWO in your debug session so firmware falls back to UART, or
      b) Build with -DLOG_FORCE_UART=1 to force UART output even with SWO enabled.
"""

import argparse
import sys
from typing import Optional, TextIO


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Receive encoder CSV over serial or stdin")
    p.add_argument("--port", "-p", default="COM3",
                   help="Serial port (e.g., COM3 or /dev/ttyUSB0). Use 'stdin' to read from stdin.")
    p.add_argument("--baud", "-b", type=int, default=115200, help="Baudrate (UART)")
    p.add_argument("--outfile", "-o", default=None, help="Optional output CSV file path")
    p.add_argument("--quiet", "-q", action="store_true", help="Do not echo parsed rows to stdout")
    return p.parse_args()


def open_serial(port: str, baud: int):
    try:
        import serial  # pyserial
        ser = serial.Serial(port=port, baudrate=baud, timeout=1)
        return ser
    except ImportError:
        print("pyserial is required: pip install pyserial", file=sys.stderr)
        sys.exit(2)


def iter_lines_from_stream(stream: TextIO):
    buf = ""
    while True:
        chunk = stream.read(1)
        if not chunk:
            if buf:
                yield buf
            break
        if chunk in ("\r", "\n"):
            if buf:
                yield buf
                buf = ""
        else:
            buf += chunk


def run():
    args = parse_args()
    outfh: Optional[TextIO] = None
    if args.outfile:
        outfh = open(args.outfile, "w", encoding="utf-8")
        outfh.write("axis,time_ms,enc_pos,step_count\n")
        outfh.flush()

    if args.port.lower() == "stdin":
        source = iter_lines_from_stream(sys.stdin)
        close_serial = None
    else:
        ser = open_serial(args.port, args.baud)
        close_serial = ser.close

        def serial_lines():
            buf = bytearray()
            while True:
                b = ser.read(1)
                if not b:
                    continue
                if b in (b"\r", b"\n"):
                    if buf:
                        try:
                            yield buf.decode("utf-8", errors="ignore")
                        finally:
                            buf.clear()
                else:
                    buf += b

        source = serial_lines()

    try:
        for line in source:
            s = line.strip()
            if not s:
                continue
            # Optional: filter out any non-CSV console noise
            if s.startswith("["):
                # Skip bracketed debug lines
                continue
            parts = s.split(',')
            if len(parts) == 4:
                try:
                    axis = int(parts[0], 10)
                    t_ms = int(parts[1], 10)
                    pos = int(parts[2], 10)
                    steps = int(parts[3], 10)
                except ValueError:
                    continue
            elif len(parts) == 3:
                # Backward compatibility (no axis)
                try:
                    axis = -1
                    t_ms = int(parts[0], 10)
                    pos = int(parts[1], 10)
                    steps = int(parts[2], 10)
                except ValueError:
                    continue
            else:
                continue
            if not args.quiet:
                print(f"axis={axis}, t={t_ms} ms, pos={pos}, steps={steps}")
            if outfh:
                outfh.write(f"{axis},{t_ms},{pos},{steps}\n")
                outfh.flush()
    except KeyboardInterrupt:
        pass
    finally:
        if outfh:
            outfh.close()
        if 'close_serial' in locals() and close_serial:
            try:
                close_serial()
            except Exception:
                pass


if __name__ == "__main__":
    run()
