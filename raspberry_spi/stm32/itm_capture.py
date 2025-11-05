"""SWV/ITM capture helpers backed by pyOCD.

This module provides a small command line tool that opens a pyOCD session, enables SWO/ITM
streaming, and stores the decoded stimulus packets in a CSV file. Optionally it can also persist
the raw SWO byte stream alongside the decoded log.

Example:
    python -m raspberry_spi.stm32.itm_capture \\
        --system-clock 168000000 \\
        --swo-clock 2000000 \\
        --output itm.csv

On exit (Ctrl+C) the script stops SWO reception and closes the pyOCD session gracefully.
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path
from typing import Iterable, Optional, Sequence

from pyocd.core import exceptions as pyocd_exceptions
from pyocd.core.helpers import ConnectHelper
from pyocd.coresight.itm import ITM
from pyocd.coresight.tpiu import TPIU
from pyocd.probe.debug_probe import DebugProbe
from pyocd.trace.events import TraceEvent, TraceITMEvent, TraceOverflow
from pyocd.trace.sink import TraceEventSink
from pyocd.trace.swo import SWOParser


class CsvItmSink(TraceEventSink):
    """Writes decoded ITM packets to a CSV file."""

    def __init__(self, writer: csv.writer, port_filter: Optional[Sequence[int]]) -> None:
        self._writer = writer
        self._port_filter = set(port_filter) if port_filter else None
        self._writer.writerow(("timestamp", "port", "width_bytes", "data_hex", "data_dec"))

    def _should_log(self, port: int) -> bool:
        return not self._port_filter or port in self._port_filter

    def receive(self, event: TraceEvent) -> None:  # type: ignore[override]
        if isinstance(event, TraceITMEvent):
            if not self._should_log(event.port):
                return

            width = event.width
            data = event.data
            self._writer.writerow(
                (
                    event.timestamp,
                    event.port,
                    width,
                    f"0x{data:0{width * 2}X}",
                    data,
                )
            )
        elif isinstance(event, TraceOverflow):
            # Overflow packets do not carry payload, but they are critical to spot dropped bytes.
            self._writer.writerow((event.timestamp, "overflow", "", "", ""))


def _configure_itm_and_tpiu(target, system_clock: int, swo_clock: int, trace_ports: Iterable[int]) -> None:
    """Initialises trace blocks so the probe can receive SWO data."""

    itm = target.get_first_child_of_type(ITM)
    if itm is None:
        raise RuntimeError("Target does not expose an ITM component. Cannot configure trace.")

    tpiu = target.get_first_child_of_type(TPIU)
    if tpiu is None:
        raise RuntimeError("Target does not expose a TPIU component. Cannot configure trace.")

    # Enable the selected stimulus ports so all requested traffic is forwarded to the host.
    port_mask = 0
    for port in trace_ports:
        if 0 <= port <= 31:
            port_mask |= 1 << port

    target.trace_start()
    itm.init()
    # Enable ITM with the selected stimulus port mask. Default to all ports if mask is zero to
    # preserve backwards compatibility with firmware that may rely on other ports.
    itm.enable(enabled_ports=port_mask or 0xFFFFFFFF)
    tpiu.init()

    if not tpiu.set_swo_clock(swo_clock, system_clock):
        raise RuntimeError(
            f"Failed to set SWO clock divider (requested {swo_clock} Hz from {system_clock} Hz system clock)."
        )


def _stop_trace(target) -> None:
    itm = target.get_first_child_of_type(ITM)
    if itm is not None:
        itm.disable()
    target.trace_stop()


def capture_itm_stream(
    *,
    output_file: Path,
    raw_file: Optional[Path],
    system_clock: int,
    swo_clock: int,
    core_index: int,
    duration: Optional[float],
    poll_interval: float,
    probe_id: Optional[str],
    target_override: Optional[str],
    swd_frequency: Optional[int],
    ports: Optional[Sequence[int]],
    skip_trace_setup: bool,
) -> None:
    """Main capture loop."""

    session = ConnectHelper.session_with_chosen_probe(
        unique_id=probe_id,
        auto_open=True,
        options={
            k: v
            for k, v in {
                "target_override": target_override,
                "frequency": swd_frequency,
                "halt_on_connect": False,
                "auto_unlock": True,
                "serve_local_only": True,
            }.items()
            if v is not None
        },
    )

    if session is None:
        raise RuntimeError("Nenhum probe compatível encontrado. Conecte o dispositivo e tente novamente.")

    ports_to_enable = ports if ports else range(32)

    with session:
        assert session.probe
        probe: DebugProbe = session.probe

        target = session.target
        if target is None:
            raise RuntimeError("Sessão pyOCD sem alvo detectado.")

        if not skip_trace_setup:
            if system_clock <= 0:
                raise ValueError("--system-clock deve ser informado quando a configuração automática está habilitada.")
            _configure_itm_and_tpiu(target, system_clock, swo_clock, ports_to_enable)

        cores = target.cores
        if core_index < 0 or core_index >= len(cores):
            raise ValueError(f"Número de core inválido: {core_index}. O alvo possui {len(cores)} núcleo(s).")

        parser = SWOParser(cores[core_index])
        with output_file.open("w", newline="") as log_handle:
            csv_writer = csv.writer(log_handle)
            sink = CsvItmSink(csv_writer, ports)
            parser.connect(sink)

            raw_handle = raw_file.open("wb") if raw_file else None

            # Guarantee SWO is restarted under our control.
            try:
                probe.swo_stop()
            except pyocd_exceptions.Error:
                pass

            probe.swo_start(swo_clock)

            try:
                deadline = None if duration is None else (time.monotonic() + duration)
                while True:
                    chunk = probe.swo_read()
                    if chunk:
                        if raw_handle:
                            raw_handle.write(chunk)
                            raw_handle.flush()

                        parser.parse(chunk)
                        log_handle.flush()
                    else:
                        time.sleep(poll_interval)

                    if deadline is not None and time.monotonic() >= deadline:
                        break
            except KeyboardInterrupt:
                # Graceful exit on Ctrl+C.
                pass
            finally:
                if raw_handle:
                    raw_handle.close()
                probe.swo_stop()
                if not skip_trace_setup:
                    _stop_trace(target)


def _parse_args(argv: Optional[Sequence[str]]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Captura pacotes ITM (SWV) usando pyOCD e armazena em CSV.",
    )
    parser.add_argument("--probe-id", help="Número de série do probe (opcional).")
    parser.add_argument("--target", dest="target_override", help="Target override para o pyOCD.")
    parser.add_argument("--swd-frequency", type=int, help="Frequência SWD em Hz (ex.: 8000000).")
    parser.add_argument(
        "--system-clock",
        type=int,
        default=0,
        help="Frequência do clock do sistema em Hz. Obrigatório salvo se usar --skip-trace-setup.",
    )
    parser.add_argument("--swo-clock", type=int, default=2000000, help="Frequência do traço SWO em Hz.")
    parser.add_argument("--core", type=int, default=0, help="Índice do core a monitorar (default: 0).")
    parser.add_argument("--duration", type=float, help="Captura apenas por N segundos (opcional).")
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=0.002,
        help="Intervalo mínimo entre leituras (segundos).",
    )
    parser.add_argument(
        "--ports",
        type=lambda value: [int(p) for p in value.split(",") if p.strip()],
        help="Lista de portas ITM separadas por vírgula a serem armazenadas (default: todas).",
    )
    parser.add_argument("--output", type=Path, default=Path("itm_log.csv"), help="Arquivo de saída (CSV).")
    parser.add_argument(
        "--raw-output",
        type=Path,
        help="Arquivo para salvar a captura bruta do SWO (binário).",
    )
    parser.add_argument(
        "--skip-trace-setup",
        action="store_true",
        help="Não tenta configurar ITM/TPIU; assume que o firmware já configurou o SWO.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parse_args(argv)
    try:
        capture_itm_stream(
            output_file=args.output,
            raw_file=args.raw_output,
            system_clock=args.system_clock,
            swo_clock=args.swo_clock,
            core_index=args.core,
            duration=args.duration,
            poll_interval=args.poll_interval,
            probe_id=args.probe_id,
            target_override=args.target_override,
            swd_frequency=args.swd_frequency,
            ports=args.ports,
            skip_trace_setup=args.skip_trace_setup,
        )
    except (RuntimeError, ValueError, pyocd_exceptions.Error) as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
