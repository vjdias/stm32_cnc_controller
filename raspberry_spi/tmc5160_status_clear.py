"""Helper subcommand for `status --clear-gstat` flow.

Sequence: dummy read -> read GSTAT -> write GSTAT=0x07 -> read GSTAT -> read DRV_STATUS.
Then optionally read any requested registers.
"""
from __future__ import annotations

import argparse
from typing import List, Sequence

try:
    from .tmc5160 import (
        REG_GCONF,
        REG_GSTAT,
        TMC5160Configurator,
        TMC5160ReadResult,
        TMC5160RegisterPreset,
        TMC5160TransferResult,
    )
except Exception:  # pragma: no cover
    from tmc5160 import (  # type: ignore
        REG_GCONF,
        REG_GSTAT,
        TMC5160Configurator,
        TMC5160ReadResult,
        TMC5160RegisterPreset,
        TMC5160TransferResult,
    )


def _add_common_spi_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--bus", type=int, default=0, help="SPI bus (default: 0)")
    parser.add_argument("--dev", type=int, default=1, help="SPI device (default: 1)")
    parser.add_argument(
        "--speed",
        type=int,
        default=4_000_000,
        help="SPI speed in Hz (default: 4_000_000)",
    )


def _build_status_clear_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Status with pre-sequence to clear GSTAT and read DRV_STATUS."
        ),
    )
    _add_common_spi_arguments(parser)
    parser.add_argument(
        "--register",
        action="append",
        default=[],
        metavar="REG",
        help="Registers to read after the sequence (aliases or numeric)",
    )
    return parser


def run_status_clear(
    argv: Sequence[str],
    *,
    configurator_factory=TMC5160Configurator,
    device_finder=lambda: [],
):
    parser = _build_status_clear_parser()
    args = parser.parse_args(list(argv))

    # Avoid circular import by defining small formatters here
    def _fmt_resp(result: TMC5160TransferResult) -> str:
        return (
            f"- 0x{result.address:02X} <= 0x{result.value:08X}\n"
            f"  Raw: {result.raw_hex}\n"
            f"  Status 0x{result.status.raw:02X}: {result.status.summary()}\n"
            f"  Prev: 0x{result.previous_data:08X}"
        )

    def _fmt_read(result: TMC5160ReadResult) -> str:
        return (
            f"- 0x{result.address:02X} => 0x{result.value:08X}\n"
            f"  Req : status=0x{result.request.status.raw:02X}, resp={result.request.raw_hex}\n"
            f"  Reply: status=0x{result.reply.status.raw:02X}, resp={result.reply.raw_hex}"
        )

    configurator = configurator_factory(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset.default(),
    )

    with configurator as driver:  # type: ignore
        print(
            f"Open SPI bus={args.bus} dev={args.dev} at {args.speed} Hz to read status"
        )
        print("Pre-sequence (clear GSTAT):")
        dummy = driver.read_register(REG_GCONF)
        print(_fmt_read(dummy))
        gstat_before = driver.read_register(REG_GSTAT)
        print(_fmt_read(gstat_before))
        cleared = driver.write_register(REG_GSTAT, 0x00000007)
        print(_fmt_resp(cleared))
        gstat_after = driver.read_register(REG_GSTAT)
        print(_fmt_read(gstat_after))
        drv = driver.read_register(0x6F)
        print(_fmt_read(drv))
        for r in (
            dummy.request,
            dummy.reply,
            gstat_before.request,
            gstat_before.reply,
            cleared,
            gstat_after.request,
            gstat_after.reply,
            drv.request,
            drv.reply,
        ):
            r.raise_on_faults()

        # Optional list of registers to read after the sequence
        regs: List[int] = []
        for item in args.register:
            key = str(item).strip().lower()
            if key in ("drv_status", "drv"):
                address = 0x6F
            elif key.startswith("0x") or key.isdigit():
                try:
                    address = int(key, 0)
                except Exception:
                    address = None
            else:
                address = None
            if address is None or not (0 <= address <= 0x7F):
                continue
            regs.append(address)
        if regs:
            print("Additional register reads:")
            for rr in driver.read_registers(regs):
                print(_fmt_read(rr))
                rr.raise_on_faults()

    print("Status with clear sequence done.")
    return 0
