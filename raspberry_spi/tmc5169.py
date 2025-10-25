#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Utilitário básico para TMC5169: modo de segurança (safe-off).

Este módulo aplica uma sequência segura aos drivers TMC5169 conectados via SPI:
- Limpa GSTAT
- Zera IHOLD/IRUN (IHOLD_IRUN=0x00000000)
- Desliga o chopper (CHOPCONF.TOFF=0)
- Coloca as pontes em alta impedância (PWMCONF.FREEWHEEL=3 por padrão)

Observação: Reutiliza o configurador SPI do TMC5160, pois o framing e os
endereços de registradores relevantes são idênticos para essa operação.
"""
from __future__ import annotations

import argparse
from typing import Iterable, List, Sequence, Tuple

try:
    # Execução como pacote
    from .tmc5160 import (
        TMC5160Configurator as _Configurator,
        TMC5160RegisterPreset as _RegisterPreset,
    )
except Exception:  # pragma: no cover - execução direta do arquivo
    # Execução como script a partir do diretório raspberry_spi
    from tmc5160 import (  # type: ignore
        TMC5160Configurator as _Configurator,
        TMC5160RegisterPreset as _RegisterPreset,
    )


# Endereços relevantes do TMC5169 (iguais ao TMC5160 para estes registros)
REG_GSTAT = 0x01
REG_IHOLD_IRUN = 0x10
REG_CHOPCONF = 0x6C
REG_PWMCONF = 0x70


def _apply_safe_off(
    *,
    bus: int,
    dev: int,
    speed_hz: int,
    freewheel: int = 3,
    verify: bool = True,
) -> None:
    ihold_irun_safe = 0x00000000  # IHOLD=0, IRUN=0, IHOLDDELAY=0
    chopconf_base = 0x14010053
    chopconf_safe = chopconf_base & ~0x0F  # TOFF=0
    pwmconf_base = 0xC10D0024
    freewheel = int(freewheel) & 0x3
    pwmconf_safe = (pwmconf_base & ~(0x3 << 20)) | (freewheel << 20)

    configurator = _Configurator(
        bus=bus,
        device=dev,
        speed_hz=speed_hz,
        register_preset=_RegisterPreset(writes=tuple()),  # não aplicar preset
    )
    with configurator as driver:  # type: ignore
        writes: List[Tuple[int, int]] = [
            (REG_GSTAT, 0x00000007),
            (REG_IHOLD_IRUN, ihold_irun_safe),
            (REG_CHOPCONF, chopconf_safe),
            (REG_PWMCONF, pwmconf_safe),
        ]
        for addr, val in writes:
            res = driver.write_register(addr, val)
            print(f"write 0x{addr:02X} <= 0x{val:08X} | status=0x{res.status.raw:02X} resp={res.raw_hex}")
            res.raise_on_faults()
        if verify:
            for addr in (REG_IHOLD_IRUN, REG_CHOPCONF, REG_PWMCONF):
                rr = driver.read_register(addr)
                print(f"read  0x{addr:02X} => 0x{rr.value:08X} | req=0x{rr.request.status.raw:02X} rep=0x{rr.reply.status.raw:02X}")
                rr.raise_on_faults()


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="TMC5169 utilitário: safe-off (corrente=0, TOFF=0, FREEWHEEL)")
    p.add_argument("--bus", type=int, default=0, help="SPI bus (default: 0)")
    p.add_argument("--dev", type=int, default=1, help="SPI device (default: 1)")
    p.add_argument("--speed", type=int, default=4_000_000, help="SPI speed Hz (default: 4_000_000)")
    p.add_argument("--freewheel", type=int, choices=[0, 1, 2, 3], default=3, help="PWMCONF.FREEWHEEL (default: 3)")
    p.add_argument("--no-verify", dest="verify", action="store_false", help="Não ler de volta após aplicar")
    return p


def main(argv: Sequence[str] | None = None) -> int:  # pragma: no cover - camada de CLI
    parser = _build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)
    try:
        _apply_safe_off(bus=args.bus, dev=args.dev, speed_hz=args.speed, freewheel=args.freewheel, verify=args.verify)
    except Exception as exc:
        print("Erro no safe-off TMC5169:", exc)
        return 1
    print("TMC5169 safe-off concluído.")
    return 0


if __name__ == "__main__":  # pragma: no cover
    import sys as _sys
    raise SystemExit(main(_sys.argv[1:]))

