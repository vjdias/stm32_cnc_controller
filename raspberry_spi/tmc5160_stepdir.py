"""Helper subcommand to initialize TMC5160 for external STEP/DIR.

Provides the `init-stepdir` subcommand used by tmc5160_cli.py.
"""
from __future__ import annotations

import argparse
from typing import List, Sequence, Tuple

try:
    # Package execution
    from .tmc5160 import (
        REG_CHOPCONF,
        REG_GCONF,
        REG_GSTAT,
        REG_IHOLD_IRUN,
        REG_PWMCONF,
        REG_TPOWERDOWN,
        REG_TPWMTHRS,
        TMC5160Configurator,
        TMC5160TransferResult,
        TMC5160RegisterPreset,
        TMC5160ReadResult,
    )
except Exception:  # pragma: no cover
    # Script execution
    from tmc5160 import (  # type: ignore
        REG_CHOPCONF,
        REG_GCONF,
        REG_GSTAT,
        REG_IHOLD_IRUN,
        REG_PWMCONF,
        REG_TPOWERDOWN,
        REG_TPWMTHRS,
        TMC5160Configurator,
        TMC5160TransferResult,
        TMC5160RegisterPreset,
        TMC5160ReadResult,
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


def _build_init_stepdir_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Apply basic configuration for TMC5160 with external STEP/DIR."
        ),
    )
    _add_common_spi_arguments(parser)
    parser.add_argument("--ihold", type=int, default=10, help="IHOLD (0-31)")
    parser.add_argument("--irun", type=int, default=31, help="IRUN (0-31)")
    parser.add_argument(
        "--ihold-delay", dest="iholddelay", type=int, default=6, help="IHOLDDELAY (0-15)"
    )
    parser.add_argument(
        "--microsteps",
        type=int,
        choices=[256, 128, 64, 32, 16, 8, 4, 2, 1],
        default=16,
        help="Microstepping resolution for external STEP pulses",
    )
    parser.add_argument(
        "--interpolate",
        dest="interpolate",
        action="store_true",
        default=True,
        help="Enable interpolation (INTPOL=1) to 256 microsteps",
    )
    parser.add_argument(
        "--no-interpolate",
        dest="interpolate",
        action="store_false",
        help="Disable interpolation (INTPOL=0)",
    )
    parser.add_argument(
        "--stealth",
        dest="stealth",
        action="store_true",
        default=True,
        help="Enable StealthChop (GCONF.EN_PWM_MODE=1)",
    )
    parser.add_argument(
        "--no-stealth",
        dest="stealth",
        action="store_false",
        help="Disable StealthChop (EN_PWM_MODE=0)",
    )
    parser.add_argument(
        "--tpowerdown",
        type=lambda x: int(x, 0),
        default=0x14,
        help="TPOWERDOWN (ticks of 2^18/fCLK)",
    )
    parser.add_argument(
        "--tpwmthrs",
        type=lambda x: int(x, 0),
        default=0x000001F4,
        help="TPWMTHRS threshold (StealthChop <-> SpreadCycle)",
    )
    return parser


def run_init_stepdir(
    argv: Sequence[str],
    *,
    configurator_factory=TMC5160Configurator,
    device_finder=lambda: [],
):
    parser = _build_init_stepdir_parser()
    args = parser.parse_args(list(argv))

    def _mres_from_microsteps(ms: int) -> int:
        table = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
        return table[ms]

    ihold = max(0, min(31, int(args.ihold)))
    irun = max(0, min(31, int(args.irun)))
    iholddelay = max(0, min(15, int(args.iholddelay)))
    mres = _mres_from_microsteps(args.microsteps)

    gconf = 0x00000000
    if args.stealth:
        gconf |= (1 << 2)  # EN_PWM_MODE

    ihold_irun = (ihold & 0x1F) | ((irun & 0x1F) << 8) | ((iholddelay & 0x0F) << 16)

    base_chopconf = 0x14010053
    chopconf = (base_chopconf & ~(0x0F << 24)) | ((mres & 0x0F) << 24)
    if args.interpolate:
        chopconf |= (1 << 28)
    else:
        chopconf &= ~(1 << 28)

    tpowerdown = int(args.tpowerdown)
    tpwmthrs = int(args.tpwmthrs)
    pwmconf = 0xC10D0024

    writes: List[Tuple[int, int]] = [
        (REG_GSTAT, 0x00000007),
        (REG_GCONF, gconf),
        (REG_IHOLD_IRUN, ihold_irun),
        (REG_TPOWERDOWN, tpowerdown),
        (REG_TPWMTHRS, tpwmthrs),
        (REG_CHOPCONF, chopconf),
        (REG_PWMCONF, pwmconf),
    ]

    configurator = configurator_factory(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset.default(),
    )

    with configurator as driver:  # type: ignore
        print(
            f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para inicializar STEP/DIR"
        )
        print("Aplicando preset padrão + ajustes específicos de STEP/DIR")
        responses: List[TMC5160TransferResult] = []
        responses.extend(driver.configure())
        responses.extend(driver.apply_registers(writes))
        print("TMC5160 responses:")
        for res in responses:
            print(f"- 0x{res.address:02X} <= 0x{res.value:08X}")
            print(f"  Raw response: {res.raw_hex}")
            print(f"  Status 0x{res.status.raw:02X}: {res.status.summary()}")
            print(f"  Previous data: 0x{res.previous_data:08X}")
            # Observação: o byte de status SPI pode sinalizar driver_error=1 de forma transitória
            # (ex.: UV_CP/DRV_ERR latched) até a limpeza de GSTAT/condições de VM/EN.
            # Não interrompemos a configuração aqui; a verificação final usa GSTAT/DRV_STATUS.

        # Somente registradores legíveis por SPI. IHOLD_IRUN/TPWMTHRS/TPOWERDOWN/PWMCONF
        # são somente-escrita no TMC5160 e tendem a retornar 0 quando lidos.
        verify_regs = [
            REG_GSTAT,
            0x6F,
            REG_GCONF,
            REG_CHOPCONF,
        ]
        print("Leituras de verificação:")
        results: List[TMC5160ReadResult] = driver.read_registers(verify_regs)
        gstat_val = None
        drv_val = None
        for r in results:
            name = {
                REG_GSTAT: "GSTAT",
                0x6F: "DRV_STATUS",
                REG_GCONF: "GCONF",
                REG_CHOPCONF: "CHOPCONF",
            }.get(r.address, f"0x{r.address:02X}")
            print(f"- {name} (0x{r.address:02X}) => 0x{r.value:08X}")
            print(f"  Requisição: status=0x{r.request.status.raw:02X}, resp={r.request.raw_hex}")
            print(f"  Resposta  : status=0x{r.reply.status.raw:02X}, resp={r.reply.raw_hex}")
            # Não encerramos por driver_error no byte de status; avaliamos registradores abaixo
            if r.address == REG_GSTAT:
                gstat_val = r.value
            elif r.address == 0x6F:
                drv_val = r.value

        # Checagem consolidada de falhas persistentes
        faults: List[str] = []
        if gstat_val is not None:
            if gstat_val & (1 << 1):
                faults.append("GSTAT.DRV_ERR=1")
            if gstat_val & (1 << 2):
                faults.append("GSTAT.UV_CP=1")
        if drv_val is not None:
            if drv_val & (1 << 26):
                faults.append("DRV_STATUS.OT=1")
            if drv_val & (1 << 25):
                faults.append("DRV_STATUS.OTPW=1")
            if drv_val & (1 << 24):
                faults.append("DRV_STATUS.S2GA=1")
            if drv_val & (1 << 23):
                faults.append("DRV_STATUS.S2GB=1")
            if drv_val & (1 << 22):
                faults.append("DRV_STATUS.S2VSA=1")
            if drv_val & (1 << 21):
                faults.append("DRV_STATUS.S2VSB=1")
            if drv_val & (1 << 20):
                faults.append("DRV_STATUS.OLA=1")
            if drv_val & (1 << 19):
                faults.append("DRV_STATUS.OLB=1")

        if faults:
            print("Falhas detectadas após configuração:")
            for f in faults:
                print(f"  - {f}")
            print("Dica: confira alimentação VM/ENN, conexões do motor e limpe GSTAT (0x01=0x07).")
            # Não lançamos exceção para permitir diagnóstico contínuo via CLI

    print("Inicialização STEP/DIR concluída.")
    return 0


