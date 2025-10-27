#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Interface de linha de comando para configurar o driver TMC5160."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import time
from typing import List, Optional, Sequence, Tuple

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .tmc5160 import (
        REG_CHOPCONF,
        REG_GCONF,
        REG_GSTAT,
        REG_IHOLD_IRUN,
        REG_GLOBAL_SCALER,
        REG_PWMCONF,
        REG_TPOWERDOWN,
        REG_TPWMTHRS,
        TMC5160Configurator,
        TMC5160ReadResult,
        TMC5160RegisterPreset,
        TMC5160TransferResult,
        decode_register_value,
    )
else:  # execuÃƒÂ§ÃƒÂ£o direta dos testes/script
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from tmc5160 import (  # type: ignore
        REG_CHOPCONF,
        REG_GCONF,
        REG_GSTAT,
        REG_IHOLD_IRUN,
        REG_GLOBAL_SCALER,
        REG_PWMCONF,
        REG_TPOWERDOWN,
        REG_TPWMTHRS,
        TMC5160Configurator,
        TMC5160ReadResult,
        TMC5160RegisterPreset,
        TMC5160TransferResult,
        decode_register_value,
    )

REGISTER_ALIASES = {
    "gstat": REG_GSTAT,
    "gconf": REG_GCONF,
    "ihold_irun": REG_IHOLD_IRUN,
    "ihold-irun": REG_IHOLD_IRUN,
    "tpowerdown": REG_TPOWERDOWN,
    "tpwmthrs": REG_TPWMTHRS,
    "chopconf": REG_CHOPCONF,
    "pwmconf": REG_PWMCONF,
    "globalscaler": REG_GLOBAL_SCALER,
    "drv_status": 0x6F,
    "drvstatus": 0x6F,
    "drv": 0x6F,
}

REGISTER_NAMES = {
    REG_GSTAT: "GSTAT",
    REG_GCONF: "GCONF",
    REG_IHOLD_IRUN: "IHOLD_IRUN",
    REG_TPOWERDOWN: "TPOWERDOWN",
    REG_TPWMTHRS: "TPWMTHRS",
    REG_CHOPCONF: "CHOPCONF",
    REG_PWMCONF: "PWMCONF",
    0x6F: "DRV_STATUS",
    REG_GLOBAL_SCALER: "GLOBALSCALER",
}

# Alguns registradores do TMC5160 sÃƒÂ£o somente-escrita (sem readback pelo SPI)
# e, por isso, retornam 0 em leituras: evitamos incluÃƒÂ­-los como padrÃƒÂ£o nas
# consultas de status para nÃƒÂ£o confundir o usuÃƒÂ¡rio. Esses valores ainda podem
# ser lidos explicitamente via --register, se desejado.
WRITE_ONLY_REGISTERS = (
    REG_IHOLD_IRUN,  # 0x10
    REG_TPOWERDOWN,  # 0x11
    REG_TPWMTHRS,    # 0x13
    REG_PWMCONF,     # 0x70
)

DEFAULT_STATUS_REGISTERS = (
    REG_GSTAT,       # 0x01 (RW)
    0x6F,            # DRV_STATUS (R)
    REG_GCONF,       # 0x00 (RW)
    REG_CHOPCONF,    # 0x6C (RW)
)


class CLIError(Exception):
    """Erro de validaÃƒÂ§ÃƒÂ£o dos parÃƒÂ¢metros da CLI."""


def _format_response(result: TMC5160TransferResult) -> str:
    status = result.status
    lines = [
        f"- 0x{result.address:02X} <= 0x{result.value:08X}",
        f"  Resposta bruta: {result.raw_hex}",
        f"  Status 0x{status.raw:02X}: {status.summary()}",
        f"  Dado retornado (comando anterior): 0x{result.previous_data:08X}",
    ]
    return "\n".join(lines)


def _format_read_result(result: TMC5160ReadResult) -> str:
    name = REGISTER_NAMES.get(result.address, f"0x{result.address:02X}")
    status_request = result.request.status
    status_reply = result.reply.status
    lines = [
        f"- {name} (0x{result.address:02X}) => 0x{result.value:08X}",
        f"  RequisiÃƒÂ§ÃƒÂ£o  : status=0x{status_request.raw:02X}, resposta={result.request.raw_hex}",
        f"  Resposta ÃƒÂºtil: status=0x{status_reply.raw:02X}, resposta={result.reply.raw_hex}",
    ]
    if status_request.raw == status_reply.raw:
        lines.append(f"  DiagnÃƒÂ³stico : {status_reply.summary()}")
    else:
        lines.append("  DiagnÃƒÂ³stico : {} / {}".format(status_request.summary(), status_reply.summary()))
    decoded = decode_register_value(result.address, result.value)
    if decoded:
        lines.append("  TraduÃƒÂ§ÃƒÂ£o    :")
        for item in decoded:
            lines.append(f"    - {item}")
    return "\n".join(lines)


def _parse_register_assignment(raw: str) -> Tuple[int, int]:
    if "=" not in raw:
        raise CLIError("Use o formato nome=valor ou endereco=valor (ex.: gconf=0x4)")

    name, value_text = raw.split("=", 1)
    name = name.strip().lower()
    value_text = value_text.strip()

    if not name:
        raise CLIError("Informe o nome ou endereÃƒÂ§o do registrador antes do '='")
    if not value_text:
        raise CLIError("Informe o valor do registrador apÃƒÂ³s o '='")

    if name in REGISTER_ALIASES:
        address = REGISTER_ALIASES[name]
    else:
        try:
            address = int(name, 0)
        except ValueError as exc:  # pragma: no cover - defensive
            raise CLIError(
                f"Registrador desconhecido '{name}'. Informe um dos aliases conhecidos "
                "ou um endereÃƒÂ§o numÃƒÂ©rico."
            ) from exc

    try:
        value = int(value_text, 0)
    except ValueError as exc:  # pragma: no cover - defensive
        raise CLIError(f"Valor invÃƒÂ¡lido '{value_text}'. Utilize decimal ou hexadecimal.") from exc

    if not 0 <= address <= 0x7F:
        raise CLIError("EndereÃƒÂ§o de registrador deve estar entre 0x00 e 0x7F")
    if not 0 <= value <= 0xFFFFFFFF:
        raise CLIError("Valor do registrador deve caber em 32 bits")

    return address, value


def _parse_register_name(raw: str) -> int:
    key = raw.strip().lower()
    if not key:
        raise CLIError("Informe o nome ou endereÃƒÂ§o do registrador a consultar")
    if key in REGISTER_ALIASES:
        return REGISTER_ALIASES[key]
    try:
        address = int(key, 0)
    except ValueError as exc:  # pragma: no cover - defensivo
        raise CLIError(
            f"Registrador desconhecido '{raw}'. Utilize um alias conhecido ou um endereÃƒÂ§o numÃƒÂ©rico."
        ) from exc
    if not 0 <= address <= 0x7F:
        raise CLIError("EndereÃƒÂ§o de registrador deve estar entre 0x00 e 0x7F")
    return address


def _add_common_spi_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--bus", type=int, default=0, help="Barramento SPI (default: 0)")
    parser.add_argument("--dev", type=int, default=1, help="Dispositivo SPI (default: 1)")
    parser.add_argument(
        "--speed",
        type=int,
        default=4_000_000,
        help="Velocidade SPI em Hz (default: 4_000_000)",
    )


def _build_configure_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Configura o driver TMC5160 atravÃƒÂ©s do barramento SPI do Raspberry Pi",
    )
    _add_common_spi_arguments(parser)
    parser.add_argument(
        "--no-defaults",
        action="store_true",
        help="NÃƒÂ£o enviar o preset padrÃƒÂ£o antes dos ajustes informados",
    )
    parser.add_argument(
        "--write",
        action="append",
        default=[],
        metavar="REG=VAL",
        help=(
            "SequÃƒÂªncia adicional de escritas de registrador (pode ser repetido). "
            "Aceita aliases (gconf, chopconf, ...) ou endereÃƒÂ§os numÃƒÂ©ricos."
        ),
    )
    parser.add_argument("--gconf", type=lambda x: int(x, 0), help="Valor para REG_GCONF")
    parser.add_argument("--gstat", type=lambda x: int(x, 0), help="Valor para REG_GSTAT")
    parser.add_argument(
        "--ihold-irun",
        dest="ihold_irun",
        type=lambda x: int(x, 0),
        help="Valor para REG_IHOLD_IRUN",
    )
    parser.add_argument(
        "--tpowerdown", type=lambda x: int(x, 0), help="Valor para REG_TPOWERDOWN"
    )
    parser.add_argument(
        "--tpwmthrs", type=lambda x: int(x, 0), help="Valor para REG_TPWMTHRS"
    )
    parser.add_argument(
        "--chopconf", type=lambda x: int(x, 0), help="Valor para REG_CHOPCONF"
    )
    parser.add_argument(
        "--pwmconf", type=lambda x: int(x, 0), help="Valor para REG_PWMCONF"
    parser.add_argument("--globalscaler", type=int, help="Valor para REG_GLOBAL_SCALER (0-255; 0 equivale a 256)")
    )
    return parser


def _build_status_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Consulta os registradores do TMC5160 sem alterar a configuraÃƒÂ§ÃƒÂ£o",
    )
    _add_common_spi_arguments(parser)
    parser.add_argument(
        "--clear-gstat",
        action="store_true",
        help=("Executa: leitura dummy, ler GSTAT, limpar GSTAT (0x07), "
              "ler GSTAT novamente e ler DRV_STATUS (0x6F) antes das leituras."),
    )
    parser.add_argument(
        "--register",
        action="append",
        default=[],
        metavar="REG",
        help=(
            "Lista de registradores a serem lidos (pode ser repetido). "
            "Aceita aliases conhecidos (gconf, gstat, drv_status, ...) ou endereÃƒÂ§os numÃƒÂ©ricos."
        ),
    )
    return parser


def _build_loop_test_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Gera um padrÃƒÂ£o repetitivo de escrita SPI para observar o barramento "
            "com instrumentos como osciloscÃƒÂ³pio ou analisador lÃƒÂ³gico."
        ),
    )
    _add_common_spi_arguments(parser)
    parser.add_argument(
        "--address",
        type=_parse_register_name,
        default=REG_GCONF,
        help=("Registrador a ser escrito (alias conhecido ou endereÃƒÂ§o numÃƒÂ©rico, default: gconf)."),
    )
    parser.add_argument(
        "--value",
        type=lambda x: int(x, 0),
        default=0x00000004,
        help=("Valor de 32 bits a ser enviado em todas as mensagens (default: 0x00000004)."),
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=0.01,
        help=(
            "Tempo em segundos entre mensagens consecutivas (default: 0.01). "
            "Utilize 0 para repetir o mais rÃƒÂ¡pido possÃƒÂ­vel."
        ),
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=0,
        help=(
            "NÃƒÂºmero de repetiÃƒÂ§ÃƒÂµes antes de encerrar automaticamente (0 = infinito, "
            "pressione Ctrl+C para parar)."
        ),
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="NÃƒÂ£o imprimir cada resposta, apenas o resumo inicial e final.",
    )
    return parser


def _build_init_stepdir_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=("Aplica configuraÃƒÂ§ÃƒÂµes bÃƒÂ¡sicas para operar o TMC5160 em STEP/DIR externo."),
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
        help="ResoluÃƒÂ§ÃƒÂ£o de microstepping para pulsos STEP externos",
    )
    parser.add_argument(
        "--interpolate",
        dest="interpolate",
        action="store_true",
        default=True,
        help="Ativa interpolaÃƒÂ§ÃƒÂ£o (INTPOL) para 256 micropassos",
    )
    parser.add_argument(
        "--no-interpolate",
        dest="interpolate",
        action="store_false",
        help="Desativa interpolaÃƒÂ§ÃƒÂ£o (INTPOL=0)",
    )
    parser.add_argument(
        "--stealth",
        dest="stealth",
        action="store_true",
        default=True,
        help="Ativa StealthChop (GCONF.EN_PWM_MODE=1)",
    )
    parser.add_argument(
        "--no-stealth",
        dest="stealth",
        action="store_false",
        help="Desativa StealthChop (EN_PWM_MODE=0)",
    )
    parser.add_argument(
        "--tpowerdown",
        type=lambda x: int(x, 0),
        default=0x14,
        help="TPOWERDOWN (ticks de 2^18/fCLK)",
    )
    parser.add_argument(
        "--tpwmthrs",
        type=lambda x: int(x, 0),
        default=0x000001F4,
        help="TPWMTHRS (limiar de StealthChopÃ¢â€ â€SpreadCycle)",
    )
    return parser


def _build_safe_off_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Coloca o TMC5160 em modo seguro: correntes=0, TOFF=0 e FREEWHEEL."
        ),
    )
    _add_common_spi_arguments(parser)
    parser.add_argument(
        "--freewheel",
        type=int,
        choices=[0, 1, 2, 3],
        default=3,
        help=(
            "PWM_FREEWHEEL (0=normal, 1=short break, 2=passive fast decay, 3=freewheeling). "
            "Default: 3"
        ),
    )
    parser.add_argument(
        "--toff",
        type=int,
        default=0,
        help="TOFF (0Ã¢â‚¬â€œ15). Default: 0 (desliga o chopper)",
    )
    parser.add_argument(
        "--clear-gstat",
        action="store_true",
        help="Limpa GSTAT (0x07) antes",
    )
    return parser


def _build_ultrafrio_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Aplica preset ultra-frio: SpreadCycle, 1/16+interpolaÃƒÂ§ÃƒÂ£o, TOFF/HEND ajustados, "
            "correntes baixas e GLOBAL_SCALER.")
    )
    _add_common_spi_arguments(parser)
    parser.add_argument("--ihold", type=int, default=1, help="IHOLD (0-31)")
    parser.add_argument("--irun", type=int, default=1, help="IRUN (0-31)")
    parser.add_argument(
        "--ihold-delay", dest="iholddelay", type=int, default=6, help="IHOLDDELAY (0-15)"
    )
    parser.add_argument(
        "--microsteps",
        type=int,
        choices=[256, 128, 64, 32, 16, 8, 4, 2, 1],
        default=16,
        help="ResoluÃƒÂ§ÃƒÂ£o de microstepping",
    )
    parser.add_argument(
        "--interpolate",
        dest="interpolate",
        action="store_true",
        default=True,
        help="Ativa interpolaÃƒÂ§ÃƒÂ£o (INTPOL=1) para 256 Ã‚Âµsteps",
    )
    parser.add_argument(
        "--no-interpolate",
        dest="interpolate",
        action="store_false",
        help="Desativa interpolaÃƒÂ§ÃƒÂ£o (INTPOL=0)",
    )
    parser.add_argument("--toff", type=int, default=5, help="CHOPCONF.TOFF (0-15)")
    parser.add_argument("--hstrt", type=int, default=5, help="CHOPCONF.HSTRT (0-7)")
    parser.add_argument("--hend", type=int, default=2, help="CHOPCONF.HEND (0-15)")
    parser.add_argument("--tbl", type=int, default=2, choices=[0, 1, 2, 3], help="CHOPCONF.TBL (0-3)")
    parser.add_argument("--globalscaler", type=int, default=32, help="GLOBAL_SCALER (0-255)")
    parser.add_argument(
        "--tpowerdown",
        type=lambda x: int(x, 0),
        default=0x14,
        help="TPOWERDOWN (ticks de 2^18/fCLK)",
    )
    parser.add_argument(
        "--clear-gstat",
        action="store_true",
        help="Limpa GSTAT (0x07) antes de aplicar as configuraÃƒÂ§ÃƒÂµes",
    )
    return parser


def _build_status_compact_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Mostra o status resumido dos drivers (byte de status SPI, bytes brutos), "
            "e se houver driver_error, lÃƒÂª GSTAT/DRV_STATUS para diagnÃƒÂ³stico."
        ),
    )
    # Para este comando, deixe --dev opcional e ofereÃƒÂ§a --devs (lista)
    parser.add_argument("--bus", type=int, default=0, help="Barramento SPI (default: 0)")
    parser.add_argument("--dev", type=int, default=None, help="Dispositivo SPI (opcional)")
    parser.add_argument(
        "--devs",
        type=str,
        default=None,
        help="Lista de dispositivos CS separada por vÃƒÂ­rgula (ex.: 1,2,3). Ignorado se --dev for informado.",
    )
    parser.add_argument(
        "--speed",
        type=int,
        default=4_000_000,
        help="Velocidade SPI em Hz (default: 4_000_000)",
    )
    parser.add_argument(
        "--clear-gstat",
        action="store_true",
        help="Executa a limpeza de GSTAT (0x07) antes de ler o status",
    )
    return parser


def _build_motion_params_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "LÃƒÂª parÃƒÂ¢metros de movimentaÃƒÂ§ÃƒÂ£o do TMC5160 (modo, microstepping, CHOPCONF, correntes, GLOBALSCALER).\n"
            "ObservaÃƒÂ§ÃƒÂ£o: IHOLD_IRUN/TPOWERDOWN/PWMCONF podem ser somente-escrita no 5160; leitura tende a 0."
        ),
    )
    _add_common_spi_arguments(parser)
    parser.add_argument(
        "--clear-gstat",
        action="store_true",
        help="Executa a limpeza de GSTAT (0x07) antes de ler (opcional)",
    )
    return parser


def _collect_overrides(args: argparse.Namespace) -> List[Tuple[int, int]]:
    overrides: List[Tuple[int, int]] = []

    option_map = {
        "gstat": args.gstat,
        "gconf": args.gconf,
        "ihold_irun": args.ihold_irun,
        "tpowerdown": args.tpowerdown,
        "tpwmthrs": args.tpwmthrs,
        "chopconf": args.chopconf,
        "pwmconf": args.pwmconf,
    }
        "globalscaler": args.globalscaler,

    for name, value in option_map.items():
        if value is None:
            continue
        overrides.append((REGISTER_ALIASES[name], value))

    for write in args.write:
        overrides.append(_parse_register_assignment(write))

    return overrides


def _collect_status_registers(args: argparse.Namespace) -> List[int]:
    if args.register:
        return [_parse_register_name(item) for item in args.register]
    return list(DEFAULT_STATUS_REGISTERS)


def _report_missing_device(
    args: argparse.Namespace, spi_node: str, device_finder
) -> int:
    available = [str(path) for path in device_finder()]
    if available:
        available_hint = "Dispositivos disponÃƒÂ­veis: {}. ".format(", ".join(sorted(available)))
    else:
        available_hint = "Nenhum dispositivo /dev/spidev* encontrado no sistema. "

    if args.bus == 0:
        overlay_hint = "dtparam=spi=on"
    elif args.bus == 1:
        overlay_hint = "dtoverlay=spi1-3cs"
    else:
        overlay_hint = f"dtoverlay=spi{args.bus}-1cs"

    print(
        (
            "Erro: dispositivo SPI '{spi}' nÃƒÂ£o encontrado. {available}"
            "Habilite o overlay SPI correspondente (ex.: {overlay}) ou ajuste --bus/--dev."
        ).format(spi=spi_node, available=available_hint, overlay=overlay_hint),
        file=sys.stderr,
    )
    return 2


def _report_permission_denied(spi_node: str) -> int:
    print(
        "Erro: permissÃƒÂ£o negada ao acessar '{spi}'. Execute o comando com sudo ou ajuste as "
        "regras de acesso (ex.: grupo 'spi').".format(spi=spi_node),
        file=sys.stderr,
    )
    return 3


def _run_spi_operation(
    args: argparse.Namespace,
    configurator,
    device_finder,
    *,
    success_message: str,
    operation,
) -> int:
    spi_node = f"/dev/spidev{args.bus}.{args.dev}"
    try:
        with configurator as driver:  # type: ignore[assignment]
            completed = operation(driver)
    except FileNotFoundError:
        return _report_missing_device(args, spi_node, device_finder)
    except PermissionError:
        return _report_permission_denied(spi_node)
    except RuntimeError as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        return 4

    if completed is False:
        return 130

    print(success_message)
    return 0


def _run_motion_params(
    argv: Sequence[str],
    *,
    configurator_factory,
    device_finder,
) -> int:
    parser = _build_motion_params_parser()
    args = parser.parse_args(list(argv))

    configurator = configurator_factory(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset(writes=tuple()),
    )

    def _field(val: int, start: int, width: int) -> int:
        mask = (1 << width) - 1
        return (val >> start) & mask

    def _msteps_from_mres(mres: int) -> Optional[int]:
        if 0 <= mres <= 8:
            return 1 << (8 - mres)
        return None

    try:
        with configurator as driver:  # type: ignore[assignment]
            if args.clear_gstat:
                # Limpa GSTAT para evitar flags antigas na sessÃƒÂ£o
                driver.write_register(REG_GSTAT, 0x00000007)

            # Leitura dos registradores relevantes
            r_gconf = driver.read_register(REG_GCONF)
            r_chop = driver.read_register(REG_CHOPCONF)
            # Write-only em muitos 5160; ainda tentamos ler e avisamos
            r_ihir = driver.read_register(REG_IHOLD_IRUN)
            r_tpwd = driver.read_register(REG_TPOWERDOWN)
            # GLOBAL_SCALER (0x0B) Ã¢â‚¬â€ pode ser R/W dependendo da variante
            try:
                r_gs = driver.read_register(REG_GLOBAL_SCALER)
            except Exception:
                r_gs = None

            # DecodificaÃƒÂ§ÃƒÂ£o bÃƒÂ¡sica
            gconf = r_gconf.value
            chop = r_chop.value
            ihir = r_ihir.value
            tpwd = r_tpwd.value
            gs = r_gs.value if r_gs is not None else None

            en_pwm_mode = (gconf >> 2) & 1
            intpol = (chop >> 28) & 1
            mres = _field(chop, 24, 4)
            microsteps = _msteps_from_mres(mres)
            vsense = (chop >> 17) & 1
            toff = _field(chop, 0, 4)
            hstrt = _field(chop, 4, 3)
            hend = _field(chop, 7, 4)
            tbl = _field(chop, 15, 2)

            ihold = _field(ihir, 0, 5)
            irun = _field(ihir, 8, 5)
            iholddelay = _field(ihir, 16, 4)

            # TPOWERDOWN em ticks de 2^18/fCLK (~12 MHz)
            tpwd_ticks = tpwd & 0xFF
            tpwd_ms = (tpwd_ticks * (2 ** 18) / 12_000_000.0) * 1000.0 if tpwd_ticks else 0.0

            print(f"ParÃƒÂ¢metros (bus={args.bus}, dev={args.dev})")
            print(f"- Modo             : {'SpreadCycle' if en_pwm_mode == 0 else 'StealthChop (EN_PWM_MODE=1)'}")
            if microsteps:
                print(f"- Microstep        : 1/{microsteps}  (MRES={mres})")
            else:
                print(f"- Microstep        : MRES={mres} (valor fora da faixa esperada)")
            print(f"- InterpolaÃƒÂ§ÃƒÂ£o     : {'ON' if intpol else 'OFF'} (INTPOL={intpol})")
            print(f"- VSENSE           : {vsense}  (0=0,325 V; 1=0,18 V)")
            print(f"- TOFF             : {toff}")
            print(f"- TBL              : {tbl}")
            print(f"- HSTRT            : {hstrt}")
            print(f"- HEND             : {hend}")

            # IHOLD/IRUN podem retornar 0 (write-only); informamos mesmo assim
            note_wr = " (write-only: leitura pode retornar 0)"
            print(f"- IRUN (CS)        : {irun}{note_wr}")
            print(f"- IHOLD (CS)       : {ihold}{note_wr}")
            print(f"- IHOLDDELAY       : {iholddelay}{note_wr}")

            if gs is None:
                print(f"- GLOBALSCALER     : N/D (nÃƒÂ£o lido)")
            else:
                print(f"- GLOBALSCALER     : {gs & 0xFF}")

            print(f"- TPOWERDOWN       : {tpwd_ticks} (~{tpwd_ms:.1f} ms){note_wr}")

            # RecomendaÃƒÂ§ÃƒÂµes (perfil ultra-frio)
            print("Recomendado (ultra-frio):")
            print("  - Modo           : SpreadCycle")
            print("  - Microstep      : 1/16 + INTPOL=1")
            print("  - VSENSE         : 0 (0,325 V)")
            print("  - TOFF/TBL       : 5 / 2")
            print("  - HSTRT/HEND     : 5 / 2 (ajuste fino Ã‚Â±1)")
            print("  - IRUN/IHOLD     : 1 / 1 (diagnÃƒÂ³stico; ajuste conforme torque)")
            print("  - IHOLDDELAY     : 4Ã¢â‚¬â€œ8")
            print("  - GLOBALSCALER   : 32 (ponto de partida)")
            print("  - TPOWERDOWN     : 20 (~2 s)")

    except FileNotFoundError:
        print(f"Dispositivo /dev/spidev{args.bus}.{args.dev} nÃƒÂ£o encontrado")
        return 1
    except PermissionError:
        print(f"PermissÃƒÂ£o negada em /dev/spidev{args.bus}.{args.dev}")
        return 1
    except RuntimeError as exc:
        # Mantenha leitura parcial para auxiliar diagnÃƒÂ³stico
        print(f"Erro durante leitura: {exc}")
        return 1

    return 0



def _run_configure(
    argv: Sequence[str],
    *,
    configurator_factory,
    device_finder,
) -> int:
    parser = _build_configure_parser()
    args = parser.parse_args(list(argv))

    overrides = _collect_overrides(args)

    preset = (
        TMC5160RegisterPreset(writes=tuple())
        if args.no_defaults
        else TMC5160RegisterPreset.default()
    )

    configurator = configurator_factory(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=preset,
    )

    def _operation(driver):
        print(f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para configurar o TMC5160")
        responses: List[TMC5160TransferResult] = []
        if not args.no_defaults:
            count = len(preset.writes)
            print(f"Aplicando preset padrÃƒÂ£o ({count} registradores)")
            responses.extend(driver.configure())
        if overrides:
            print("Aplicando ajustes adicionais:")
            for address, value in overrides:
                print(f" - 0x{address:02X} = 0x{value:08X}")
            responses.extend(driver.apply_registers(overrides))
        else:
            print("Nenhum ajuste adicional informado; mantendo preset padrÃƒÂ£o.")

        if responses:
            print("Respostas do TMC5160:")
            try:
                for result in responses:
                    print(_format_response(result))
                    result.raise_on_faults()
            except RuntimeError as exc:
                # DiagnÃƒÂ³stico detalhado com base em GSTAT e DRV_STATUS
                try:
                    gstat_read = driver.read_register(REG_GSTAT)
                    drv_read = driver.read_register(0x6F)
                    gstat_val = gstat_read.value
                    drv_val = drv_read.value
                    gstat_text = decode_register_value(REG_GSTAT, gstat_val)
                    drv_text = decode_register_value(0x6F, drv_val)
                    details = []
                    details.append(
                        f"GSTAT=0x{gstat_val:08X}: " + ("; ".join(gstat_text) if gstat_text else "")
                    )
                    details.append(
                        f"DRV_STATUS=0x{drv_val:08X}: " + ("; ".join(drv_text) if drv_text else "")
                    )
                    raise RuntimeError(str(exc) + " | DiagnÃƒÂ³stico: " + " | ".join(details)) from exc
                except Exception:
                    raise

    return _run_spi_operation(
        args,
        configurator,
        device_finder,
        success_message="ConfiguraÃƒÂ§ÃƒÂ£o concluÃƒÂ­da.",
        operation=_operation,
    )


def _run_status(
    argv: Sequence[str],
    *,
    configurator_factory,
    device_finder,
) -> int:
    parser = _build_status_parser()
    args = parser.parse_args(list(argv))

    registers = _collect_status_registers(args)
    configurator = configurator_factory(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset.default(),
    )

    register_list = ", ".join(REGISTER_NAMES.get(addr, f"0x{addr:02X}") for addr in registers)

    def _operation(driver):
        print(f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para consultar o TMC5160")
        if args.clear_gstat:
            print("Executando sequÃƒÂªncia clear-GSTAT:")
            dummy = driver.read_register(REG_GCONF)
            print(_format_read_result(dummy))
            gstat_before = driver.read_register(REG_GSTAT)
            print(_format_read_result(gstat_before))
            cleared = driver.write_register(REG_GSTAT, 0x00000007)
            print(_format_response(cleared))
            gstat_after = driver.read_register(REG_GSTAT)
            print(_format_read_result(gstat_after))
            drv = driver.read_register(0x6F)
            print(_format_read_result(drv))
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
        if registers:
            print(f"Consultando registradores: {register_list}")
        else:
            print("Nenhum registrador informado; nada serÃƒÂ¡ lido.")
            return

        results = driver.read_registers(registers)
        print("Respostas do TMC5160:")
        for result in results:
            print(_format_read_result(result))
            result.raise_on_faults()

    return _run_spi_operation(
        args,
        configurator,
        device_finder,
        success_message="Consulta concluÃƒÂ­da.",
        operation=_operation,
    )


def _run_loop_test(
    argv: Sequence[str],
    *,
    configurator_factory,
    device_finder,
) -> int:
    parser = _build_loop_test_parser()
    args = parser.parse_args(list(argv))

    if not 0 <= args.value <= 0xFFFFFFFF:
        raise CLIError("O valor informado precisa caber em 32 bits (0x00000000-0xFFFFFFFF)")
    if args.interval < 0:
        raise CLIError("O intervalo entre mensagens deve ser maior ou igual a zero")
    if args.iterations < 0:
        raise CLIError("O nÃƒÂºmero de repetiÃƒÂ§ÃƒÂµes deve ser zero (infinito) ou positivo")

    configurator = configurator_factory(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset.default(),
    )

    register_name = REGISTER_NAMES.get(args.address, f"0x{args.address:02X}")

    def _operation(driver):
        print(
            "Abrindo SPI bus={bus} dev={dev} a {speed} Hz para gerar padrÃƒÂ£o de teste".format(
                bus=args.bus, dev=args.dev, speed=args.speed
            )
        )
        if args.iterations:
            print(f"RepetiÃƒÂ§ÃƒÂµes solicitadas: {args.iterations}")
        else:
            print("RepetiÃƒÂ§ÃƒÂµes solicitadas: infinito (interrompa com Ctrl+C)")
        if args.interval > 0:
            print(f"Intervalo entre mensagens: {args.interval:.6f} s")
        else:
            print("Intervalo entre mensagens: sem pausa (taxa mÃƒÂ¡xima permitida pela SPI)")

        print(
            "Registrador {name} (0x{addr:02X}) <= 0x{value:08X}".format(
                name=register_name, addr=args.address, value=args.value
            )
        )

        count = 0
        try:
            while args.iterations == 0 or count < args.iterations:
                result = driver.write_register(args.address, args.value)
                count += 1
                if not args.quiet:
                    print(
                        "[{count}] status=0x{status:02X} resposta={raw}".format(
                            count=count,
                            status=result.status.raw,
                            raw=result.raw_hex,
                        )
                    )
                result.raise_on_faults()
                if args.interval > 0:
                    time.sleep(args.interval)
        except KeyboardInterrupt:
            print(f"Loop interrompido pelo usuÃƒÂ¡rio apÃƒÂ³s {count} iteraÃƒÂ§ÃƒÂµes.")
            return False

        print(f"Loop concluÃƒÂ­do apÃƒÂ³s {count} iteraÃƒÂ§ÃƒÂµes.")
        return True

    return _run_spi_operation(
        args,
        configurator,
        device_finder,
        success_message="PadrÃƒÂ£o de teste finalizado.",
        operation=_operation,
    )


def _run_safe_off(
    argv: Sequence[str],
    *,
    configurator_factory,
    device_finder,
) -> int:
    parser = _build_safe_off_parser()
    args = parser.parse_args(list(argv))

    # NÃƒÂ£o aplicar preset no modo seguro
    configurator = configurator_factory(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset(writes=tuple()),
    )

    def _operation(driver):
        print(
            f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para modo seguro (safe-off)"
        )

        if args.clear_gstat:
            cleared = driver.write_register(REG_GSTAT, 0x00000007)
            print(_format_response(cleared))
            cleared.raise_on_faults()

        # Leitura para preservar campos e fazer read-modify-write
        chop_old = driver.read_register(REG_CHOPCONF)
        pwm_old = driver.read_register(REG_PWMCONF)
        print(_format_read_result(chop_old))
        print(_format_read_result(pwm_old))

        chop = chop_old.value
        pwm = pwm_old.value

        # CHOPCONF[3:0] = TOFF
        new_chop = (chop & ~0xF) | (int(args.toff) & 0xF)

        # PWMCONF: FREEWHEEL em [21:20]; tambÃƒÂ©m zera AUTOSCALE(18)/AUTOGRAD(19)
        new_pwm = (pwm & ~(0b11 << 20) & ~(1 << 18) & ~(1 << 19)) | (
            (int(args.freewheel) & 0b11) << 20
        )

        responses: List[TMC5160TransferResult] = []
        # Zera correntes
        responses.append(driver.write_register(REG_IHOLD_IRUN, 0x00000000))
        # Aplica TOFF
        responses.append(driver.write_register(REG_CHOPCONF, new_chop))
        # Coloca FREEWHEEL
        responses.append(driver.write_register(REG_PWMCONF, new_pwm))

        print("Respostas do TMC5160:")
        for r in responses:
            print(_format_response(r))
            r.raise_on_faults()

    return _run_spi_operation(
        args,
        configurator,
        device_finder,
        success_message="Safe-off aplicado.",
        operation=_operation,
    )


def _run_preset_ultrafrio(
    argv: Sequence[str],
    *,
    configurator_factory,
    device_finder,
) -> int:
    parser = _build_ultrafrio_parser()
    args = parser.parse_args(list(argv))

    def _mres_from_microsteps(ms: int) -> int:
        table = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
        return table[ms]

    # Clamp inputs
    ihold = max(0, min(31, int(args.ihold)))
    irun = max(0, min(31, int(args.irun)))
    iholddelay = max(0, min(15, int(args.iholddelay)))
    toff = max(0, min(15, int(args.toff)))
    hstrt = max(0, min(7, int(args.hstrt)))
    hend = max(0, min(15, int(args.hend)))
    tbl = max(0, min(3, int(args.tbl)))
    mres = _mres_from_microsteps(args.microsteps)
    globalscaler = max(0, min(255, int(args.globalscaler)))
    tpowerdown = int(args.tpowerdown)

    # Build registers
    gconf = 0x00000000  # SpreadCycle (EN_PWM_MODE=0)

    ihold_irun = (ihold & 0x1F) | ((irun & 0x1F) << 8) | ((iholddelay & 0x0F) << 16)

    # Compose CHOPCONF with requested fields; keep other protections enabled
    chop = 0
    chop |= (toff & 0x0F)
    chop |= (hstrt & 0x07) << 4
    chop |= (hend & 0x0F) << 7
    chop |= (tbl & 0x03) << 15
    # VSENSE=0 (0.325 V) -> bit 17 stays 0
    # SYNC/TPFD left at 0
    chop |= (mres & 0x0F) << 24
    if args.interpolate:
        chop |= (1 << 28)  # INTPOL

    # Ensure TMC5160-specific protections not disabled
    # DISS2G=0 (bit 30), DISS2VS=0 (bit 31)

    writes: List[Tuple[int, int]] = [
        (REG_GCONF, gconf),
        (REG_IHOLD_IRUN, ihold_irun),
        (REG_TPOWERDOWN, tpowerdown),
        (REG_CHOPCONF, chop),
        (REG_GLOBAL_SCALER, globalscaler & 0xFF),
    ]

    configurator = configurator_factory(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset.default(),
    )


def _build_chopconf_set_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Ajusta campos especÃƒÂ­ficos de CHOPCONF preservando os demais (mÃƒÂ¡scara/merge)."
        ),
    )
    _add_common_spi_arguments(parser)
    parser.add_argument("--toff", type=int, default=None, help="TOFF (0-15)")
    parser.add_argument("--tbl", type=int, default=None, help="TBL (0-3)")
    parser.add_argument("--hstrt", type=int, default=None, help="HSTRT (0-7)")
    parser.add_argument(
        "--verify",
        action="store_true",
        help="LÃƒÂª CHOPCONF apÃƒÂ³s a escrita para confirmar",
    )
    return parser


def _run_chopconf_set(
    argv: Sequence[str],
    *,
    configurator_factory,
    device_finder,
) -> int:
    parser = _build_chopconf_set_parser()
    args = parser.parse_args(list(argv))

    toff = args.toff
    tbl = args.tbl
    hstrt = args.hstrt
    if toff is None and tbl is None and hstrt is None:
        print("Nada a fazer: informe ao menos --toff, --tbl ou --hstrt")
        return 1
    if toff is not None and not (0 <= int(toff) <= 15):
        print("--toff fora da faixa (0-15)")
        return 1
    if tbl is not None and not (0 <= int(tbl) <= 3):
        print("--tbl fora da faixa (0-3)")
        return 1
    if hstrt is not None and not (0 <= int(hstrt) <= 7):
        print("--hstrt fora da faixa (0-7)")
        return 1

    configurator = configurator_factory(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset(writes=tuple()),
    )

    with configurator as driver:  # type: ignore[assignment]
        before = driver.read_register(REG_CHOPCONF)
        old = before.value
        new = int(old)
        if toff is not None:
            new = (new & ~0xF) | (int(toff) & 0xF)
        if tbl is not None:
            new = (new & ~(0x3 << 15)) | ((int(tbl) & 0x3) << 15)
        if hstrt is not None:
            new = (new & ~(0x7 << 4)) | ((int(hstrt) & 0x7) << 4)

        print(
            (
                "CHOPCONF antes: 0x{old:08X}  Ã¢â€ â€™  aplicando mÃƒÂ¡scara: "
                "TOFF={toff}, TBL={tbl}, HSTRT={hstrt}"
            ).format(
                old=old,
                toff=(toff if toff is not None else 'Ã¢â‚¬â€'),
                tbl=(tbl if tbl is not None else 'Ã¢â‚¬â€'),
                hstrt=(hstrt if hstrt is not None else 'Ã¢â‚¬â€'),
            )
        )
        resp = driver.write_register(REG_CHOPCONF, new)
        print(_format_response(resp))

        if args.verify:
            after = driver.read_register(REG_CHOPCONF)
            print(_format_read_result(after))

    return 0

    def _operation(driver):
        print(
            f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para aplicar preset ultra-frio"
        )
        responses: List[TMC5160TransferResult] = []
        # Base preset first for predictable state
        responses.extend(driver.configure())
        if args.clear_gstat:
            responses.append(driver.write_register(REG_GSTAT, 0x00000007))
        print("Aplicando ajustes ultra-frios:")
        for address, value in writes:
            print(f" - 0x{address:02X} = 0x{value:08X}")
        responses.extend(driver.apply_registers(writes))

        print("Respostas do TMC5160:")
        for res in responses:
            print(_format_response(res))
            res.raise_on_faults()

        # Verify readable regs
        verify_regs = [
            REG_GSTAT,
            0x6F,
            REG_GCONF,
            REG_CHOPCONF,
        ]
        print("Leituras de verificaÃƒÂ§ÃƒÂ£o:")
        results = driver.read_registers(verify_regs)
        for r in results:
            print(_format_read_result(r))
            r.raise_on_faults()

    return _run_spi_operation(
        args,
        configurator,
        device_finder,
        success_message="Preset ultra-frio aplicado.",
        operation=_operation,
    )


def _format_status_flags(byte: int) -> str:
    names = [
        (7, "status_stop_r"),
        (6, "status_stop_l"),
        (5, "position_reached"),
        (4, "velocity_reached"),
        (3, "standstill"),
        (2, "stallguard"),
        (1, "driver_error"),
        (0, "reset_flag"),
    ]
    active = [name for bit, name in names if (byte & (1 << bit))]
    return ", ".join(active) if active else "nenhuma"


def _classify_drv_gstat(gstat_val: int, drv_val: int) -> str:
    # DRV_STATUS dominant failures
    if drv_val & (1 << 26):
        return "Sobretemperatura (OT): reduza IRUN e melhore a dissipaÃƒÂ§ÃƒÂ£o"
    if drv_val & (1 << 25):
        return "PrÃƒÂ©Ã¢â‚¬â€˜aviso de sobretemperatura (OTPW): monitore temperatura/corrente"
    if drv_val & (1 << 24):
        return "Curto ÃƒÂ  terra na fase A (S2GA): verifique fiaÃƒÂ§ÃƒÂ£o/mÃƒÂ³dulo"
    if drv_val & (1 << 23):
        return "Curto ÃƒÂ  terra na fase B (S2GB): verifique fiaÃƒÂ§ÃƒÂ£o/mÃƒÂ³dulo"
    if drv_val & (1 << 22):
        return "Curto ÃƒÂ  alimentaÃƒÂ§ÃƒÂ£o na fase A (S2VSA): verifique fiaÃƒÂ§ÃƒÂ£o/mÃƒÂ³dulo"
    if drv_val & (1 << 21):
        return "Curto ÃƒÂ  alimentaÃƒÂ§ÃƒÂ£o na fase B (S2VSB): verifique fiaÃƒÂ§ÃƒÂ£o/mÃƒÂ³dulo"
    if drv_val & (1 << 20):
        return "OpenÃ¢â‚¬â€˜load na fase A (OLA): motor/cabo desconectado ou intermitente"
    if drv_val & (1 << 19):
        return "OpenÃ¢â‚¬â€˜load na fase B (OLB): motor/cabo desconectado ou intermitente"

    # GSTAT
    if gstat_val & (1 << 2):
        return "UV_CP (subtensÃƒÂ£o do chargeÃ¢â‚¬â€˜pump): confira VM/ENN/partida"
    if gstat_val & (1 << 1):
        return "DRV_ERR latched: ver DRV_STATUS para a causa"
    if gstat_val & (1 << 0):
        return "RESET recente: limpe GSTAT ou ignore se jÃƒÂ¡ limpo"

    # Nothing found in regs
    if drv_val & (1 << 31):
        return "Standstill (STST) sem falhas: provÃƒÂ¡vel 'driver_error' residual do status SPI"
    return "Sem falhas em GSTAT/DRV_STATUS (driver_error apenas no byte de status SPI)"


def _bin8(v: int) -> str:
    return f"0b{v & 0xFF:08b}"


def _bin32(v: int) -> str:
    return f"0b{v & 0xFFFFFFFF:032b}"


def _bin_with_bracket(value: int, total_bits: int, bit_start: int, bit_end: int | None = None) -> str:
    """Renderiza o valor em binÃƒÂ¡rio e destaca [entre colchetes] o(s) bit(s) de interesse.

    Os bits sÃƒÂ£o indexados MSB=total_bits-1 ... LSB=0, igual aos mapeamentos do datasheet.
    """
    if bit_end is None:
        bit_end = bit_start
    bit_start = int(bit_start)
    bit_end = int(bit_end)
    if not (0 <= bit_start < total_bits and 0 <= bit_end < total_bits):
        return _bin32(value) if total_bits == 32 else _bin8(value)
    # MSB primeiro
    bits = f"{value & ((1<<total_bits)-1):0{total_bits}b}"
    # Converter ÃƒÂ­ndice de bit (MSB=total_bits-1) para ÃƒÂ­ndice de string (0..total_bits-1)
    i0 = total_bits - 1 - bit_start
    i1 = total_bits - 1 - bit_end
    i_min, i_max = min(i0, i1), max(i0, i1)
    left = bits[:i_min]
    mid = bits[i_min:i_max+1]
    right = bits[i_max+1:]
    # Inserir espaÃƒÂ§o antes e depois dos colchetes para evidenciar a janela
    return "0b" + left + (" " if left else "") + "[" + mid + "]" + (" " if right else "") + right


def _format_status_bitlist(byte: int) -> str:
    labels = {
        7: ("status_stop_r", "fim de curso R"),
        6: ("status_stop_l", "fim de curso L"),
        5: ("position_reached", "posiÃƒÂ§ÃƒÂ£o atingida"),
        4: ("velocity_reached", "velocidade atingida"),
        3: ("standstill", "parado"),
        2: ("stallguard", "stallguard"),
        1: ("driver_error", "erro de driver"),
        0: ("reset_flag", "reset detectado"),
    }
    lines = []
    for bit in range(7, -1, -1):
        if (byte >> bit) & 1:
            name, desc = labels.get(bit, (f"bit{bit}", ""))
            lines.append(f"  {_bin_with_bracket(byte, 8, bit)} [{name}] {desc}")
    if not lines:
        lines.append("  (nenhuma flag ativa)")
    return "\n".join(lines)


def _format_drvstatus_bitlist(value: int) -> str:
    mapping = [
        (31, "STST", "standstill"),
        (26, "OT", "sobretemperatura"),
        (25, "OTPW", "prÃƒÂ©Ã¢â‚¬â€˜aviso sobretemp"),
        (24, "S2GA", "curto ÃƒÂ  terra fase A"),
        (23, "S2GB", "curto ÃƒÂ  terra fase B"),
        (22, "S2VSA", "curto ÃƒÂ  alimentaÃƒÂ§ÃƒÂ£o A"),
        (21, "S2VSB", "curto ÃƒÂ  alimentaÃƒÂ§ÃƒÂ£o B"),
        (20, "OLA", "openÃ¢â‚¬â€˜load fase A"),
        (19, "OLB", "openÃ¢â‚¬â€˜load fase B"),
    ]
    lines = []
    for bit, name, desc in mapping:
        if (value >> bit) & 1:
            lines.append(f"  {_bin_with_bracket(value, 32, bit)} [{name}] {desc}")
    if not lines:
        lines.append("  (nenhum bit de falha relevante em DRV_STATUS)")
    return "\n".join(lines)


def _format_gstat_bitlist(value: int) -> str:
    mapping = [
        (2, "UV_CP", "subtensÃƒÂ£o chargeÃ¢â‚¬â€˜pump"),
        (1, "DRV_ERR", "falha latched"),
        (0, "RESET", "reset detectado"),
    ]
    lines = []
    for bit, name, desc in mapping:
        if (value >> bit) & 1:
            lines.append(f"  {_bin_with_bracket(value, 32, bit)} [{name}] {desc}")
    if not lines:
        lines.append("  (nenhum bit ativo em GSTAT)")
    return "\n".join(lines)


def _run_status_compact(
    argv: Sequence[str],
    *,
    configurator_factory,
    device_finder,
) -> int:
    parser = _build_status_compact_parser()
    args = parser.parse_args(list(argv))

    # Resolve dev list
    if args.dev is not None:
        devs = [int(args.dev)]
    elif args.devs:
        try:
            devs = [int(x.strip()) for x in str(args.devs).split(",") if x.strip()]
        except Exception:
            print("Erro: formato invÃƒÂ¡lido em --devs (use ex.: 1,2,3)")
            return 1
        if not devs:
            devs = [1, 2, 3]
    else:
        devs = [1, 2, 3]

    print(f"Status compacto (bus={args.bus}, devs={devs})")

    any_error = False

    for dev in devs:
        configurator = configurator_factory(
            bus=args.bus,
            device=dev,
            speed_hz=args.speed,
            register_preset=TMC5160RegisterPreset(writes=tuple()),
        )

        try:
            with configurator as driver:  # type: ignore[assignment]
                # Opcional: limpar GSTAT antes de ler status
                if args.clear_gstat:
                    gstat_before = driver.read_register(REG_GSTAT)
                    cleared = driver.write_register(REG_GSTAT, 0x00000007)
                    gstat_after = driver.read_register(REG_GSTAT)
                    print(
                        (
                            "- CS {dev}: clear-gstat before=0x{b:08X} resp={rb} | write={wc} | after=0x{a:08X} resp={ra}"
                        ).format(
                            dev=dev,
                            b=gstat_before.value,
                            rb=gstat_before.reply.raw_hex,
                            wc=cleared.raw_hex,
                            a=gstat_after.value,
                            ra=gstat_after.reply.raw_hex,
                        )
                    )

                # Fazer uma leitura simples para obter bytes de status e frame
                rr = driver.read_register(REG_GCONF)
                s_req = rr.request.status.raw
                s_rep = rr.reply.status.raw
                req_bytes = rr.request.response
                rep_bytes = rr.reply.response
                req_bin = " ".join(_bin8(b) for b in req_bytes)
                rep_bin = " ".join(_bin8(b) for b in rep_bytes)
                print(f"- CS {dev}:")
                # Frames juntos: resp imediatamente abaixo do req
                print(f"  frame_req : {req_bin}")
                print(f"  frame_resp: {rep_bin}")

                # Se driver_error aparecer no byte de status, coletar diagnÃƒÂ³stico
                # e marcar ERRO somente quando houver indÃƒÂ­cios reais conforme o MD:
                # GSTAT: DRV_ERR/UV_CP; DRV_STATUS: OT/OTPW/S2G*/S2VS*/OL*
                confirmed_error = False
                if (s_req & 0x02) or (s_rep & 0x02):
                    g = driver.read_register(REG_GSTAT)
                    d = driver.read_register(0x6F)
                    g_dec = decode_register_value(REG_GSTAT, g.value)
                    d_dec = decode_register_value(0x6F, d.value)
                    g_has_err = bool(g.value & ((1 << 2) | (1 << 1)))  # UV_CP / DRV_ERR
                    d_has_err = bool(d.value & ((1 << 26) | (1 << 25) | (1 << 24) | (1 << 23) |
                                               (1 << 22) | (1 << 21) | (1 << 20) | (1 << 19)))
                    confirmed_error = g_has_err or d_has_err
                    if confirmed_error:
                        print("  ERRO: " + _classify_drv_gstat(g.value, d.value))
                # Em seguida, mostra o status_req e flags
                print(f"  status_req: {_bin8(s_req)} Ã¢â€ â€™ [{_format_status_flags(s_req)}]")
                print(_format_status_bitlist(s_req))
                # Agora mostra status final (reply) e flags somente se diferente
                if (s_rep != s_req) or (rep_bytes != req_bytes):
                    print(f"  status    : {_bin8(s_rep)} Ã¢â€ â€™ [{_format_status_flags(s_rep)}]")
                    print(_format_status_bitlist(s_rep))
                if (s_req & 0x02) or (s_rep & 0x02):
                    print(f"  Diag GSTAT:")
                    print(f"    valor: {_bin32(g.value)} (0x{g.value:08X})")
                    print(_format_gstat_bitlist(g.value))
                    print(f"    resp : {' '.join(_bin8(b) for b in g.request.response)} -> {g.reply.raw_hex}")
                    print(f"  Diag DRV_STATUS:")
                    print(f"    valor: {_bin32(d.value)} (0x{d.value:08X})")
                    print(_format_drvstatus_bitlist(d.value))
                    print(f"    resp : {' '.join(_bin8(b) for b in d.request.response)} -> {d.reply.raw_hex}")
                # Acumula no resumo apenas erros confirmados
                if confirmed_error:
                    any_error = True
        except FileNotFoundError:
            print(f"- CS {dev}: dispositivo /dev/spidev{args.bus}.{dev} nÃƒÂ£o encontrado")
            continue
        except PermissionError:
            print(f"- CS {dev}: permissÃƒÂ£o negada em /dev/spidev{args.bus}.{dev}")
            continue
        except RuntimeError as exc:
            # NÃƒÂ£o abortar a varredura dos demais
            print(f"- CS {dev}: erro durante leitura: {exc}")
            continue

    if any_error:
        print("Resumo: erro confirmado em pelo menos um driver (ver itens com 'ERRO' e 'Diag').")
    else:
        print("Resumo: nenhum erro confirmado por GSTAT/DRV_STATUS.")
    return 0
def run(
    argv: Optional[Sequence[str]] = None,
    *,
    configurator_factory=TMC5160Configurator,
    device_finder=lambda: sorted(Path("/dev").glob("spidev*")),
) -> int:
    if argv is None:
        argv_list: List[str] = sys.argv[1:]
    else:
        argv_list = list(argv)

    # Subcomandos
    if argv_list and argv_list[0] == "help":
        print(
            "Uso:\n"
            "  configure [opÃƒÂ§ÃƒÂµes]         Configura registradores\n"
            "  status [opÃƒÂ§ÃƒÂµes]            LÃƒÂª registradores (suporta --clear-gstat)\n"
            "  loop-test [opÃƒÂ§ÃƒÂµes]         Gera padrÃƒÂ£o de escrita contÃƒÂ­nuo\n"
            "  init-stepdir [opÃƒÂ§ÃƒÂµes]      Aplica preset para STEP/DIR externo\n"
            "  safe-off [opÃƒÂ§ÃƒÂµes]          Zera correntes, TOFF=0 e FREEWHEEL\n"
            "  preset-ultrafrio [opÃƒÂ§ÃƒÂµes]  Aplica perfil ultra-frio (SpreadCycle/1\x2f16/TOFF=5/HEND=2)\n"
            "  motion-params [opÃƒÂ§ÃƒÂµes]     LÃƒÂª parÃƒÂ¢metros que definem a movimentaÃƒÂ§ÃƒÂ£o (modo, Ã‚Âµstep, chopper, correntes)\n"
            "  status-compact [opÃƒÂ§ÃƒÂµes]    Status resumido 1 ou 1,2,3 (bus 0) com diag se driver_error\n"
        )
        return 0

    if argv_list and argv_list[0] == "status":
        return _run_status(
            argv_list[1:],
            configurator_factory=configurator_factory,
            device_finder=device_finder,
        )

    if argv_list and argv_list[0] == "loop-test":
        return _run_loop_test(
            argv_list[1:],
            configurator_factory=configurator_factory,
            device_finder=device_finder,
        )

    if argv_list and argv_list[0] == "init-stepdir":
        if __package__:
            from .tmc5160_stepdir import run_init_stepdir as _run_init_stepdir  # type: ignore
        else:
            from tmc5160_stepdir import run_init_stepdir as _run_init_stepdir  # type: ignore
        return _run_init_stepdir(
            argv_list[1:],
            configurator_factory=configurator_factory,
            device_finder=device_finder,
        )

    if argv_list and argv_list[0] == "safe-off":
        return _run_safe_off(
            argv_list[1:],
            configurator_factory=configurator_factory,
            device_finder=device_finder,
        )

    if argv_list and argv_list[0] == "preset-ultrafrio":
        return _run_preset_ultrafrio(
            argv_list[1:],
            configurator_factory=configurator_factory,
            device_finder=device_finder,
        )

    if argv_list and argv_list[0] == "motion-params":
        return _run_motion_params(
            argv_list[1:],
            configurator_factory=configurator_factory,
            device_finder=device_finder,
        )

    if argv_list and argv_list[0] == "chopconf-set":
        return _run_chopconf_set(
            argv_list[1:],
            configurator_factory=configurator_factory,
            device_finder=device_finder,
        )

    if argv_list and argv_list[0] == "status-compact":
        return _run_status_compact(
            argv_list[1:],
            configurator_factory=configurator_factory,
            device_finder=device_finder,
        )

    # Fallback: se o usuÃƒÂ¡rio passou flags de CHOPCONF sem subcomando, trata como chopconf-set
    if argv_list and any(arg.startswith("--toff") or arg.startswith("--tbl") or arg.startswith("--hstrt") for arg in argv_list):
        return _run_chopconf_set(
            argv_list,
            configurator_factory=configurator_factory,
            device_finder=device_finder,
        )

    # Compat: permitir chamar "configure" explicitamente
    if argv_list and argv_list[0] == "configure":
        argv_list = argv_list[1:]

    # PadrÃƒÂ£o: configurar
    return _run_configure(
        argv_list,
        configurator_factory=configurator_factory,
        device_finder=device_finder,
    )


def main() -> int:  # pragma: no cover - camada fina de execuÃƒÂ§ÃƒÂ£o
    try:
        return run()
    except CLIError as exc:
        print(f"Erro: {exc}")
        return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
