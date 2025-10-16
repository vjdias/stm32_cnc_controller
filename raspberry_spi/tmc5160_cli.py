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
        REG_PWMCONF,
        REG_TPOWERDOWN,
        REG_TPWMTHRS,
        TMC5160Configurator,
        TMC5160ReadResult,
        TMC5160RegisterPreset,
        TMC5160TransferResult,
        decode_register_value,
    )
else:  # execução direta dos testes/script
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from tmc5160 import (  # type: ignore
        REG_CHOPCONF,
        REG_GCONF,
        REG_GSTAT,
        REG_IHOLD_IRUN,
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
}

DEFAULT_STATUS_REGISTERS = (
    REG_GSTAT,
    0x6F,
    REG_GCONF,
    REG_IHOLD_IRUN,
    REG_TPOWERDOWN,
    REG_TPWMTHRS,
    REG_CHOPCONF,
    REG_PWMCONF,
)


class CLIError(Exception):
    """Erro de validação dos parâmetros da CLI."""


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
        f"  Requisição  : status=0x{status_request.raw:02X}, resposta={result.request.raw_hex}",
        f"  Resposta útil: status=0x{status_reply.raw:02X}, resposta={result.reply.raw_hex}",
    ]
    if status_request.raw == status_reply.raw:
        lines.append(f"  Diagnóstico : {status_reply.summary()}")
    else:
        lines.append("  Diagnóstico : {} / {}".format(status_request.summary(), status_reply.summary()))
    decoded = decode_register_value(result.address, result.value)
    if decoded:
        lines.append("  Tradução    :")
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
        raise CLIError("Informe o nome ou endereço do registrador antes do '='")
    if not value_text:
        raise CLIError("Informe o valor do registrador após o '='")

    if name in REGISTER_ALIASES:
        address = REGISTER_ALIASES[name]
    else:
        try:
            address = int(name, 0)
        except ValueError as exc:  # pragma: no cover - defensive
            raise CLIError(
                f"Registrador desconhecido '{name}'. Informe um dos aliases conhecidos "
                "ou um endereço numérico."
            ) from exc

    try:
        value = int(value_text, 0)
    except ValueError as exc:  # pragma: no cover - defensive
        raise CLIError(f"Valor inválido '{value_text}'. Utilize decimal ou hexadecimal.") from exc

    if not 0 <= address <= 0x7F:
        raise CLIError("Endereço de registrador deve estar entre 0x00 e 0x7F")
    if not 0 <= value <= 0xFFFFFFFF:
        raise CLIError("Valor do registrador deve caber em 32 bits")

    return address, value


def _parse_register_name(raw: str) -> int:
    key = raw.strip().lower()
    if not key:
        raise CLIError("Informe o nome ou endereço do registrador a consultar")
    if key in REGISTER_ALIASES:
        return REGISTER_ALIASES[key]
    try:
        address = int(key, 0)
    except ValueError as exc:  # pragma: no cover - defensivo
        raise CLIError(
            f"Registrador desconhecido '{raw}'. Utilize um alias conhecido ou um endereço numérico."
        ) from exc
    if not 0 <= address <= 0x7F:
        raise CLIError("Endereço de registrador deve estar entre 0x00 e 0x7F")
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
        description="Configura o driver TMC5160 através do barramento SPI do Raspberry Pi",
    )
    _add_common_spi_arguments(parser)
    parser.add_argument(
        "--no-defaults",
        action="store_true",
        help="Não enviar o preset padrão antes dos ajustes informados",
    )
    parser.add_argument(
        "--write",
        action="append",
        default=[],
        metavar="REG=VAL",
        help=(
            "Sequência adicional de escritas de registrador (pode ser repetido). "
            "Aceita aliases (gconf, chopconf, ...) ou endereços numéricos."
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
    )
    return parser


def _build_status_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Consulta os registradores do TMC5160 sem alterar a configuração",
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
            "Aceita aliases conhecidos (gconf, gstat, drv_status, ...) ou endereços numéricos."
        ),
    )
    return parser


def _build_loop_test_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Gera um padrão repetitivo de escrita SPI para observar o barramento "
            "com instrumentos como osciloscópio ou analisador lógico."
        ),
    )
    _add_common_spi_arguments(parser)
    parser.add_argument(
        "--address",
        type=_parse_register_name,
        default=REG_GCONF,
        help=("Registrador a ser escrito (alias conhecido ou endereço numérico, default: gconf)."),
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
            "Utilize 0 para repetir o mais rápido possível."
        ),
    )
    parser.add_argument(
        "--iterations",
        type=int,
        default=0,
        help=(
            "Número de repetições antes de encerrar automaticamente (0 = infinito, "
            "pressione Ctrl+C para parar)."
        ),
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Não imprimir cada resposta, apenas o resumo inicial e final.",
    )
    return parser


def _build_init_stepdir_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=("Aplica configurações básicas para operar o TMC5160 em STEP/DIR externo."),
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
        help="Resolução de microstepping para pulsos STEP externos",
    )
    parser.add_argument(
        "--interpolate",
        dest="interpolate",
        action="store_true",
        default=True,
        help="Ativa interpolação (INTPOL) para 256 micropassos",
    )
    parser.add_argument(
        "--no-interpolate",
        dest="interpolate",
        action="store_false",
        help="Desativa interpolação (INTPOL=0)",
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
        help="TPWMTHRS (limiar de StealthChop↔SpreadCycle)",
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
        available_hint = "Dispositivos disponíveis: {}. ".format(", ".join(sorted(available)))
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
            "Erro: dispositivo SPI '{spi}' não encontrado. {available}"
            "Habilite o overlay SPI correspondente (ex.: {overlay}) ou ajuste --bus/--dev."
        ).format(spi=spi_node, available=available_hint, overlay=overlay_hint),
        file=sys.stderr,
    )
    return 2


def _report_permission_denied(spi_node: str) -> int:
    print(
        "Erro: permissão negada ao acessar '{spi}'. Execute o comando com sudo ou ajuste as "
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
            print(f"Aplicando preset padrão ({count} registradores)")
            responses.extend(driver.configure())
        if overrides:
            print("Aplicando ajustes adicionais:")
            for address, value in overrides:
                print(f" - 0x{address:02X} = 0x{value:08X}")
            responses.extend(driver.apply_registers(overrides))
        else:
            print("Nenhum ajuste adicional informado; mantendo preset padrão.")

        if responses:
            print("Respostas do TMC5160:")
            try:
                for result in responses:
                    print(_format_response(result))
                    result.raise_on_faults()
            except RuntimeError as exc:
                # Diagnóstico detalhado com base em GSTAT e DRV_STATUS
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
                    raise RuntimeError(str(exc) + " | Diagnóstico: " + " | ".join(details)) from exc
                except Exception:
                    raise

    return _run_spi_operation(
        args,
        configurator,
        device_finder,
        success_message="Configuração concluída.",
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
            print("Executando sequência clear-GSTAT:")
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
            print("Nenhum registrador informado; nada será lido.")
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
        success_message="Consulta concluída.",
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
        raise CLIError("O número de repetições deve ser zero (infinito) ou positivo")

    configurator = configurator_factory(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset.default(),
    )

    register_name = REGISTER_NAMES.get(args.address, f"0x{args.address:02X}")

    def _operation(driver):
        print(
            "Abrindo SPI bus={bus} dev={dev} a {speed} Hz para gerar padrão de teste".format(
                bus=args.bus, dev=args.dev, speed=args.speed
            )
        )
        if args.iterations:
            print(f"Repetições solicitadas: {args.iterations}")
        else:
            print("Repetições solicitadas: infinito (interrompa com Ctrl+C)")
        if args.interval > 0:
            print(f"Intervalo entre mensagens: {args.interval:.6f} s")
        else:
            print("Intervalo entre mensagens: sem pausa (taxa máxima permitida pela SPI)")

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
            print(f"Loop interrompido pelo usuário após {count} iterações.")
            return False

        print(f"Loop concluído após {count} iterações.")
        return True

    return _run_spi_operation(
        args,
        configurator,
        device_finder,
        success_message="Padrão de teste finalizado.",
        operation=_operation,
    )


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
            "  configure [opções]         Configura registradores\n"
            "  status [opções]            Lê registradores (suporta --clear-gstat)\n"
            "  loop-test [opções]         Gera padrão de escrita contínuo\n"
            "  init-stepdir [opções]      Aplica preset para STEP/DIR externo\n"
            "  safe-off [opções]          Zera correntes, TOFF=0 e FREEWHEEL\n"
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

    # Compat: permitir chamar "configure" explicitamente
    if argv_list and argv_list[0] == "configure":
        argv_list = argv_list[1:]

    # Padrão: configurar
    return _run_configure(
        argv_list,
        configurator_factory=configurator_factory,
        device_finder=device_finder,
    )


def main() -> int:  # pragma: no cover - camada fina de execução
    try:
        return run()
    except CLIError as exc:
        print(f"Erro: {exc}")
        return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
