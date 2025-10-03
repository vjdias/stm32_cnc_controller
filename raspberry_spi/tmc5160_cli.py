"""Interface de linha de comando para configurar o driver TMC5160."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
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
}

REGISTER_NAMES = {
    REG_GSTAT: "GSTAT",
    REG_GCONF: "GCONF",
    REG_IHOLD_IRUN: "IHOLD_IRUN",
    REG_TPOWERDOWN: "TPOWERDOWN",
    REG_TPWMTHRS: "TPWMTHRS",
    REG_CHOPCONF: "CHOPCONF",
    REG_PWMCONF: "PWMCONF",
}

DEFAULT_STATUS_REGISTERS = (
    REG_GSTAT,
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
    stall_text = (
        "StallGuard detectado (SG=1)."
        if status.stallguard
        else "StallGuard inativo (SG=0)."
    )
    faults = status.active_faults()
    if faults:
        faults_text = "Alertas: {}.".format(", ".join(faults))
    else:
        faults_text = (
            "Alertas: nenhum (OT/OTPW/S2GA/S2GB/S2VSA/S2VSB/UV_CP limpos)."
        )

    lines = [
        f"- 0x{result.address:02X} <= 0x{result.value:08X}",
        f"  Resposta bruta: {result.raw_hex}",
        f"  Status 0x{status.raw:02X}: {stall_text} {faults_text}",
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
        lines.append(
            "  Diagnóstico : {} / {}".format(
                status_request.summary(), status_reply.summary()
            )
        )
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
        "--register",
        action="append",
        default=[],
        metavar="REG",
        help=(
            "Lista de registradores a serem lidos (pode ser repetido). "
            "Aceita aliases conhecidos (gconf, gstat, ...) ou endereços numéricos."
        ),
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
        available_hint = "Dispositivos disponíveis: {}. ".format(
            ", ".join(sorted(available))
        )
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
            "Erro: dispositivo SPI '{spi}' não encontrado. {available}" "Habilite o overlay SPI "
            "correspondente (ex.: {overlay}) ou ajuste --bus/--dev."
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
            operation(driver)
    except FileNotFoundError:
        return _report_missing_device(args, spi_node, device_finder)
    except PermissionError:
        return _report_permission_denied(spi_node)
    except RuntimeError as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        return 4

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
        print(
            f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para configurar o TMC5160"
        )
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
            for result in responses:
                print(_format_response(result))
                result.raise_on_faults()

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

    register_list = ", ".join(
        REGISTER_NAMES.get(addr, f"0x{addr:02X}") for addr in registers
    )

    def _operation(driver):
        print(
            f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para consultar o TMC5160"
        )
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

    if argv_list and argv_list[0] == "status":
        return _run_status(
            argv_list[1:],
            configurator_factory=configurator_factory,
            device_finder=device_finder,
        )

    if argv_list and argv_list[0] == "configure":
        argv_list = argv_list[1:]

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
