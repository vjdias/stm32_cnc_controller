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
        TMC5160RegisterPreset,
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
        TMC5160RegisterPreset,
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


class CLIError(Exception):
    """Erro de validação dos parâmetros da CLI."""


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


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Configura o driver TMC5160 através do barramento SPI do Raspberry Pi",
    )
    parser.add_argument("--bus", type=int, default=1, help="Barramento SPI (default: 1)")
    parser.add_argument("--dev", type=int, default=0, help="Dispositivo SPI (default: 0)")
    parser.add_argument(
        "--speed",
        type=int,
        default=4_000_000,
        help="Velocidade SPI em Hz (default: 4_000_000)",
    )
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


def run(
    argv: Optional[Sequence[str]] = None,
    *,
    configurator_factory=TMC5160Configurator,
) -> int:
    parser = _build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

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

    spi_node = f"/dev/spidev{args.bus}.{args.dev}"

    try:
        with configurator as driver:  # type: ignore[assignment]
            print(
                f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para configurar o TMC5160"
            )
            if not args.no_defaults:
                count = len(preset.writes)
                print(f"Aplicando preset padrão ({count} registradores)")
                driver.configure()
            if overrides:
                print("Aplicando ajustes adicionais:")
                for address, value in overrides:
                    print(f" - 0x{address:02X} = 0x{value:08X}")
                driver.apply_registers(overrides)
            else:
                print("Nenhum ajuste adicional informado; mantendo preset padrão.")
    except FileNotFoundError:
        print(
            "Erro: dispositivo SPI '{spi}' não encontrado. Habilite o overlay SPI correspondente "
            "(ex.: dtparam=spi=on ou dtoverlay=spi1-3cs) ou ajuste --bus/--dev.".format(
                spi=spi_node
            ),
            file=sys.stderr,
        )
        return 2
    except PermissionError:
        print(
            "Erro: permissão negada ao acessar '{spi}'. Execute o comando com sudo ou ajuste as "
            "regras de acesso (ex.: grupo 'spi').".format(spi=spi_node),
            file=sys.stderr,
        )
        return 3
    except RuntimeError as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        return 4

    print("Configuração concluída.")
    return 0


def main() -> int:  # pragma: no cover - camada fina de execução
    try:
        return run()
    except CLIError as exc:
        print(f"Erro: {exc}")
        return 1


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
