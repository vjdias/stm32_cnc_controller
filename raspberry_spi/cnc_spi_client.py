#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Cliente SPI para comunicação com o firmware CNC no STM32."""

import argparse
import sys
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import List, Optional

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_client import CNCClient
    from .cnc_commands import CNCCommandExecutor
else:  # execução direta do script a partir do diretório raspberry_spi
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_client import CNCClient
    from cnc_commands import CNCCommandExecutor


def _parse_byte(value: str) -> int:
    try:
        byte_val = int(value, 0)
    except ValueError as exc:  # pragma: no cover - defensive path
        raise argparse.ArgumentTypeError(
            f"Valor de byte inválido: '{value}'"
        ) from exc
    if not 0 <= byte_val <= 0xFF:
        raise argparse.ArgumentTypeError(
            "Informe um byte entre 0 e 255 (ex.: 0x3C)"
        )
    return byte_val


def _common_args(
    p: argparse.ArgumentParser,
    *,
    include_tries: bool = False,
    default_tries: int = 5,
    include_settle_delay: bool = False,
    default_settle_delay: float = 0.001,
) -> None:
    p.add_argument("--bus", type=int, default=0)
    p.add_argument("--dev", type=int, default=0)
    p.add_argument("--speed", type=int, default=1_000_000)
    p.add_argument(
        "--spi-log-format",
        choices=("hex", "bin"),
        default="hex",
        help=(
            "Formato usado para imprimir as trocas SPI (hex ou bin). "
            "Padrão: %(default)s"
        ),
    )
    p.add_argument(
        "--poll-byte",
        type=_parse_byte,
        default=None,
        help=(
            "Byte utilizado pelo polling do cliente. Utilize 0x3C para manter "
            "o comportamento atual ou informe outro valor."
        ),
    )
    p.add_argument(
        "--disable-poll",
        action="store_true",
        help="Não enviar polling após o handshake inicial (usa apenas o handshake).",
    )
    if include_tries:
        p.add_argument(
            "--tries",
            type=int,
            default=default_tries,
            help=(
                "Número máximo de leituras para validar a resposta"
                " (padrão: %(default)s)"
            ),
        )
    if include_settle_delay:
        p.add_argument(
            "--settle-delay",
            type=float,
            default=default_settle_delay,
            help="Tempo (s) para aguardar entre tentativas de leitura",
        )


def _parse_led_frequency(raw_value: str) -> int:
    try:
        value = Decimal(raw_value)
    except InvalidOperation as exc:  # pragma: no cover - defensive path
        raise argparse.ArgumentTypeError(
            f"Frequência inválida: '{raw_value}'"
        ) from exc

    if value < 0:
        raise argparse.ArgumentTypeError("A frequência deve ser não negativa")

    quantized = value.quantize(Decimal("0.01"), rounding=ROUND_HALF_UP)
    centi_hz = quantized * 100
    if centi_hz > Decimal(0xFFFF):
        raise argparse.ArgumentTypeError(
            "Frequência máxima suportada é 655.35 Hz"
        )

    return int(centi_hz)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Cliente SPI (Raspberry) para CNC_Controller (STM32 SPI1 Slave)",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    led_ctrl = sub.add_parser(
        "led-control",
        aliases=["led-ctrl"],
        help="Controle do LED discreto (LED1)",
    )
    _common_args(led_ctrl, include_tries=True, include_settle_delay=True)
    led_ctrl.add_argument("--frame-id", type=int, required=True)
    led_ctrl.add_argument(
        "--mask",
        type=lambda x: int(x, 0),
        required=True,
        help="Bit0=LED1",
    )
    led_ctrl.add_argument(
        "--led1-mode", type=int, choices=[0, 1, 2], default=0,
        help="0=Off, 1=On, 2=Pisca",
    )
    led_ctrl.add_argument(
        "--led1-freq",
        type=_parse_led_frequency,
        default=0,
        help=(
            "Frequência de pisca do LED1 em Hz (modo=2). "
            "Aceita números com até duas casas decimais"
        ),
    )
    led_ctrl.set_defaults(handler="led_control", needs_client=True)

    q_add = sub.add_parser("queue-add", help="Adicionar movimento à fila")
    _common_args(q_add, include_tries=True, include_settle_delay=True)
    q_add.add_argument("--frame-id", type=int, required=True)
    q_add.add_argument("--dir", type=lambda x: int(x, 0), required=True)
    q_add.add_argument("--vx", type=int, required=True)
    q_add.add_argument("--sx", type=int, required=True)
    q_add.add_argument("--vy", type=int, required=True)
    q_add.add_argument("--sy", type=int, required=True)
    q_add.add_argument("--vz", type=int, required=True)
    q_add.add_argument("--sz", type=int, required=True)
    for axis in ("x", "y", "z"):
        q_add.add_argument(f"--kp-{axis}", type=int, default=0)
        q_add.add_argument(f"--ki-{axis}", type=int, default=0)
        q_add.add_argument(f"--kd-{axis}", type=int, default=0)
    q_add.add_argument(
        "--settle-delay",
        type=float,
        default=0.001,
        help="Tempo (s) para aguardar entre tentativas de leitura",
    )
    q_add.set_defaults(handler="queue_add", needs_client=True)

    q_status = sub.add_parser("queue-status", help="Consultar status da fila")
    _common_args(q_status, include_tries=True, include_settle_delay=True)
    q_status.add_argument("--frame-id", type=int, required=True)
    q_status.set_defaults(handler="queue_status", needs_client=True)

    start_move = sub.add_parser("start-move", help="Iniciar execução")
    _common_args(start_move, include_tries=True, include_settle_delay=True)
    start_move.add_argument("--frame-id", type=int, required=True)
    start_move.set_defaults(handler="start_move", needs_client=True)

    end_move = sub.add_parser("end-move", help="Finalizar execução")
    _common_args(end_move, include_tries=True, include_settle_delay=True)
    end_move.add_argument("--frame-id", type=int, required=True)
    end_move.set_defaults(handler="end_move", needs_client=True)

    home = sub.add_parser("home", help="Sequência de homing")
    _common_args(home, include_tries=True, include_settle_delay=True)
    home.add_argument("--frame-id", type=int, required=True)
    home.add_argument("--axes", type=lambda x: int(x, 0), required=True)
    home.add_argument("--dirs", type=lambda x: int(x, 0), required=True)
    home.add_argument("--vhome", type=lambda x: int(x, 0), required=True)
    home.set_defaults(handler="home", needs_client=True)

    probe = sub.add_parser("probe-level", help="Sequência de probe level")
    _common_args(probe, include_tries=True, include_settle_delay=True)
    probe.add_argument("--frame-id", type=int, required=True)
    probe.add_argument("--axes", type=lambda x: int(x, 0), required=True)
    probe.add_argument("--vprobe", type=lambda x: int(x, 0), required=True)
    probe.set_defaults(handler="probe_level", needs_client=True)

    hello = sub.add_parser(
        "hello",
        help="Enviar uma requisição 'hello' e aguardar a resposta do STM32",
    )
    _common_args(hello, include_tries=True, include_settle_delay=True)
    hello.set_defaults(handler="hello", needs_client=True)

    boot_hello = sub.add_parser(
        "boot-hello",
        help=(
            "Ler frame de teste 'hello' enfileirado automaticamente no boot "
            "(quando habilitado no firmware)"
        ),
    )
    _common_args(
        boot_hello,
        include_tries=True,
        default_tries=16,
        include_settle_delay=True,
        default_settle_delay=0.002,
    )
    boot_hello.add_argument("--chunk-len", type=int, default=7)
    boot_hello.set_defaults(handler="boot_hello", needs_client=True)

    led_boot = sub.add_parser(
        "led",
        help=(
            "Ler frame de teste 'led' do STM32 (enfileirado no boot quando "
            "habilitado)"
        ),
    )
    _common_args(
        led_boot,
        include_tries=True,
        default_tries=16,
        include_settle_delay=True,
        default_settle_delay=0.002,
    )
    led_boot.add_argument("--chunk-len", type=int, default=7)
    led_boot.set_defaults(handler="boot_led", needs_client=True)

    examples = sub.add_parser(
        "examples",
        help="Listar os comandos disponíveis acompanhados de exemplos de uso",
    )
    examples.set_defaults(handler=print_examples, needs_client=False)

    return parser


def print_examples(_: argparse.Namespace) -> None:
    base_cmd = "python3 cnc_spi_client.py"
    examples = [
        (
            "LED discreto",
            f"{base_cmd} led-control --frame-id 1 --mask 0x01 --led1-mode 2 --led1-freq 0.5",
        ),
        (
            "Adicionar movimento à fila",
            f"{base_cmd} queue-add --frame-id 2 --dir 0x03 --vx 1000 --sx 2000 --vy 1000 --sy 2000 --vz 500 --sz 800",
        ),
        ("Status da fila", f"{base_cmd} queue-status --frame-id 3"),
        ("Iniciar execução", f"{base_cmd} start-move --frame-id 4"),
        ("Finalizar execução", f"{base_cmd} end-move --frame-id 5"),
        (
            "Sequência de homing",
            f"{base_cmd} home --frame-id 6 --axes 0x03 --dirs 0x01 --vhome 0x1200",
        ),
        (
            "Probe level",
            f"{base_cmd} probe-level --frame-id 7 --axes 0x04 --vprobe 0x0100",
        ),
        ("Requisição 'hello'", f"{base_cmd} hello --tries 5"),
        (
            "Frame de boot 'hello' (requer firmware com boot-test habilitado)",
            f"{base_cmd} boot-hello --tries 10 --chunk-len 7",
        ),
        (
            "Frame de boot 'led' (requer firmware com boot-test habilitado)",
            f"{base_cmd} led --tries 10 --chunk-len 7",
        ),
    ]

    print("Comandos disponíveis e exemplos:")
    for title, command in examples:
        print(f"- {title}:\n  {command}")


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    handler = getattr(args, "handler", None)
    if handler is None:
        parser.error("Nenhum comando informado")

    needs_client = getattr(args, "needs_client", True)

    client: Optional[CNCClient] = None
    executor: Optional[CNCCommandExecutor] = None
    try:
        if needs_client:
            log_format = getattr(args, "spi_log_format", "hex")
            client = CNCClient(
                bus=args.bus,
                dev=args.dev,
                speed_hz=args.speed,
                log_format=log_format,
            )
            executor = CNCCommandExecutor(client)

        if isinstance(handler, str):
            if executor is None:
                handler_fn = globals().get(handler)
            else:
                handler_fn = getattr(executor, handler, None)
        else:
            handler_fn = handler

        if handler_fn is None:
            parser.error("Handler desconhecido")

        handler_fn(args)
    finally:
        if client is not None:
            client.close()

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
