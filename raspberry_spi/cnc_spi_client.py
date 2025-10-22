#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Cliente SPI para comunicação com o firmware CNC no STM32."""

import argparse
import os
import sys
import glob
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import List, Optional, Dict, Tuple

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
        option_name = "--settle-delay"

        def _find_action(parser: argparse.ArgumentParser) -> Optional[argparse.Action]:
            for container in (
                getattr(parser, "_actions", []),
                getattr(getattr(parser, "_optionals", None), "_actions", []),
            ):
                for action in container or []:
                    if option_name in getattr(action, "option_strings", ()):  # pragma: no cover - defensive
                        return action
            return None

        existing_action = getattr(p, "_settle_delay_action", None)
        if existing_action is None:
            existing_action = _find_action(p)

        if existing_action is None:
            existing_action = p.add_argument(
                option_name,
                type=float,
                default=default_settle_delay,
                help=(
                    "Tempo (s) para aguardar entre tentativas de leitura (ex.: 0.002 "
                    "para 2 ms)."
                ),
            )
        else:
            existing_action.default = default_settle_delay

        setattr(p, "_settle_delay_action", existing_action)
        p.set_defaults(settle_delay=default_settle_delay)


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
        description="Cliente SPI (Raspberry) para CNC_Controller (STM32 SPI2 Slave)",
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

    # Verificação de CS / SPI
    cs_check = sub.add_parser(
        "cs-check",
        help=(
            "Listar dispositivos SPI (spidevX.Y), conferir configurações e opcionalmente ler GPIOs de CS manual."
        ),
    )
    cs_check.add_argument(
        "--expected-mode",
        type=int,
        choices=[0, 1, 2, 3],
        default=3,
        help="Modo SPI esperado (0..3). Default: 3",
    )
    cs_check.add_argument(
        "--expected-bpw",
        type=int,
        default=8,
        help="Bits por palavra esperados. Default: 8",
    )
    cs_check.add_argument(
        "--expected-active-low",
        action="store_true",
        help="Tratar como OK apenas quando CS for ativo em nível baixo (cshigh=False).",
    )
    cs_check.add_argument(
        "--manual-cs",
        action="append",
        metavar="NAME=GPIO",
        help=(
            "GPIO(s) usados como CS manual, no formato NOME=GPIO. "
            "Pode repetir a opção: --manual-cs STM32=17 --manual-cs TMC5160=27"
        ),
    )
    cs_check.add_argument(
        "--toggle-test",
        action="store_true",
        help=(
            "Se informado, alterna os GPIOs de CS manual (alto->baixo->alto) para teste. "
            "Use com cuidado, isso pode ativar escravos momentaneamente."
        ),
    )
    cs_check.set_defaults(handler="cs_check", needs_client=False)

    examples = sub.add_parser(
        "examples",
        help="Listar os comandos disponíveis acompanhados de exemplos de uso",
    )
    examples.set_defaults(handler=print_examples, needs_client=False)

    # Safe-off: para o movimento no STM32 e coloca TMC5160 em alta impedância
    safe_off = sub.add_parser(
        "safe-off",
        help=(
            "Parar movimento (MOVE_END) e colocar drivers TMC5160 em modo de segurança (IHOLD/IRUN=0, TOFF=0, FREEWHEEL)."
        ),
    )
    _common_args(safe_off, include_tries=True, include_settle_delay=True)
    safe_off.add_argument("--frame-id", type=int, default=0, help="FrameId para MOVE_END (default: 0)")
    # Alvos TMC5160: por padrão age em TODOS os /dev/spidevX.Y
    safe_off.add_argument("--tmc-bus", type=int, help="SPI bus do TMC (omite para agir em todos)")
    safe_off.add_argument("--tmc-dev", type=int, help="SPI device do TMC (omite para agir em todos)")
    safe_off.add_argument("--tmc-speed", type=int, default=4_000_000, help="Velocidade SPI TMC (default: 4_000_000)")
    safe_off.add_argument(
        "--tmc-freewheel",
        type=int,
        choices=[0, 1, 2, 3],
        default=3,
        help="PWMCONF.FREEWHEEL para o TMC (0..3, default: 3 = alta impedância)",
    )
    safe_off.add_argument(
        "--skip-tmc",
        action="store_true",
        help="Somente enviar MOVE_END ao STM32; não altera TMC5160",
    )
    safe_off.set_defaults(handler="safe_off", needs_client=True)

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
            f"{base_cmd} queue-add --frame-id 2 --dir 0x03 --vx 1000 --sx 2000 "
            "--vy 1000 --sy 2000 --vz 500 --sz 800 --settle-delay 0.002",
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
        (
            "Checar CS / SPI",
            f"{base_cmd} cs-check --expected-mode 3 --manual-cs STM32=17 --manual-cs TMC5160=27",
        ),
    ]

    print("Comandos disponíveis e exemplos:")
    for title, command in examples:
        print(f"- {title}:\n  {command}")


def _decode_mode(mode: int) -> str:
    cpol = 1 if (mode & 0x02) else 0
    cpha = 1 if (mode & 0x01) else 0
    return f"mode={mode} (CPOL={cpol}, CPHA={cpha})"


def _list_spidev_nodes() -> List[Tuple[int, int, str]]:
    nodes = []
    for path in sorted(glob.glob("/dev/spidev*")):
        try:
            base = os.path.basename(path)
            _, bus_dev = base.split("spidev", 1)
            bus_s, dev_s = bus_dev.split(".")
            bus, dev = int(bus_s), int(dev_s)
            nodes.append((bus, dev, path))
        except Exception:
            continue
    return nodes


def _open_spidev(bus: int, dev: int):  # pragma: no cover - depende de hardware
    try:
        import spidev  # type: ignore
    except Exception:
        return None, "spidev não disponível (instale python3-spidev)"
    try:
        spi = spidev.SpiDev()
        spi.open(bus, dev)
        return spi, None
    except Exception as exc:
        return None, str(exc)


def _print_spidev_info(bus: int, dev: int, expected_mode: int, expected_bpw: int,
                       expected_active_low: bool) -> None:
    spi, err = _open_spidev(bus, dev)
    label = f"/dev/spidev{bus}.{dev}"
    print(f"Device: {label}")
    if spi is None:
        print(f"  open: FAIL: {err}")
        return
    try:
        mode = int(spi.mode)
        bpw = int(spi.bits_per_word or 0)
        speed = int(getattr(spi, "max_speed_hz", 0) or 0)
        cshigh = bool(getattr(spi, "cshigh", False))
        lsbfirst = bool(getattr(spi, "lsbfirst", False))
        threewire = bool(getattr(spi, "threewire", False))
        loop = bool(getattr(spi, "loop", False))
        no_cs = bool(getattr(spi, "no_cs", False))
        alias = None
        if bus == 0 and dev in (0, 1):
            alias = f"CE{dev} (BCM{8 if dev == 0 else 7})"
        print("  open: OK")
        if alias:
            print(f"  alias: {alias}")
        print(f"  mode: {_decode_mode(mode)}")
        print(f"  cshigh: {cshigh} (ativo={'alto' if cshigh else 'baixo'})")
        print(f"  bits_per_word: {bpw}")
        print(f"  max_speed_hz: {speed}")
        print(f"  lsbfirst: {lsbfirst}")
        print(f"  threewire: {threewire}")
        print(f"  loop: {loop}")
        print(f"  no_cs: {no_cs}")
        # Heurística simples de OK
        ok_flags: List[str] = []
        warn_flags: List[str] = []
        if expected_mode == mode:
            ok_flags.append("mode")
        else:
            warn_flags.append(f"mode esperado={expected_mode}")
        if expected_bpw == 0 or bpw in (0, expected_bpw):
            # Alguns drivers reportam 0 antes de setar explicitamente
            ok_flags.append("bpw")
        else:
            warn_flags.append(f"bpw esperado={expected_bpw}")
        if expected_active_low and not cshigh:
            ok_flags.append("cs ativo-baixo")
        elif expected_active_low and cshigh:
            warn_flags.append("CS configurado como ativo-alto (cshigh=True)")
        if no_cs:
            warn_flags.append("no_cs=True (CS de hardware desabilitado)")
        print("  check:")
        if ok_flags:
            print("    OK:", ", ".join(ok_flags))
        if warn_flags:
            print("    WARN:", ", ".join(warn_flags))
        # Teste rápido de troca
        try:
            rx = spi.xfer2([0x00])
            _ = len(rx)
            print("  transfer: OK (xfer2 de 1 byte)")
        except Exception as exc:
            print(f"  transfer: FAIL: {exc}")
    finally:
        try:
            spi.close()
        except Exception:
            pass


def _parse_manual_cs(items: Optional[List[str]]) -> Dict[str, int]:
    result: Dict[str, int] = {}
    if not items:
        return result
    for raw in items:
        if not raw:
            continue
        if "=" not in raw:
            raise ValueError(f"Formato inválido para --manual-cs: '{raw}'. Use NOME=GPIO.")
        name, num = raw.split("=", 1)
        name = name.strip() or "GPIO"
        try:
            gpio = int(num.strip())
        except ValueError:
            raise ValueError(f"GPIO inválido em --manual-cs: '{num}'.")
        result[name] = gpio
    return result


def _manual_cs_read_and_optionally_toggle(mapping: Dict[str, int], toggle: bool) -> None:
    if not mapping:
        return
    # Preferência: gpiod (v2). Fallback: RPi.GPIO
    gpiod_err: Optional[str] = None
    try:
        import gpiod  # type: ignore
        # Usar o primeiro chip por padrão (Raspberry Pi)
        chip = gpiod.Chip("gpiochip0")
        for name, line in mapping.items():
            print(f"Manual CS: {name} (GPIO{line})")
            try:
                req = chip.request_lines(
                    consumer="cs-check",
                    config={line: gpiod.LineSettings(direction=gpiod.LineDirection.INPUT)},
                )
                val = req.get_values([line])[0]
                print(f"  level: {'alto' if val else 'baixo'}")
                req.release()
                if toggle:
                    req = chip.request_lines(
                        consumer="cs-check",
                        config={
                            line: gpiod.LineSettings(direction=gpiod.LineDirection.OUTPUT,
                                                      output_value=gpiod.LineValue.ONE)
                        },
                    )
                    req.set_values({line: gpiod.LineValue.ZERO})
                    req.set_values({line: gpiod.LineValue.ONE})
                    req.release()
                    print("  toggle: OK (alto→baixo→alto)")
            except Exception as exc:
                print(f"  erro: {exc}")
        return
    except Exception as exc:
        gpiod_err = str(exc)
    # Fallback RPi.GPIO
    try:
        import RPi.GPIO as GPIO  # type: ignore
    except Exception as exc:
        print("Aviso: gpiod e RPi.GPIO indisponíveis. Pulei teste de CS manual.")
        if gpiod_err:
            print(f"  gpiod erro: {gpiod_err}")
        return
    try:
        GPIO.setmode(GPIO.BCM)
        for name, pin in mapping.items():
            print(f"Manual CS: {name} (GPIO{pin})")
            try:
                GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_OFF)
                val = GPIO.input(pin)
                print(f"  level: {'alto' if val else 'baixo'}")
                if toggle:
                    GPIO.setup(pin, GPIO.OUT, initial=GPIO.HIGH)
                    GPIO.output(pin, GPIO.LOW)
                    GPIO.output(pin, GPIO.HIGH)
                    print("  toggle: OK (alto→baixo→alto)")
            except Exception as exc:
                print(f"  erro: {exc}")
    finally:
        try:
            GPIO.cleanup()
        except Exception:
            pass


def cs_check(args: argparse.Namespace) -> None:
    nodes = _list_spidev_nodes()
    if not nodes:
        print("Nenhum /dev/spidevX.Y encontrado. Habilite SPI no Raspberry (raspi-config).")
    for bus, dev, path in nodes:
        _print_spidev_info(
            bus,
            dev,
            expected_mode=int(getattr(args, "expected_mode", 3) or 3),
            expected_bpw=int(getattr(args, "expected_bpw", 8) or 8),
            expected_active_low=bool(getattr(args, "expected_active_low", False)),
        )
    mapping = _parse_manual_cs(getattr(args, "manual_cs", None))
    if mapping:
        _manual_cs_read_and_optionally_toggle(mapping, bool(getattr(args, "toggle_test", False)))


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
