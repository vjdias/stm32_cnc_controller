#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Cliente SPI para comunicação com o firmware CNC no STM32."""

import argparse
import os
import sys
import time
import glob
import json
from decimal import Decimal, InvalidOperation, ROUND_HALF_UP
from pathlib import Path
from typing import List, Optional, Dict, Tuple

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .stm32_commands import STM32CommandExecutor
    from .stm32_responses import STM32ResponseDecoder
    from .stm32_protocol import (
        SPI_DMA_CLIENT_POLL_BYTE,
        RESP_HEADER,
        RESP_TAIL,
        REQ_SET_ORIGIN,
        REQ_MOTION_ESTIMATE,
        REQ_DIAG_CTRL,
        REQ_SET_ENC_PPR,
    )
    from .stm32_requests import STM32RequestBuilder
else:  # execução direta do script a partir do diretório raspberry_spi
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from stm32_commands import STM32CommandExecutor  # type: ignore
    from stm32_responses import STM32ResponseDecoder  # type: ignore
    from stm32_protocol import (  # type: ignore
        SPI_DMA_CLIENT_POLL_BYTE,
        RESP_HEADER,
        RESP_TAIL,
        REQ_SET_ORIGIN,
        REQ_MOTION_ESTIMATE,
        REQ_DIAG_CTRL,
        REQ_SET_ENC_PPR,
    )
    from stm32_requests import STM32RequestBuilder  # type: ignore

try:  # pragma: no cover - dependência externa
    import spidev  # type: ignore
except Exception:  # pragma: no cover
    spidev = None


# Helpers essenciais do cliente STM32 (enxutos)
def _build_spi_dma_frame(payload: List[int], filler: int = 0x00, *, frame_len: int = 42) -> List[int]:
    if len(payload) > frame_len:
        raise ValueError(f"payload excede {frame_len} bytes: {len(payload)}")
    frame = [filler & 0xFF] * frame_len
    start = frame_len - len(payload)
    for idx, byte in enumerate(payload):
        frame[start + idx] = byte & 0xFF
    return frame


def _validate_handshake_frame(tx_frame: List[int], handshake_frame: List[int], payload_len: int) -> None:
    if payload_len <= 0:
        raise ValueError("payload_len deve ser positivo")
    if len(tx_frame) != len(handshake_frame):
        raise ValueError(
            "Comprimento do handshake difere do frame transmitido: "
            f"tx={len(tx_frame)} rx={len(handshake_frame)}"
        )
    prefix_len = len(tx_frame) - payload_len
    if prefix_len < 0:
        raise ValueError("payload_len maior que o frame transmitido")
    normalized_statuses = [b & 0xFF for b in handshake_frame]
    if not normalized_statuses:
        raise ValueError("handshake_frame vazio")
    # Aceita READY puro
    if set(normalized_statuses) == {0xA5}:  # READY
        return
    # Aceita presença do header/tail no buffer
    try:
        header_idx = normalized_statuses.index(RESP_HEADER)
        tail_idx = normalized_statuses.index(RESP_TAIL, header_idx + 1)
        if header_idx >= 0 and tail_idx > header_idx:
            return
    except ValueError:
        pass
    # Aceita NO_COMM/READY mistos (transição de preenchimento)
    if {0xA5, 0x00}.issuperset(set(normalized_statuses)) and any(b == 0xA5 for b in normalized_statuses):
        return
    # BUSY puro
    if set(normalized_statuses) == {0x5A}:
        raise BufferError("STM32 respondeu BUSY (0x5A) para todo o frame DMA. Aguarde e tente novamente.")


def _extract_response_frame(rx_frame: List[int], expected_len: int, expected_type: int) -> List[int] | None:
    """Extrai um frame de resposta procurando por HEADER..TAIL do tipo esperado.

    Compatível com respostas de comprimento variável (ex.: 4 ou 5 bytes),
    tentando primeiro `expected_len` e, se não casar, procurando o próximo TAIL.
    """
    if expected_len <= 0 or not rx_frame:
        return None
    normalized = [b & 0xFF for b in rx_frame]
    try:
        header_idx = normalized.index(RESP_HEADER)
    except ValueError:
        if 0x5A in normalized:
            raise BufferError("STM32 respondeu BUSY (0x5A) durante o polling da resposta.")
        return None

    # 1) Tenta extração rígida com expected_len
    end_idx = header_idx + expected_len
    if end_idx <= len(normalized):
        frame = normalized[header_idx:end_idx]
        if (
            len(frame) == expected_len
            and frame[0] == RESP_HEADER
            and frame[-1] == RESP_TAIL
            and frame[1] == expected_type
        ):
            return frame

    # 2) Flexível: procura TAIL nas próximas posições e valida tipo
    # Varre até 12 bytes adiante do HEADER para achar um TAIL válido
    search_max = min(len(normalized) - 1, header_idx + max(expected_len, 12))
    for tail_idx in range(header_idx + 3, search_max + 1):
        if normalized[tail_idx] != RESP_TAIL:
            continue
        # Checa tipo esperado logo após HEADER
        if header_idx + 1 < len(normalized) and normalized[header_idx + 1] == expected_type:
            candidate = normalized[header_idx : tail_idx + 1]
            # Mínimo de 4 bytes [HDR,TYPE,*,TAIL]
            if len(candidate) >= 4:
                return candidate
    return None


class STM32Client:
    def __init__(
        self,
        bus: int = 0,
        dev: int = 0,
        speed_hz: int = 1_000_000,
        mode: int = 0b11,
        *,
        log_format: str = "hex",
    ) -> None:
        if spidev is None:
            raise RuntimeError("spidev não disponível. Instale `python3-spidev` no Raspberry.")
        self.spi = spidev.SpiDev()
        self.spi.open(bus, dev)
        self.spi.max_speed_hz = int(speed_hz)
        self.spi.mode = mode  # MODE 3: 0b11
        self.spi.bits_per_word = 8
        self._log_format = str(log_format or "hex").lower().strip()

    @staticmethod
    def _format_bytes(bs: List[int]) -> str:
        return " ".join(f"{b & 0xFF:08b}" for b in bs)

    def _xfer(self, tx: List[int]) -> List[int]:  # pragma: no cover - depende de hardware
        try:
            if self._log_format == "bin":
                print("SPI TX bits:", self._format_bytes(tx))
        except Exception:
            pass
        rx = self.spi.xfer2(tx)
        try:
            if self._log_format == "bin":
                print("SPI RX bits:", self._format_bytes(rx))
        except Exception:
            pass
        return rx

    def exchange(
        self,
        request_type: int,
        request: List[int],
        tries: int = 0,
        settle_delay_s: float = 0.001,
        poll_byte: int | None = SPI_DMA_CLIENT_POLL_BYTE,
    ) -> List[int]:
        if tries < 0:
            raise ValueError("tries cannot be negative")
        spec = STM32ResponseDecoder.SPECS[request_type]
        dma_frame = _build_spi_dma_frame(request)
        rx_frame = self._xfer(dma_frame)
        _validate_handshake_frame(dma_frame, rx_frame, len(request))
        handshake_response = _extract_response_frame(rx_frame, spec.length, spec.response_type)
        if handshake_response is not None:
            return handshake_response
        if settle_delay_s > 0:
            time.sleep(settle_delay_s)

        if poll_byte is None:
            raise TimeoutError("Polling desabilitado e resposta não estava presente no handshake.")

        poll_payload_len = max(1, len(request))
        for _ in range(max(1, tries)):
            poll = [poll_byte & 0xFF] * poll_payload_len
            rx = self._xfer(_build_spi_dma_frame(poll))
            r = _extract_response_frame(rx, spec.length, spec.response_type)
            if r is not None:
                return r
            if settle_delay_s > 0:
                time.sleep(settle_delay_s)
        raise TimeoutError("Resposta não encontrada após polling SPI.")

    def poll_for(
        self,
        expected_response_type: int,
        *,
        expected_len: int,
        tries: int = 100,
        settle_delay_s: float = 0.01,
        poll_byte: int = SPI_DMA_CLIENT_POLL_BYTE,
    ) -> List[int]:  # pragma: no cover - depende de hardware
        """Faz polling do buffer DMA do STM32 até encontrar o tipo de resposta esperado.

        Não envia nenhum comando, apenas quadros de polling (byte repetido).
        """
        payload_len = 1
        for _ in range(max(1, tries)):
            rx = self._xfer(_build_spi_dma_frame([poll_byte] * payload_len))
            r = _extract_response_frame(rx, expected_len, expected_response_type)
            if r is not None:
                return r
            if settle_delay_s > 0:
                time.sleep(settle_delay_s)
        raise TimeoutError("Polling atingiu o limite sem encontrar a resposta esperada.")

    def close(self) -> None:  # pragma: no cover - depende de hardware
        try:
            spi = getattr(self, "spi", None)
            if spi is not None:
                spi.close()
        except Exception:
            pass


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

    # queue-status removido na versão sucinta

    start_move = sub.add_parser("start-move", help="Iniciar execução")
    _common_args(start_move, include_tries=True, include_settle_delay=True)
    start_move.add_argument("--frame-id", type=int, required=True)
    start_move.add_argument("--wait-end", action="store_true", help="Aguarda e imprime RESP_MOVE_END após o ack do start")
    start_move.add_argument("--end-timeout", type=float, default=60.0, help="Tempo máximo (s) aguardando MOVE_END quando --wait-end")
    start_move.set_defaults(handler="start_move", needs_client=True)

    end_move = sub.add_parser("end-move", help="Finalizar execução")
    _common_args(end_move, include_tries=True, include_settle_delay=True)
    end_move.add_argument("--frame-id", type=int, required=True)
    end_move.set_defaults(handler="end_move", needs_client=True)

    # home removido

    # probe-level removido

    # hello removido

    # boot-hello removido

    # led (boot) removido

    # Verificação de CS / SPI (removido)
    # examples removido
    # Safe-off removido

    # Config: Atualiza application.cfg (test.start/initial)
    cfg = sub.add_parser(
        "config",
        help="Atualiza application.cfg (test.start/initial)",
    )
    cfg.add_argument("--file", type=str, default="application.cfg", help="Caminho do application.cfg")
    cfg.add_argument("--set-start", nargs=3, metavar=("X","Y","Z"), type=float, default=None,
                     help="Atualiza test.start = (X,Y,Z)")
    cfg.add_argument("--set-initial", nargs=3, metavar=("X","Y","Z"), type=float, default=None,
                     help="Atualiza test.initial = (X,Y,Z)")
    cfg.set_defaults(handler="config_set", needs_client=False)

    # Set encoder origin no firmware
    origin = sub.add_parser("origin-set", help="Fixar origem dos encoders (g_encoder_origin=g_encoder_position)")
    _common_args(origin, include_tries=True, include_settle_delay=True)
    origin.add_argument("--frame-id", type=int, default=0x42)
    origin.add_argument("--mask", type=lambda x: int(x,0), default=0x07, help="Axes mask (bit0 X, bit1 Y, bit2 Z)")
    origin.add_argument("--mode", type=str, choices=["start","initial"], default="start")
    origin.set_defaults(handler="origin_set", needs_client=True)

    # Encoder status (abs/rel), pid errors, delta
    # Estimativas de movimento (médias de acel/cruzeiro/desacel)
    est = sub.add_parser("model-estimate", help="Consultar médias de aceleração, cruzeiro e desaceleração (após teste)")
    _common_args(est, include_tries=True, include_settle_delay=True)
    est.add_argument("--frame-id", type=int, default=0x60)
    est.set_defaults(handler="model_estimate", needs_client=True)

    # Rodar teste de movimento (um eixo) com opção de imprimir SWO
    mv = sub.add_parser("model-run", help="Executa movimento de teste (um eixo) para estimativa de parâmetros")
    _common_args(mv, include_tries=True, include_settle_delay=True)
    mv.add_argument("--frame-id", type=int, default=0x50)
    mv.add_argument("--axis", choices=("x","y","z"), default="x")
    mv.add_argument("--neg", action="store_true", help="Direção negativa (inverte DIR do eixo)")
    mv.add_argument("--vx", type=int, required=True, help="Velocidade em steps/s (comando)")
    mv.add_argument("--turns", type=int, required=True, help="Número de voltas completas (1..20)")
    mv.add_argument("--ppr", type=int, default=None, help="Pulsos do encoder por volta (default por eixo: X=40000,Y=2500,Z=40000)")
    mv.add_argument("--swo-print", action="store_true", help="Habilita linhas SPD:<axis> via SWO durante o movimento")
    mv.add_argument("--wait-end", action="store_true", help="Aguarda RESP_MOVE_END após iniciar")
    mv.set_defaults(handler="move_speed", needs_client=True)

    return parser


def print_examples(_: argparse.Namespace) -> None:
    # Preferir o entrypoint instalado globalmente
    base_cmd = "stm32-cli"
    examples = [
        ("LED discreto",
         f"{base_cmd} led-control --frame-id 1 --mask 0x01 --led1-mode 2 --led1-freq 0.5"),
        ("Rodar teste (modelo)",
         f"{base_cmd} model-run --axis x --vx 2000 --turns 1 --ppr 40000 --wait-end --swo-print"),
        ("Ler estimativas",
         f"{base_cmd} model-estimate --frame-id 0x60"),
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


def _rpi_cs_info(bus: int, dev: int) -> Optional[Tuple[str, int]]:
    """Retorna (label, bcm_pin) do CS padrão do Raspberry para (bus, dev).
    Somente mapeia pares comuns (SPI0/1)."""
    mapping: Dict[Tuple[int, int], Tuple[str, int]] = {
        (0, 0): ("SPI0.CE0", 8),
        (0, 1): ("SPI0.CE1", 7),
        # (0, 2) não é exposto na maioria dos modelos
        (1, 0): ("SPI1.CE0", 18),
        (1, 1): ("SPI1.CE1", 17),
        (1, 2): ("SPI1.CE2", 16),
    }
    return mapping.get((bus, dev))


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
        print("  open: OK")
        cs_info = _rpi_cs_info(bus, dev)
        if cs_info:
            cs_label, bcm_pin = cs_info
            print(f"  cs_line: {cs_label}")
            print(f"  cs_pin_bcm: {bcm_pin}")
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


def origin_set(args: argparse.Namespace) -> None:
    frame_id = int(getattr(args, "frame_id", 0) or 0)
    mask = int(getattr(args, "mask", 0x07) or 0x07)
    mode = 0 if str(getattr(args, "mode", "start")).lower() == "start" else 1
    req = STM32RequestBuilder.set_origin(frame_id, mask, mode)
    resp = _client_instance.exchange(REQ_SET_ORIGIN, req, tries=int(getattr(args, "tries", 3) or 3), settle_delay_s=float(getattr(args, "settle_delay", 0.002) or 0.002))
    try:
        data = STM32ResponseDecoder.set_origin(resp)
    except Exception:
        print("Resposta set_origin bruta:", resp)
        return
    print(data)


def model_estimate(args: argparse.Namespace) -> None:
    frame_id = int(getattr(args, "frame_id", 0) or 0)
    req = STM32RequestBuilder.motion_estimate(frame_id)
    resp = _client_instance.exchange(REQ_MOTION_ESTIMATE, req, tries=int(getattr(args, "tries", 3) or 3), settle_delay_s=float(getattr(args, "settle_delay", 0.002) or 0.002))
    try:
        data = STM32ResponseDecoder.motion_estimate(resp)
    except Exception:
        print("Resposta motion-estimate bruta:", resp)
        return
    print(data)


def move_speed(args: argparse.Namespace) -> None:
    # Monta um segmento simples para um único eixo com velocidade fixa
    frame_id = int(getattr(args, "frame_id", 0) or 0)
    axis = str(getattr(args, "axis", "x") or "x").lower()
    neg = bool(getattr(args, "neg", False))
    vx = int(getattr(args, "vx", 0) or 0)
    turns = int(getattr(args, "turns", 0) or 0)
    if turns <= 0 or vx <= 0 or turns > 20:
        print("Erro: informe --vx>0 e --turns entre 1 e 20")
        return
    axis_idx = {"x": 0, "y": 1, "z": 2}[axis]
    dir_mask = (1 << axis_idx) if neg else 0

    # 1) Ajustar PPR no firmware para o eixo
    default_ppr = {0: 40000, 1: 2500, 2: 40000}[axis_idx]
    ppr = int(getattr(args, "ppr", default_ppr) or default_ppr)
    req_ppr = STM32RequestBuilder.set_enc_ppr(frame_id, axis_idx, ppr)
    _client_instance.exchange(REQ_SET_ENC_PPR, req_ppr, tries=int(getattr(args, "tries", 3) or 3), settle_delay_s=float(getattr(args, "settle_delay", 0.002) or 0.002))
    # 0) Controle de diagnóstico (SWO SPD on/off)
    if bool(getattr(args, "swo_print", False)):
        dreq = STM32RequestBuilder.diag_ctrl(frame_id, 0x01)
        _client_instance.exchange(REQ_DIAG_CTRL, dreq, tries=int(getattr(args, "tries", 3) or 3), settle_delay_s=float(getattr(args, "settle_delay", 0.002) or 0.002))
    else:
        dreq = STM32RequestBuilder.diag_ctrl(frame_id, 0x00)
        _client_instance.exchange(REQ_DIAG_CTRL, dreq, tries=int(getattr(args, "tries", 3) or 3), settle_delay_s=float(getattr(args, "settle_delay", 0.002) or 0.002))

    # 2) Envia comando RAW de execução (sem fila/CASC)
    req = STM32RequestBuilder.model_run(frame_id, axis_idx, neg, vx, turns)
    ack = _client_instance.exchange(REQ_MODEL_RUN, req, tries=int(getattr(args, "tries", 3) or 3), settle_delay_s=float(getattr(args, "settle_delay", 0.002) or 0.002))
    print("ack:", ack)

    if bool(getattr(args, "wait_end", False)):
        # O firmware encerra automaticamente quando o encoder atingir N×PPR.
        import time as _t
        _t.sleep(0.1 * max(1, turns))

    if bool(getattr(args, "swo_print", False)):
        print("Dica: monitore SWO por linhas 'SPD:<axis> cmd=.. meas=.. sps'.")


def _load_cfg_with_fallback(path: str) -> tuple[dict, str]:
    cfg_path = path or "application.cfg"
    candidates = [cfg_path]
    if cfg_path == "application.cfg":
        try:
            here = Path(__file__).resolve().parent.parent
            pkg = here / "application.cfg"
            candidates.append(str(pkg))
        except Exception:
            pass
    for p in candidates:
        try:
            if os.path.exists(p):
                with open(p, "r", encoding="utf-8") as f:
                    return json.load(f), p
        except Exception:
            continue
    return {}, cfg_path


def config_set(args: argparse.Namespace) -> None:
    cfg, used = _load_cfg_with_fallback(str(getattr(args, "file", "application.cfg") or "application.cfg"))
    if not isinstance(cfg.get("test"), dict):
        cfg["test"] = {}
    if getattr(args, "set_start", None) is not None:
        sx, sy, sz = [float(v) for v in args.set_start]
        cfg["test"]["start"] = {"x": sx, "y": sy, "z": sz}
        print("Atualizado test.start:", cfg["test"]["start"])
    if getattr(args, "set_initial", None) is not None:
        ix, iy, iz = [float(v) for v in args.set_initial]
        cfg["test"]["initial"] = {"x": ix, "y": iy, "z": iz}
        print("Atualizado test.initial:", cfg["test"]["initial"])
    out = str(getattr(args, "file", used) or used)
    try:
        with open(out, "w", encoding="utf-8") as f:
            json.dump(cfg, f, indent=2)
        print("Salvo:", out)
    except Exception as exc:
        print("Erro ao salvar cfg:", exc)


def main(argv: Optional[List[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)

    handler = getattr(args, "handler", None)
    if handler is None:
        parser.error("Nenhum comando informado")

    needs_client = getattr(args, "needs_client", True)

    client: Optional[STM32Client] = None
    executor: Optional[STM32CommandExecutor] = None
    try:
        if needs_client:
            log_format = getattr(args, "spi_log_format", "hex")
            client = STM32Client(
                bus=args.bus,
                dev=args.dev,
                speed_hz=args.speed,
                log_format=log_format,
            )
            executor = STM32CommandExecutor(client)
            # Disponibiliza para handlers simples (origin_set/encoder_status)
            global _client_instance
            _client_instance = client

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
