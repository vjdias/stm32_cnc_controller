#!/usr/bin/env python3
"""Ferramenta de host para trocar quadros SPI com o firmware CNC_Controller.

O script replica a camada de protocolo descrita em ``App/Inc/Protocol`` do
firmware STM32L475: cabeçalhos 0xAA/0xAB, tails 0x55/0x54 e os campos de cada
request/response. Ele foi pensado para ser executado em um Raspberry Pi (ou
qualquer host Linux com ``spidev``) atuando como master do barramento SPI.

Principais recursos:

* Geração dos frames de *request* com paridade idêntica à do firmware.
* Decodificação básica das respostas, validando a paridade quando existir.
* Comando ``listen`` para apenas coletar respostas já enfileiradas pelo STM32.
* ``--dry-run`` automático quando ``spidev`` não está disponível (ex.: testes).

Exemplos rápidos::

    # Acender o LED on-board usando máscara 0x01 (monocromático)
    python cnc_spi_client.py led --mask 0x01 --r 1 --frame-id 1

    # Consultar o status da fila de movimentos
    python cnc_spi_client.py queue-status --frame-id 2

    # Apenas observar respostas pendentes (útil após comandos anteriores)
    python cnc_spi_client.py listen --count 3
"""

from __future__ import annotations

import argparse
import enum
import importlib.util
import struct
import sys
import time
from typing import Dict, List, Optional, Sequence

REQ_HEADER = 0xAA
REQ_TAIL = 0x55
RESP_HEADER = 0xAB
RESP_TAIL = 0x54

_SPIDEV_SPEC = importlib.util.find_spec("spidev")
if _SPIDEV_SPEC is not None and _SPIDEV_SPEC.loader is not None:
    spidev = importlib.util.module_from_spec(_SPIDEV_SPEC)
    _SPIDEV_SPEC.loader.exec_module(spidev)  # type: ignore[attr-defined]
else:
    spidev = None  # type: ignore[assignment]

HAVE_SPIDEV = spidev is not None


class RequestType(enum.IntEnum):
    """IDs dos requests, iguais ao enum ``req_msg_type_t`` do firmware."""

    MOVE_QUEUE_ADD = 0x01
    MOVE_QUEUE_STATUS = 0x02
    START_MOVE = 0x03
    MOVE_HOME = 0x04
    MOVE_PROBE_LEVEL = 0x05
    MOVE_END = 0x06
    LED_CTRL = 0x07
    FPGA_STATUS = 0x20


class ResponseType(enum.IntEnum):
    """IDs das respostas, iguais ao enum ``resp_msg_type_t`` do firmware."""

    MOVE_QUEUE_ADD_ACK = 0x01
    MOVE_QUEUE_STATUS = 0x02
    START_MOVE = 0x03
    MOVE_HOME = 0x04
    MOVE_PROBE_LEVEL = 0x05
    MOVE_END = 0x06
    LED_CTRL = 0x07
    FPGA_STATUS = 0x20
    HOME_STATUS = 0x21


class SpiError(RuntimeError):
    """Erro para sinalizar problemas na camada SPI do host."""


def _ensure_u8(value: int, name: str) -> int:
    if not 0 <= value <= 0xFF:
        raise ValueError(f"{name} fora de faixa (0..255): {value}")
    return value & 0xFF


def _ensure_u16(value: int, name: str) -> int:
    if not 0 <= value <= 0xFFFF:
        raise ValueError(f"{name} fora de faixa (0..65535): {value}")
    return value & 0xFFFF


def _ensure_u32(value: int, name: str) -> int:
    if not 0 <= value <= 0xFFFFFFFF:
        raise ValueError(f"{name} fora de faixa (0..4294967295): {value}")
    return value & 0xFFFFFFFF


def _xor_reduce(data: Sequence[int]) -> int:
    x = 0
    for b in data:
        x ^= b & 0xFF
    return x & 0xFF


def _xor_bit_reduce(data: Sequence[int]) -> int:
    x = _xor_reduce(data)
    x ^= x >> 4
    x ^= x >> 2
    x ^= x >> 1
    return x & 0x1


def _parity_byte_1n(frame: Sequence[int], last_index: int) -> int:
    return _xor_reduce(frame[1 : last_index + 1])


def _parity_bit_1n(frame: Sequence[int], last_index: int) -> int:
    return _xor_bit_reduce(frame[1 : last_index + 1])


def build_move_queue_status(frame_id: int) -> bytes:
    frame = bytearray(4)
    frame[0] = REQ_HEADER
    frame[1] = RequestType.MOVE_QUEUE_STATUS
    frame[2] = _ensure_u8(frame_id, "frame_id")
    frame[3] = REQ_TAIL
    return bytes(frame)


def build_start_move(frame_id: int) -> bytes:
    frame = bytearray(4)
    frame[0] = REQ_HEADER
    frame[1] = RequestType.START_MOVE
    frame[2] = _ensure_u8(frame_id, "frame_id")
    frame[3] = REQ_TAIL
    return bytes(frame)


def build_move_end(frame_id: int) -> bytes:
    frame = bytearray(4)
    frame[0] = REQ_HEADER
    frame[1] = RequestType.MOVE_END
    frame[2] = _ensure_u8(frame_id, "frame_id")
    frame[3] = REQ_TAIL
    return bytes(frame)


def build_fpga_status(frame_id: int) -> bytes:
    frame = bytearray(4)
    frame[0] = REQ_HEADER
    frame[1] = RequestType.FPGA_STATUS
    frame[2] = _ensure_u8(frame_id, "frame_id")
    frame[3] = REQ_TAIL
    return bytes(frame)


def build_led_ctrl(frame_id: int, led_mask: int, r: int, g: int, b: int) -> bytes:
    frame = bytearray(9)
    frame[0] = REQ_HEADER
    frame[1] = RequestType.LED_CTRL
    frame[2] = _ensure_u8(frame_id, "frame_id")
    frame[3] = _ensure_u8(led_mask, "led_mask")
    frame[4] = _ensure_u8(r, "r")
    frame[5] = _ensure_u8(g, "g")
    frame[6] = _ensure_u8(b, "b")
    frame[7] = _parity_byte_1n(frame, 6)
    frame[8] = REQ_TAIL
    return bytes(frame)


def build_move_home(frame_id: int, axis_mask: int, dir_mask: int, vhome: int) -> bytes:
    frame = bytearray(9)
    frame[0] = REQ_HEADER
    frame[1] = RequestType.MOVE_HOME
    frame[2] = _ensure_u8(frame_id, "frame_id")
    frame[3] = _ensure_u8(axis_mask, "axis_mask")
    frame[4] = _ensure_u8(dir_mask, "dir_mask")
    struct.pack_into(">H", frame, 5, _ensure_u16(vhome, "vhome"))
    frame[7] = _parity_byte_1n(frame, 6)
    frame[8] = REQ_TAIL
    return bytes(frame)


def build_move_probe_level(frame_id: int, axis_mask: int, vprobe: int) -> bytes:
    frame = bytearray(8)
    frame[0] = REQ_HEADER
    frame[1] = RequestType.MOVE_PROBE_LEVEL
    frame[2] = _ensure_u8(frame_id, "frame_id")
    frame[3] = _ensure_u8(axis_mask, "axis_mask")
    struct.pack_into(">H", frame, 4, _ensure_u16(vprobe, "vprobe"))
    frame[6] = _parity_byte_1n(frame, 5)
    frame[7] = REQ_TAIL
    return bytes(frame)


def build_move_queue_add(
    frame_id: int,
    dir_mask: int,
    vx: int,
    sx: int,
    vy: int,
    sy: int,
    vz: int,
    sz: int,
    kp_x: int,
    ki_x: int,
    kd_x: int,
    kp_y: int,
    ki_y: int,
    kd_y: int,
    kp_z: int,
    ki_z: int,
    kd_z: int,
) -> bytes:
    frame = bytearray(42)
    frame[0] = REQ_HEADER
    frame[1] = RequestType.MOVE_QUEUE_ADD
    frame[2] = _ensure_u8(frame_id, "frame_id")
    frame[3] = _ensure_u8(dir_mask, "dir_mask")
    struct.pack_into(">H", frame, 4, _ensure_u16(vx, "vx"))
    struct.pack_into(">I", frame, 6, _ensure_u32(sx, "sx"))
    struct.pack_into(">H", frame, 10, _ensure_u16(vy, "vy"))
    struct.pack_into(">I", frame, 12, _ensure_u32(sy, "sy"))
    struct.pack_into(">H", frame, 16, _ensure_u16(vz, "vz"))
    struct.pack_into(">I", frame, 18, _ensure_u32(sz, "sz"))
    struct.pack_into(">H", frame, 22, _ensure_u16(kp_x, "kp_x"))
    struct.pack_into(">H", frame, 24, _ensure_u16(ki_x, "ki_x"))
    struct.pack_into(">H", frame, 26, _ensure_u16(kd_x, "kd_x"))
    struct.pack_into(">H", frame, 28, _ensure_u16(kp_y, "kp_y"))
    struct.pack_into(">H", frame, 30, _ensure_u16(ki_y, "ki_y"))
    struct.pack_into(">H", frame, 32, _ensure_u16(kd_y, "kd_y"))
    struct.pack_into(">H", frame, 34, _ensure_u16(kp_z, "kp_z"))
    struct.pack_into(">H", frame, 36, _ensure_u16(ki_z, "ki_z"))
    struct.pack_into(">H", frame, 38, _ensure_u16(kd_z, "kd_z"))
    frame[40] = _parity_bit_1n(frame, 39)
    frame[41] = REQ_TAIL
    return bytes(frame)


def decode_response(frame: bytes) -> Dict[str, object]:
    if len(frame) < 3:
        raise ValueError("frame muito curto")
    if frame[0] != RESP_HEADER or frame[-1] != RESP_TAIL:
        raise ValueError("header/tail inválidos")
    try:
        resp_type = ResponseType(frame[1])
    except ValueError as exc:  # pragma: no cover - enum desconhecido
        raise ValueError(f"tipo de resposta desconhecido: 0x{frame[1]:02X}") from exc

    info: Dict[str, object] = {"type": resp_type, "frame_id": frame[2]}

    if resp_type == ResponseType.LED_CTRL:
        if len(frame) != 7:
            raise ValueError("LED_CTRL deve ter 7 bytes")
        expected = _parity_byte_1n(frame, 4)
        if frame[5] != expected:
            raise ValueError("paridade inválida em LED_CTRL")
        info.update({"led_mask": frame[3], "status": frame[4]})
    elif resp_type == ResponseType.MOVE_QUEUE_ADD_ACK:
        if len(frame) != 6:
            raise ValueError("MOVE_QUEUE_ADD_ACK deve ter 6 bytes")
        expected = _parity_bit_1n(frame, 3)
        if (frame[4] & 0x1) != expected:
            raise ValueError("paridade inválida em MOVE_QUEUE_ADD_ACK")
        info.update({"status": frame[3]})
    elif resp_type == ResponseType.MOVE_QUEUE_STATUS:
        if len(frame) != 12:
            raise ValueError("MOVE_QUEUE_STATUS deve ter 12 bytes")
        expected = _parity_bit_1n(frame, 9)
        if (frame[10] & 0x1) != expected:
            raise ValueError("paridade inválida em MOVE_QUEUE_STATUS")
        info.update(
            {
                "status": frame[3],
                "pid_error": (frame[4], frame[5], frame[6]),
                "duty_cycle_pct": (frame[7], frame[8], frame[9]),
            }
        )
    elif resp_type == ResponseType.START_MOVE:
        if len(frame) != 4:
            raise ValueError("START_MOVE deve ter 4 bytes")
    elif resp_type == ResponseType.MOVE_HOME:
        if len(frame) != 8:
            raise ValueError("MOVE_HOME deve ter 8 bytes")
        expected = _parity_byte_1n(frame, 5)
        if frame[6] != expected:
            raise ValueError("paridade inválida em MOVE_HOME")
        info.update(
            {
                "status": frame[3],
                "axis_home_mask": frame[4],
                "error_flags": frame[5],
            }
        )
    elif resp_type == ResponseType.MOVE_PROBE_LEVEL:
        if len(frame) != 20:
            raise ValueError("MOVE_PROBE_LEVEL deve ter 20 bytes")
        expected = _parity_byte_1n(frame, 17)
        if frame[18] != expected:
            raise ValueError("paridade inválida em MOVE_PROBE_LEVEL")
        info.update(
            {
                "status": frame[3],
                "axis_done_mask": frame[4],
                "error_flags": frame[5],
                "latched_pos": (
                    struct.unpack_from(">I", frame, 6)[0],
                    struct.unpack_from(">I", frame, 10)[0],
                    struct.unpack_from(">I", frame, 14)[0],
                ),
            }
        )
    elif resp_type == ResponseType.MOVE_END:
        if len(frame) != 4:
            raise ValueError("MOVE_END deve ter 4 bytes")
    elif resp_type == ResponseType.HOME_STATUS:
        if len(frame) != 18:
            raise ValueError("HOME_STATUS deve ter 18 bytes")
        expected = _parity_byte_1n(frame, 15)
        if frame[16] != expected:
            raise ValueError("paridade inválida em HOME_STATUS")
        info.update(
            {
                "axis_mask": frame[3],
                "pos_rel": (
                    struct.unpack_from(">H", frame, 4)[0],
                    struct.unpack_from(">H", frame, 8)[0],
                    struct.unpack_from(">H", frame, 12)[0],
                ),
                "home_offset": (
                    struct.unpack_from(">H", frame, 6)[0],
                    struct.unpack_from(">H", frame, 10)[0],
                    struct.unpack_from(">H", frame, 14)[0],
                ),
            }
        )
    elif resp_type == ResponseType.FPGA_STATUS:
        info["payload"] = frame[2:-1]
    else:  # pragma: no cover - valores adicionais no futuro
        info["payload"] = frame[2:-1]

    return info


class SpiClient:
    """Pequeno *wrapper* para ``spidev`` com suporte a *dry-run*."""

    def __init__(
        self,
        bus: int,
        device: int,
        max_speed_hz: int,
        mode: int,
        bits_per_word: int,
        poll_delay: float,
        dry_run: bool,
    ) -> None:
        self.bus = bus
        self.device = device
        self.max_speed_hz = max_speed_hz
        self.mode = mode
        self.bits_per_word = bits_per_word
        self.poll_delay = poll_delay
        self.dry_run = dry_run or not HAVE_SPIDEV
        self._spi: Optional["spidev.SpiDev"] = None

    def open(self) -> None:
        if self._spi or self.dry_run:
            if self.dry_run and not HAVE_SPIDEV:
                return
            if self._spi:
                return
        if not HAVE_SPIDEV:
            raise SpiError(
                "spidev não está disponível; instale python3-spidev ou use --dry-run"
            )
        dev = spidev.SpiDev()
        dev.open(self.bus, self.device)
        dev.mode = self.mode & 0x3
        dev.max_speed_hz = self.max_speed_hz
        dev.bits_per_word = self.bits_per_word
        self._spi = dev

    def close(self) -> None:
        if self._spi is not None:
            self._spi.close()
            self._spi = None

    def transfer(self, data: Sequence[int]) -> List[int]:
        if self.dry_run or self._spi is None:
            return [0] * len(data)
        return self._spi.xfer2(list(data))

    def read_response(
        self, *, max_len: int, timeout: float, chunk_size: int
    ) -> Optional[bytes]:
        if self.dry_run or self._spi is None:
            return None
        if chunk_size <= 0:
            raise ValueError("chunk_size deve ser > 0")
        deadline = time.monotonic() + max(timeout, 0.0)
        buf = bytearray()
        while time.monotonic() < deadline:
            rx = self.transfer([0x00] * chunk_size)
            buf.extend(rx)
            if len(buf) > max_len:
                del buf[: len(buf) - max_len]
            while buf and buf[0] != RESP_HEADER:
                del buf[0]
            if not buf:
                time.sleep(self.poll_delay)
                continue
            try:
                tail_index = buf.index(RESP_TAIL, 1)
            except ValueError:
                time.sleep(self.poll_delay)
                continue
            frame = bytes(buf[: tail_index + 1])
            del buf[: tail_index + 1]
            return frame
        return None


def _parse_int(value: str) -> int:
    return int(value, 0)


def _format_int(value: int) -> str:
    return f"{value} (0x{value:02X})"


def _format_decoded(data: Dict[str, object]) -> str:
    items: List[str] = []
    for key, value in data.items():
        if key == "type" and isinstance(value, ResponseType):
            items.append(f"type={value.name} (0x{value.value:02X})")
        elif isinstance(value, int):
            items.append(f"{key}={_format_int(value)}")
        elif isinstance(value, tuple):
            tuple_str = ",".join(_format_int(v) if isinstance(v, int) else str(v) for v in value)
            items.append(f"{key}=({tuple_str})")
        else:
            items.append(f"{key}={value}")
    return ", ".join(items)


def _send_and_maybe_read(args: argparse.Namespace, frame: bytes) -> None:
    print(f"TX[{len(frame)}]: {frame.hex()}")
    client = SpiClient(
        bus=args.bus,
        device=args.device,
        max_speed_hz=args.max_speed,
        mode=args.mode,
        bits_per_word=args.bits_per_word,
        poll_delay=args.poll_delay,
        dry_run=args.dry_run,
    )
    if args.dry_run and not HAVE_SPIDEV:
        print("(dry-run) spidev indisponível; nenhum byte foi enviado")
        return
    client.open()
    try:
        duplex = client.transfer(frame)
        if args.show_duplex:
            print(f"RX simultâneo[{len(duplex)}]: {bytes(duplex).hex()}")
        if args.skip_response:
            return
        resp = client.read_response(
            max_len=args.max_read, timeout=args.timeout, chunk_size=args.poll_chunk
        )
        if resp is None:
            print(f"Nenhuma resposta em {args.timeout:.3f}s")
            return
        print(f"RESP[{len(resp)}]: {resp.hex()}")
        try:
            decoded = decode_response(resp)
        except ValueError as exc:
            print(f"Falha ao decodificar: {exc}")
        else:
            print(f"→ {_format_decoded(decoded)}")
    finally:
        client.close()


def _cmd_listen(args: argparse.Namespace) -> None:
    if args.dry_run and not HAVE_SPIDEV:
        print("listen em dry-run não consegue ler da SPI")
        return
    client = SpiClient(
        bus=args.bus,
        device=args.device,
        max_speed_hz=args.max_speed,
        mode=args.mode,
        bits_per_word=args.bits_per_word,
        poll_delay=args.poll_delay,
        dry_run=args.dry_run,
    )
    client.open()
    try:
        remaining = args.count
        deadline = None if args.duration is None else time.monotonic() + args.duration
        while remaining is None or remaining > 0:
            if deadline is not None and time.monotonic() >= deadline:
                break
            resp = client.read_response(
                max_len=args.max_read, timeout=args.timeout, chunk_size=args.poll_chunk
            )
            if resp is None:
                continue
            print(f"RESP[{len(resp)}]: {resp.hex()}")
            try:
                decoded = decode_response(resp)
            except ValueError as exc:
                print(f"  ! Falha ao decodificar: {exc}")
            else:
                print(f"  → {_format_decoded(decoded)}")
            if remaining is not None:
                remaining -= 1
    finally:
        client.close()


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bus", type=int, default=0, help="Número do bus SPI (default: 0)")
    parser.add_argument(
        "--device", type=int, default=0, help="Dispositivo/chip-select (default: 0)"
    )
    parser.add_argument(
        "--max-speed",
        type=int,
        default=2_000_000,
        help="Frequência máxima de SCK em Hz (default: 2 MHz)",
    )
    parser.add_argument(
        "--mode",
        type=lambda v: int(v, 0),
        default=0b11,
        help="Modo SPI (0..3). MODE 3 (CPOL=1, CPHA=1) por padrão.",
    )
    parser.add_argument(
        "--bits-per-word",
        type=int,
        default=8,
        help="Número de bits por palavra (default: 8)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=0.05,
        help="Tempo máximo (s) esperando pela resposta após o request (default: 0.05)",
    )
    parser.add_argument(
        "--max-read",
        type=int,
        default=64,
        help="Tamanho máximo da resposta lida (default: 64 bytes)",
    )
    parser.add_argument(
        "--poll-chunk",
        type=int,
        default=16,
        help="Quantidade de bytes fictícios enviados para puxar a resposta (default: 16)",
    )
    parser.add_argument(
        "--poll-delay",
        type=float,
        default=0.001,
        help="Atraso (s) entre tentativas de leitura quando a resposta ainda não chegou",
    )
    parser.add_argument(
        "--show-duplex",
        action="store_true",
        help="Imprime os bytes recebidos durante o envio do request (full-duplex)",
    )
    parser.add_argument(
        "--skip-response",
        action="store_true",
        help="Apenas envia o request, sem tentar ler/decodificar resposta",
    )
    parser.add_argument(
        "--dry-run",
        dest="dry_run",
        action="store_true",
        help="Não abre a SPI; apenas mostra os frames gerados",
    )
    parser.add_argument(
        "--no-dry-run",
        dest="dry_run",
        action="store_false",
        help=argparse.SUPPRESS,
    )
    parser.set_defaults(dry_run=not HAVE_SPIDEV)

    sub = parser.add_subparsers(dest="command", required=True)

    p_led = sub.add_parser("led", help="Envia REQ_LED_CTRL (liga/desliga LED)")
    p_led.add_argument("--frame-id", type=_parse_int, default=0, help="FrameId do request")
    p_led.add_argument("--mask", type=_parse_int, required=True, help="Máscara de LEDs")
    p_led.add_argument("--r", type=_parse_int, default=0, help="Canal R ou valor ON/OFF")
    p_led.add_argument("--g", type=_parse_int, default=0, help="Canal G")
    p_led.add_argument("--b", type=_parse_int, default=0, help="Canal B")

    p_qstatus = sub.add_parser("queue-status", help="Envia REQ_MOVE_QUEUE_STATUS")
    p_qstatus.add_argument("--frame-id", type=_parse_int, default=0)

    p_start = sub.add_parser("start-move", help="Envia REQ_START_MOVE")
    p_start.add_argument("--frame-id", type=_parse_int, default=0)

    p_end = sub.add_parser("move-end", help="Envia REQ_MOVE_END")
    p_end.add_argument("--frame-id", type=_parse_int, default=0)

    p_home = sub.add_parser("move-home", help="Envia REQ_MOVE_HOME")
    p_home.add_argument("--frame-id", type=_parse_int, default=0)
    p_home.add_argument("--axis-mask", type=_parse_int, required=True)
    p_home.add_argument("--dir-mask", type=_parse_int, required=True)
    p_home.add_argument("--vhome", type=_parse_int, required=True, help="Velocidade de homing")

    p_probe = sub.add_parser("move-probe", help="Envia REQ_MOVE_PROBE_LEVEL")
    p_probe.add_argument("--frame-id", type=_parse_int, default=0)
    p_probe.add_argument("--axis-mask", type=_parse_int, required=True)
    p_probe.add_argument("--vprobe", type=_parse_int, required=True, help="Velocidade de probe")

    p_queue_add = sub.add_parser("queue-add", help="Envia REQ_MOVE_QUEUE_ADD")
    p_queue_add.add_argument("--frame-id", type=_parse_int, default=0)
    p_queue_add.add_argument("--dir-mask", type=_parse_int, required=True)
    p_queue_add.add_argument("--vx", type=_parse_int, required=True)
    p_queue_add.add_argument("--sx", type=_parse_int, required=True)
    p_queue_add.add_argument("--vy", type=_parse_int, required=True)
    p_queue_add.add_argument("--sy", type=_parse_int, required=True)
    p_queue_add.add_argument("--vz", type=_parse_int, required=True)
    p_queue_add.add_argument("--sz", type=_parse_int, required=True)
    p_queue_add.add_argument("--kp-x", type=_parse_int, default=0)
    p_queue_add.add_argument("--ki-x", type=_parse_int, default=0)
    p_queue_add.add_argument("--kd-x", type=_parse_int, default=0)
    p_queue_add.add_argument("--kp-y", type=_parse_int, default=0)
    p_queue_add.add_argument("--ki-y", type=_parse_int, default=0)
    p_queue_add.add_argument("--kd-y", type=_parse_int, default=0)
    p_queue_add.add_argument("--kp-z", type=_parse_int, default=0)
    p_queue_add.add_argument("--ki-z", type=_parse_int, default=0)
    p_queue_add.add_argument("--kd-z", type=_parse_int, default=0)

    p_fpga = sub.add_parser("fpga-status", help="Envia REQ_FPGA_STATUS")
    p_fpga.add_argument("--frame-id", type=_parse_int, default=0)

    p_raw = sub.add_parser(
        "raw",
        help="Envia bytes arbitrários (hex) já no framing AA..55",
    )
    p_raw.add_argument("data", help="Sequência hex (ex.: aa0701000101ff55)")

    p_listen = sub.add_parser("listen", help="Apenas coleta respostas pendentes")
    p_listen.add_argument("--count", type=int, default=None, help="Número de frames a capturar")
    p_listen.add_argument(
        "--duration", type=float, default=None, help="Tempo máximo de escuta (s)"
    )

    return parser


def _dispatch(args: argparse.Namespace) -> None:
    cmd = args.command
    if cmd == "led":
        frame = build_led_ctrl(args.frame_id, args.mask, args.r, args.g, args.b)
        _send_and_maybe_read(args, frame)
    elif cmd == "queue-status":
        frame = build_move_queue_status(args.frame_id)
        _send_and_maybe_read(args, frame)
    elif cmd == "start-move":
        frame = build_start_move(args.frame_id)
        _send_and_maybe_read(args, frame)
    elif cmd == "move-end":
        frame = build_move_end(args.frame_id)
        _send_and_maybe_read(args, frame)
    elif cmd == "move-home":
        frame = build_move_home(args.frame_id, args.axis_mask, args.dir_mask, args.vhome)
        _send_and_maybe_read(args, frame)
    elif cmd == "move-probe":
        frame = build_move_probe_level(args.frame_id, args.axis_mask, args.vprobe)
        _send_and_maybe_read(args, frame)
    elif cmd == "queue-add":
        frame = build_move_queue_add(
            args.frame_id,
            args.dir_mask,
            args.vx,
            args.sx,
            args.vy,
            args.sy,
            args.vz,
            args.sz,
            args.kp_x,
            args.ki_x,
            args.kd_x,
            args.kp_y,
            args.ki_y,
            args.kd_y,
            args.kp_z,
            args.ki_z,
            args.kd_z,
        )
        _send_and_maybe_read(args, frame)
    elif cmd == "fpga-status":
        frame = build_fpga_status(args.frame_id)
        _send_and_maybe_read(args, frame)
    elif cmd == "raw":
        data = args.data.replace(" ", "")
        if len(data) % 2:
            raise ValueError("Número de dígitos hex precisa ser par")
        frame = bytes.fromhex(data)
        _send_and_maybe_read(args, frame)
    elif cmd == "listen":
        _cmd_listen(args)
    else:  # pragma: no cover - argparse garante comandos válidos
        raise RuntimeError(f"Comando desconhecido: {cmd}")


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    try:
        _dispatch(args)
    except (ValueError, SpiError) as exc:
        print(f"Erro: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

