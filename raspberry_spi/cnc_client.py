"""Cliente SPI que conversa com o firmware CNC no STM32."""

import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Tuple

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_protocol import (
        REQ_HEADER,
        REQ_TAIL,
        RESP_HEADER,
        RESP_TAIL,
        RESP_TEST_HELLO,
        SPI_DMA_FRAME_LEN,
        SPI_DMA_HANDSHAKE_BUSY,
        SPI_DMA_HANDSHAKE_BYTES,
        SPI_DMA_HANDSHAKE_READY,
        SPI_DMA_MAX_PAYLOAD,
        handshake_status_label,
        bits_str,
    )
    from .cnc_responses import CNCResponseDecoder
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_protocol import (  # type: ignore
        REQ_HEADER,
        REQ_TAIL,
        RESP_HEADER,
        RESP_TAIL,
        RESP_TEST_HELLO,
        SPI_DMA_FRAME_LEN,
        SPI_DMA_HANDSHAKE_BUSY,
        SPI_DMA_HANDSHAKE_BYTES,
        SPI_DMA_HANDSHAKE_READY,
        SPI_DMA_MAX_PAYLOAD,
        handshake_status_label,
        bits_str,
    )
    from cnc_responses import CNCResponseDecoder  # type: ignore

try:  # pragma: no cover - dependência externa
    import spidev  # type: ignore
except Exception as exc:  # pragma: no cover
    spidev = None


def _build_spi_dma_frame(payload: List[int]) -> List[int]:
    if len(payload) > SPI_DMA_MAX_PAYLOAD:
        raise ValueError(f"payload excede {SPI_DMA_MAX_PAYLOAD} bytes: {len(payload)}")
    frame = [0x00] * SPI_DMA_FRAME_LEN
    start = SPI_DMA_FRAME_LEN - len(payload)
    for idx, byte in enumerate(payload):
        frame[start + idx] = byte & 0xFF
    return frame


def _validate_handshake_frame(
    tx_frame: List[int], handshake_frame: List[int], payload_len: int
) -> None:
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

    for idx, (tx_byte, status) in enumerate(zip(tx_frame, handshake_frame)):
        status &= 0xFF
        if status == SPI_DMA_HANDSHAKE_READY:
            continue

        tx_byte &= 0xFF
        if idx >= prefix_len:
            payload_idx = idx - prefix_len
            location = f"payload[{payload_idx}] (0x{tx_byte:02X})"
        else:
            location = f"preenchimento[{idx}] (0x{tx_byte:02X})"

        label = handshake_status_label(status)
        label_suffix = f" ({label})" if label and label != "desconhecido" else ""
        base_msg = (
            f"STM32 sinalizou erro de handshake no byte {idx} ({location}) "
            f"com código 0x{status:02X}{label_suffix}."
        )
        if status == SPI_DMA_HANDSHAKE_BUSY:
            raise BufferError(base_msg + " Aguarde e tente novamente.")
        raise RuntimeError(base_msg)


class CNCClient:
    def __init__(self, bus: int = 0, dev: int = 0,
                 speed_hz: int = 1_000_000, mode: int = 0b11) -> None:
        if spidev is None:
            raise RuntimeError("spidev não disponível. Instale `python3-spidev` no Raspberry.")
        self.spi = spidev.SpiDev()
        self.spi.open(bus, dev)
        self.spi.max_speed_hz = int(speed_hz)
        self.spi.mode = mode  # MODE 3: 0b11
        self.spi.bits_per_word = 8

    def close(self) -> None:
        try:
            self.spi.close()
        except Exception:  # pragma: no cover - limpeza defensiva
            pass

    def _xfer(self, data: List[int]) -> List[int]:
        tx = [d & 0xFF for d in data]
        try:
            print("SPI TX bits:", bits_str(tx))
        except Exception:
            pass
        rx = self.spi.xfer2(tx)
        try:
            print("SPI RX bits:", bits_str(rx))
        except Exception:
            pass
        return rx

    def exchange(self, request_type: int, request: List[int],
                 tries: int = 8, settle_delay_s: float = 0.001) -> List[int]:
        spec = CNCResponseDecoder.SPECS[request_type]
        dma_frame = _build_spi_dma_frame(request)
        rx_frame = self._xfer(dma_frame)
        _validate_handshake_frame(dma_frame, rx_frame, len(request))
        time.sleep(settle_delay_s)

        for _ in range(max(1, tries)):
            rx = self._xfer([0x00] * spec.length)
            try:
                idx = rx.index(RESP_HEADER)
            except ValueError:
                time.sleep(settle_delay_s)
                continue
            if idx + spec.length <= len(rx):
                frame = rx[idx:idx + spec.length]
                if frame[0] == RESP_HEADER and frame[-1] == RESP_TAIL and frame[1] == spec.response_type:
                    return frame
            time.sleep(settle_delay_s)
        raise TimeoutError("Resposta SPI não recebida/validada no prazo.")

    @staticmethod
    def _build_boot_poll_frame(chunk_len: int) -> List[int]:
        if chunk_len <= 0:
            return []
        frame = [0x00] * chunk_len
        if chunk_len > SPI_DMA_HANDSHAKE_BYTES:
            header_idx = SPI_DMA_HANDSHAKE_BYTES
            frame[header_idx] = REQ_HEADER
            tail_idx = chunk_len - 1
            if tail_idx > header_idx:
                frame[tail_idx] = REQ_TAIL
        return frame

    def _read_boot_token_info(self, token_bytes: bytes, tries: int, settle_delay_s: float,
                              chunk_len: int) -> Tuple[List[int], Dict[str, Any]]:
        if chunk_len <= 0:
            raise ValueError("chunk_len deve ser positivo")
        token_list = [b & 0xFF for b in token_bytes]
        expected = [RESP_HEADER] + token_list + [RESP_TAIL]
        expected_len = len(expected)
        accum: List[int] = []
        base_offset = 0
        chunks: List[List[int]] = []
        reads_used = 0
        poll_frame = self._build_boot_poll_frame(chunk_len)
        for _ in range(max(1, tries)):
            chunk = self._xfer(poll_frame)
            reads_used += 1
            chunks.append(chunk)
            accum.extend(chunk)
            i = 0
            while i + expected_len <= len(accum):
                window = accum[i:i + expected_len]
                if (window[0] == RESP_HEADER and window[-1] == RESP_TAIL
                        and window[1:-1] == token_list):
                    bytes_before_header = base_offset + i
                    bytes_until_tail = base_offset + i + expected_len
                    frame_list = window[:]
                    handshake_start = max(0, i - SPI_DMA_HANDSHAKE_BYTES)
                    handshake_bytes = accum[handshake_start:i]
                    stats = {
                        "bytesBeforeHeader": int(bytes_before_header),
                        "bytesUntilTail": int(bytes_until_tail),
                        "readsUsed": int(reads_used),
                        "chunkLen": int(chunk_len),
                        "chunks": chunks,
                        "expected": expected[:],
                        "handshakeBytes": [b & 0xFF for b in handshake_bytes],
                    }
                    return frame_list, stats
                i = i + 1

            if settle_delay_s > 0:
                time.sleep(settle_delay_s)
            max_keep = (4 * max(1, chunk_len)) + (2 * len(expected))
            if len(accum) > max_keep:
                drop = len(accum) - max_keep
                del accum[:drop]
                base_offset += drop
        token_label = token_bytes.decode("ascii", errors="replace")
        raise TimeoutError(f"Frame '{token_label}' nao encontrado. Reinicie o STM32 e tente novamente.")

    def read_boot_hello(self, tries: int = 16, settle_delay_s: float = 0.002,
                        chunk_len: int = 7) -> List[int]:
        frame, _stats = self.read_boot_hello_info(tries=tries, settle_delay_s=settle_delay_s,
                                                  chunk_len=chunk_len)
        return frame

    def read_boot_hello_info(self, tries: int = 16, settle_delay_s: float = 0.002,
                             chunk_len: int = 7) -> Tuple[List[int], Dict[str, Any]]:
        payload = bytes([RESP_TEST_HELLO]) + b"ello"
        return self._read_boot_token_info(payload, tries, settle_delay_s, chunk_len)

    def read_boot_led(self, tries: int = 16, settle_delay_s: float = 0.002,
                      chunk_len: int = 7) -> List[int]:
        frame, _stats = self.read_boot_led_info(tries=tries, settle_delay_s=settle_delay_s,
                                                chunk_len=chunk_len)
        return frame

    def read_boot_led_info(self, tries: int = 16, settle_delay_s: float = 0.002,
                           chunk_len: int = 7) -> Tuple[List[int], Dict[str, Any]]:
        return self._read_boot_token_info(b"led", tries, settle_delay_s, chunk_len)

    def print_until_zero_after_activity(self, chunk_len: int = 32,
                                        settle_delay_s: float = 0.0) -> None:
        saw_activity = False
        while True:
            rx = self._xfer([0x00] * chunk_len)
            print(" ".join(f"{b:02X}" for b in rx))
            if any(b != 0x00 for b in rx):
                saw_activity = True
            if saw_activity and any(b == 0x00 for b in rx):
                break
            if settle_delay_s > 0:
                time.sleep(settle_delay_s)


__all__ = ["CNCClient"]
