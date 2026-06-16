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
        SPI_DMA_HANDSHAKE_NO_COMM,
        SPI_DMA_HANDSHAKE_READY,
        SPI_DMA_CLIENT_POLL_BYTE,
        SPI_DMA_MAX_PAYLOAD,
        handshake_status_label,
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
        SPI_DMA_HANDSHAKE_NO_COMM,
        SPI_DMA_HANDSHAKE_READY,
        SPI_DMA_CLIENT_POLL_BYTE,
        SPI_DMA_MAX_PAYLOAD,
        handshake_status_label,
    )
    from cnc_responses import CNCResponseDecoder  # type: ignore

try:  # pragma: no cover - dependência externa
    import spidev  # type: ignore
except Exception as exc:  # pragma: no cover
    spidev = None


def _build_spi_dma_frame(payload: List[int], filler: int = 0x00) -> List[int]:
    if len(payload) > SPI_DMA_MAX_PAYLOAD:
        raise ValueError(f"payload excede {SPI_DMA_MAX_PAYLOAD} bytes: {len(payload)}")
    frame = [filler & 0xFF] * SPI_DMA_FRAME_LEN
    start = SPI_DMA_FRAME_LEN - len(payload)
    for idx, byte in enumerate(payload):
        frame[start + idx] = byte & 0xFF
    return frame


# O handshake do STM32 agora reutiliza o mesmo buffer DMA de 42 bytes para
# publicar respostas prontas. Isso significa que, dependendo do momento em que
# o Raspberry fizer a leitura, o quadro de "handshake" pode conter apenas os
# bytes preenchidos com ``0x00`` ou até mesmo a resposta completa já alinhada
# à direita com zeros à esquerda. O validador abaixo trata esse cenário como
# sucesso para evitar falsos positivos de erro de comunicação.
def _validate_handshake_frame(
    tx_frame: List[int], handshake_frame: List[int], payload_len: int
) -> None:
    if payload_len <= 0:
        raise ValueError("payload_len deve ser positivo")
    if len(tx_frame) != SPI_DMA_FRAME_LEN:
        raise ValueError(
            "tx_frame deve ocupar exatamente o frame DMA de "
            f"{SPI_DMA_FRAME_LEN} bytes"
        )
    if len(tx_frame) != len(handshake_frame):
        raise ValueError(
            "Comprimento do handshake difere do frame transmitido: "
            f"tx={len(tx_frame)} rx={len(handshake_frame)}"
        )
    if payload_len > SPI_DMA_FRAME_LEN:
        raise ValueError(
            "payload_len maior que o frame DMA: "
            f"{payload_len} > {SPI_DMA_FRAME_LEN}"
        )

    prefix_len = len(tx_frame) - payload_len
    if prefix_len < 0:
        raise ValueError("payload_len maior que o frame transmitido")

    normalized_statuses = [status & 0xFF for status in handshake_frame]
    if not normalized_statuses:
        raise ValueError("handshake_frame vazio")

    if set(normalized_statuses) == {SPI_DMA_HANDSHAKE_READY}:
        return

    if {
        SPI_DMA_HANDSHAKE_READY,
        SPI_DMA_HANDSHAKE_NO_COMM,
    }.issuperset(set(normalized_statuses)) and any(
        status == SPI_DMA_HANDSHAKE_READY for status in normalized_statuses
    ):
        # Alguns firmwares podem atualizar o eco do handshake de maneira
        # preguiçosa, deixando ``0x00`` (NO_COMM) nos primeiros bytes do
        # payload até que o laço principal termine de copiar todo o buffer
        # DMA. Desde que pelo menos um ``0xA5`` (READY) tenha sido observado,
        # tratamos o quadro como handshake válido para continuar com o
        # polling.
        return

    if set(normalized_statuses) == {SPI_DMA_HANDSHAKE_BUSY}:
        raise BufferError(
            "STM32 respondeu BUSY (0x5A) para todo o frame DMA de "
            f"{SPI_DMA_FRAME_LEN} bytes. Aguarde e tente novamente."
        )

    if set(normalized_statuses) == {SPI_DMA_HANDSHAKE_NO_COMM}:
        # O firmware passou a reutilizar o mesmo quadro de handshake para
        # publicar respostas pendentes, o que significa que nem sempre haverá
        # tempo hábil para preencher cada byte com READY/BUSY antes da leitura
        # do Raspberry Pi. Quando todo o quadro vem zerado, não tratamos mais
        # como falha de comunicação: assumimos que o STM32 apenas ainda não
        # escreveu o eco do handshake neste ciclo e seguimos para o polling.
        return

    # Caso os bytes recebidos formem uma resposta válida (ex.: ``AB ... 54``)
    # alinhada à direita com zeros à esquerda, tratamos o quadro como um
    # handshake bem-sucedido que já traz a carga útil aguardada.
    header_idx = -1
    tail_idx = -1
    try:
        header_idx = normalized_statuses.index(RESP_HEADER)
        tail_idx = normalized_statuses.index(RESP_TAIL, header_idx + 1)
    except ValueError:
        header_idx = -1
        tail_idx = -1

    if header_idx >= 0 and tail_idx > header_idx:
        prefix = normalized_statuses[:header_idx]
        allowed_prefix_statuses = {
            SPI_DMA_HANDSHAKE_READY,
            SPI_DMA_HANDSHAKE_BUSY,
            SPI_DMA_HANDSHAKE_NO_COMM,
            0x00,
        }
        unexpected = [
            (idx, status)
            for idx, status in enumerate(prefix)
            if status not in allowed_prefix_statuses
        ]
        if unexpected:
            idx, status = unexpected[0]
            label = handshake_status_label(status)
            label_suffix = f" ({label})" if label and label != "desconhecido" else ""
            raise RuntimeError(
                "STM32 retornou byte inesperado antes do header 0x"
                f"{RESP_HEADER:02X} durante o handshake inicial (offset {idx}, "
                f"valor 0x{status:02X}{label_suffix})."
            )

        if prefix and all(status == SPI_DMA_HANDSHAKE_BUSY for status in prefix):
            raise BufferError(
                "STM32 sinalizou BUSY (0x5A) antes do header 0x"
                f"{RESP_HEADER:02X} durante o handshake inicial."
            )

        # Permite zeros (padding) e quaisquer bytes da resposta propriamente
        # dita; o polling subsequente irá confirmar o conteúdo detalhadamente.
        return

    for idx, (tx_byte, status) in enumerate(zip(tx_frame, normalized_statuses)):
        status &= 0xFF
        if status == SPI_DMA_HANDSHAKE_READY:
            continue

        if idx < prefix_len and status == SPI_DMA_HANDSHAKE_NO_COMM and tx_byte == 0:
            # ``0x00`` no preenchimento indica apenas que o STM32 ainda não
            # escreveu o eco do handshake naquele byte (padding).
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
        if status == SPI_DMA_HANDSHAKE_NO_COMM:
            raise ConnectionError(
                base_msg
                + " Comunicação SPI não ocorreu (verifique alimentação, conexões e configuração)."
            )
        raise RuntimeError(base_msg)


def _extract_response_frame(
    rx_frame: List[int], expected_len: int, expected_type: int
) -> List[int] | None:
    if expected_len <= 0:
        raise ValueError("expected_len deve ser positivo")
    if not rx_frame:
        return None

    normalized = [byte & 0xFF for byte in rx_frame]

    try:
        header_idx = normalized.index(RESP_HEADER)
    except ValueError:
        if SPI_DMA_HANDSHAKE_BUSY in normalized:
            raise BufferError(
                "STM32 respondeu BUSY (0x5A) durante o polling da resposta. "
                "Aguarde antes de tentar ler novamente."
            )
        return None

    busy_before = [idx for idx, val in enumerate(normalized[:header_idx]) if val == SPI_DMA_HANDSHAKE_BUSY]
    if busy_before:
        raise BufferError(
            "STM32 sinalizou BUSY (0x5A) antes do header 0x"
            f"{RESP_HEADER:02X} durante o polling da resposta (byte {busy_before[0]})."
        )

    end_idx = header_idx + expected_len
    if end_idx > len(normalized):
        return None

    busy_after = [
        idx for idx, val in enumerate(normalized[end_idx:], start=end_idx)
        if val == SPI_DMA_HANDSHAKE_BUSY
    ]
    if busy_after:
        raise BufferError(
            "STM32 sinalizou BUSY (0x5A) após o tail 0x"
            f"{RESP_TAIL:02X} durante o polling da resposta (byte {busy_after[0]})."
        )

    frame = normalized[header_idx:end_idx]
    if (
        len(frame) == expected_len
        and frame[0] == RESP_HEADER
        and frame[-1] == RESP_TAIL
        and frame[1] == expected_type
    ):
        return frame

    return None


class CNCClient:
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

        normalized_format = (log_format or "hex").strip().lower()
        if normalized_format not in {"hex", "bin"}:
            raise ValueError(
                "log_format inválido. Utilize 'hex' ou 'bin' (padrão: 'hex')."
            )
        self._log_format = normalized_format

    def close(self) -> None:
        try:
            self.spi.close()
        except Exception:  # pragma: no cover - limpeza defensiva
            pass

    def _format_bytes(self, data: List[int]) -> str:
        if self._log_format == "hex":
            return " ".join(f"{d & 0xFF:02X}" for d in data)
        return " ".join(f"{d & 0xFF:08b}" for d in data)

    def _xfer(self, data: List[int]) -> List[int]:
        tx = [d & 0xFF for d in data]
        try:
            print("SPI TX bits:", self._format_bytes(tx))
        except Exception:
            pass
        rx = self.spi.xfer2(tx)
        try:
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
        spec = CNCResponseDecoder.SPECS[request_type]
        dma_frame = _build_spi_dma_frame(request)
        rx_frame = self._xfer(dma_frame)
        _validate_handshake_frame(dma_frame, rx_frame, len(request))
        handshake_response = _extract_response_frame(
            rx_frame, spec.length, spec.response_type
        )
        if handshake_response is not None:
            return handshake_response
        if settle_delay_s > 0:
            time.sleep(settle_delay_s)

        if poll_byte is None:
            raise TimeoutError(
                "Polling desabilitado e resposta não estava presente no handshake."
            )

        poll_payload_len = max(1, len(request))
        # ``poll_byte`` (padrão 0x3C) é o byte acordado com o firmware para
        # clockar a resposta sem simular um novo header 0xAA. Ele pode ser
        # ajustado via CLI conforme necessário.
        poll_frame = _build_spi_dma_frame(
            [poll_byte] * poll_payload_len,
            filler=poll_byte,
        )
        attempts = max(1, tries)
        for _ in range(attempts):
            rx = self._xfer(poll_frame)
            frame = _extract_response_frame(rx, spec.length, spec.response_type)
            if frame is not None:
                return frame
            if settle_delay_s > 0:
                time.sleep(settle_delay_s)
        raise TimeoutError("Resposta SPI nao recebida/validada no prazo.")


    @staticmethod
    def _build_boot_poll_frame(chunk_len: int) -> List[int]:
        if chunk_len <= 0:
            return []
        # O polling de boot também reutiliza o byte combinado para evitar que
        # leituras extras imitem um novo header de request.
        frame = [SPI_DMA_CLIENT_POLL_BYTE] * chunk_len
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
            rx = self._xfer([SPI_DMA_CLIENT_POLL_BYTE] * chunk_len)
            print(" ".join(f"{b:02X}" for b in rx))
            if any(b != SPI_DMA_CLIENT_POLL_BYTE for b in rx):
                saw_activity = True
            if saw_activity and all(b == SPI_DMA_CLIENT_POLL_BYTE for b in rx):
                break
            if settle_delay_s > 0:
                time.sleep(settle_delay_s)


__all__ = ["CNCClient"]
