"""Constantes e utilitários comuns para o protocolo CNC SPI."""

from typing import List, Tuple

# Framing bytes
REQ_HEADER = 0xAA
REQ_TAIL = 0x55
RESP_HEADER = 0xAB
RESP_TAIL = 0x54

# Request types
REQ_MOVE_QUEUE_ADD = 0x01
REQ_MOVE_QUEUE_STATUS = 0x02
REQ_START_MOVE = 0x03
REQ_MOVE_HOME = 0x04
REQ_MOVE_PROBE_LEVEL = 0x05
REQ_MOVE_END = 0x06
REQ_LED_CTRL = 0x07
REQ_FPGA_STATUS = 0x20
REQ_TEST_HELLO = 0x68

# Response types
RESP_MOVE_QUEUE_ADD_ACK = 0x01
RESP_MOVE_QUEUE_STATUS = 0x02
RESP_START_MOVE = 0x03
RESP_MOVE_HOME = 0x04
RESP_MOVE_PROBE_LEVEL = 0x05
RESP_MOVE_END = 0x06
RESP_LED_CTRL = 0x07
RESP_FPGA_STATUS = 0x20
RESP_HOME_STATUS = 0x21
RESP_TEST_HELLO = 0x68

# SPI DMA framing (STM32 handshake + payload)
SPI_DMA_MAX_PAYLOAD = 42
SPI_DMA_HANDSHAKE_BYTES = 1
SPI_DMA_FRAME_LEN = SPI_DMA_HANDSHAKE_BYTES + SPI_DMA_MAX_PAYLOAD
SPI_DMA_HANDSHAKE_READY = 0xA5
SPI_DMA_HANDSHAKE_BUSY = 0x5A


def xor_reduce_bytes(bs: List[int]) -> int:
    x = 0
    for b in bs:
        x ^= (b & 0xFF)
    return x & 0xFF


def xor_bit_reduce_bytes(bs: List[int]) -> int:
    x = xor_reduce_bytes(bs)
    x ^= (x >> 4)
    x ^= (x >> 2)
    x ^= (x >> 1)
    return x & 0x1


def be16_bytes(v: int) -> Tuple[int, int]:
    return ((v >> 8) & 0xFF, v & 0xFF)


def be32_bytes(v: int) -> Tuple[int, int, int, int]:
    return (
        (v >> 24) & 0xFF,
        (v >> 16) & 0xFF,
        (v >> 8) & 0xFF,
        v & 0xFF,
    )


def parity_set_byte_1N(raw: List[int], last_index: int, parity_index: int) -> None:
    raw[parity_index] = xor_reduce_bytes(raw[1:last_index + 1])


def parity_check_byte_1N(raw: List[int], last_index: int, parity_index: int) -> bool:
    return (raw[parity_index] & 0xFF) == xor_reduce_bytes(raw[1:last_index + 1])


def parity_set_bit_1N(raw: List[int], last_index: int, parity_index: int) -> None:
    raw[parity_index] = xor_bit_reduce_bytes(raw[1:last_index + 1]) & 0x1


def parity_check_bit_1N(raw: List[int], last_index: int, parity_index: int) -> bool:
    return (raw[parity_index] & 0x1) == xor_bit_reduce_bytes(raw[1:last_index + 1])


def bits_str(bs: List[int]) -> str:
    return " ".join(f"{b:08b}" for b in bs)


def pad_request(raw: List[int], total_len: int = SPI_DMA_MAX_PAYLOAD) -> List[int]:
    """Valida o tamanho da requisição sem adicionar zeros à direita.

    O buffer DMA do STM32 opera com ``SPI_DMA_MAX_PAYLOAD`` bytes, porém os
    frames efetivos podem ser menores. O empacotamento final insere zeros antes
    do header automaticamente (vide ``_build_spi_dma_frame``). Portanto basta
    garantir que a mensagem não exceda o limite e devolver uma cópia segura.
    """

    if not raw:
        raise ValueError("Request vazia")
    if total_len < len(raw):
        raise ValueError(f"Request excede {total_len} bytes: {len(raw)}")
    return raw[:]


__all__ = [
    "REQ_HEADER",
    "REQ_TAIL",
    "RESP_HEADER",
    "RESP_TAIL",
    "REQ_MOVE_QUEUE_ADD",
    "REQ_MOVE_QUEUE_STATUS",
    "REQ_START_MOVE",
    "REQ_MOVE_HOME",
    "REQ_MOVE_PROBE_LEVEL",
    "REQ_MOVE_END",
    "REQ_LED_CTRL",
    "REQ_FPGA_STATUS",
    "REQ_TEST_HELLO",
    "RESP_MOVE_QUEUE_ADD_ACK",
    "RESP_MOVE_QUEUE_STATUS",
    "RESP_START_MOVE",
    "RESP_MOVE_HOME",
    "RESP_MOVE_PROBE_LEVEL",
    "RESP_MOVE_END",
    "RESP_LED_CTRL",
    "RESP_FPGA_STATUS",
    "RESP_HOME_STATUS",
    "RESP_TEST_HELLO",
    "SPI_DMA_MAX_PAYLOAD",
    "SPI_DMA_HANDSHAKE_BYTES",
    "SPI_DMA_FRAME_LEN",
    "SPI_DMA_HANDSHAKE_READY",
    "SPI_DMA_HANDSHAKE_BUSY",
    "xor_reduce_bytes",
    "xor_bit_reduce_bytes",
    "be16_bytes",
    "be32_bytes",
    "parity_set_byte_1N",
    "parity_check_byte_1N",
    "parity_set_bit_1N",
    "parity_check_bit_1N",
    "bits_str",
    "pad_request",
]
