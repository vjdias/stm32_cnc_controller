"""Constantes e utilitários comuns para o protocolo CNC SPI."""

from typing import List, Tuple

# Framing bytes
REQ_HEADER = 0xAA
REQ_TAIL = 0x55
RESP_HEADER = 0xAB
RESP_TAIL = 0x54

# Request types
#
# FUTURO: para aceitar PID em ponto flutuante (ex.: 4 casas decimais) sem
# quebrar compatibilidade, pode-se definir um novo request MOVE_QUEUE_ADD_F
# com campos de 32 bits (inteiros com escala 10^4 ou IEEE-754). Tomar como
# referência a decisão do LED (centi-Hz) para documentar a escala escolhida.
REQ_MOVE_QUEUE_ADD = 0x01
REQ_MOVE_QUEUE_STATUS = 0x02
REQ_START_MOVE = 0x03
REQ_MOVE_HOME = 0x04
REQ_MOVE_PROBE_LEVEL = 0x05
REQ_MOVE_END = 0x06
REQ_LED_CTRL = 0x07
REQ_STM32_STATUS = 0x20
REQ_TEST_HELLO = 0x68
REQ_MOTION_AUTO_FRICTION = 0x69

# Extended control/status (host additions; firmware support required)
REQ_SET_ORIGIN = 0x24
REQ_ENCODER_STATUS = 0x25
REQ_SET_MICROSTEPS = 0x26
REQ_SET_MICROSTEPS_AX = 0x27

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
RESP_MOTION_AUTO_FRICTION = 0x69
RESP_SET_ORIGIN = 0x24
RESP_ENCODER_STATUS = 0x25
RESP_SET_MICROSTEPS = 0x26

# SPI DMA framing (STM32 handshake + payload)
SPI_DMA_MAX_PAYLOAD = 42
SPI_DMA_HANDSHAKE_BYTES = 0
SPI_DMA_FRAME_LEN = SPI_DMA_MAX_PAYLOAD
SPI_DMA_HANDSHAKE_READY = 0xA5
SPI_DMA_HANDSHAKE_BUSY = 0x5A
SPI_DMA_HANDSHAKE_NO_COMM = 0x00
SPI_DMA_CLIENT_POLL_BYTE = 0x3C

# Handshake interpretation helpers (per-byte status echo from STM32)
SPI_DMA_HANDSHAKE_STATUS_LABELS = {
    SPI_DMA_HANDSHAKE_READY: "ok",
    SPI_DMA_HANDSHAKE_BUSY: "fila cheia/busy",
    SPI_DMA_HANDSHAKE_NO_COMM: "sem comunicação",
}


def handshake_status_label(code: int) -> str:
    """Retorna uma descrição curta para o status de handshake informado."""

    return SPI_DMA_HANDSHAKE_STATUS_LABELS.get(code & 0xFF, "desconhecido")


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
    "REQ_STM32_STATUS",
    "REQ_TEST_HELLO",
    "REQ_SET_ORIGIN",
    "REQ_ENCODER_STATUS",
    "REQ_SET_MICROSTEPS",
    "REQ_SET_MICROSTEPS_AX",
    "REQ_MOTION_AUTO_FRICTION",
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
    "RESP_MOTION_AUTO_FRICTION",
    "RESP_SET_ORIGIN",
    "RESP_ENCODER_STATUS",
    "SPI_DMA_MAX_PAYLOAD",
    "SPI_DMA_HANDSHAKE_BYTES",
    "SPI_DMA_FRAME_LEN",
    "SPI_DMA_HANDSHAKE_READY",
    "SPI_DMA_HANDSHAKE_BUSY",
    "SPI_DMA_HANDSHAKE_NO_COMM",
    "SPI_DMA_CLIENT_POLL_BYTE",
    "SPI_DMA_HANDSHAKE_STATUS_LABELS",
    "handshake_status_label",
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
