"""Utilitários de acesso SPI ao driver TMC5160."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence

try:  # pragma: no cover - dependência externa
    import spidev  # type: ignore
except Exception:  # pragma: no cover - testes substituem via monkeypatch
    spidev = None  # type: ignore


TMC5160_GSTAT = 0x01
TMC5160_DRV_STATUS = 0x6F
_TMC5160_ADDR_MASK = 0x7F
_TMC5160_READ_BIT = 0x80
_TMC5160_FRAME_BYTES = 5


@dataclass
class TMC5160Reply:
    """Resposta SPI genérica do TMC5160."""

    status: int
    value: int


@dataclass
class TMC5160DrvStatus:
    """Decodificação dos principais campos do DRV_STATUS."""

    status_byte: int
    raw: int
    sg_result: int
    fsactive: bool
    s2ga: bool
    s2gb: bool
    s2vsa: bool
    s2vsb: bool
    ola: bool
    olb: bool
    stallguard: bool
    ot: bool
    otpw: bool
    driver_error: bool
    uv_cp: bool


class TMC5160Spi:
    """Wrapper simples para comandos básicos do TMC5160 via SPI."""

    def __init__(self, bus: int = 0, dev: int = 0, *, speed_hz: int = 1_000_000) -> None:
        if spidev is None:  # pragma: no cover - ambiente sem spidev
            raise RuntimeError(
                "spidev não disponível. Instale `python3-spidev` para acessar o TMC5160."
            )

        self._spi = spidev.SpiDev()
        self._spi.open(bus, dev)
        self._spi.max_speed_hz = int(speed_hz)
        self._spi.mode = 0b11  # MODE 3 (CPOL=1, CPHA=2nd)
        self._spi.bits_per_word = 8

    def close(self) -> None:
        try:
            self._spi.close()
        except Exception:  # pragma: no cover - encerramento defensivo
            pass

    def __enter__(self) -> "TMC5160Spi":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:  # pragma: no cover - sugar
        self.close()

    def _xfer(self, frame: Sequence[int]) -> List[int]:
        if len(frame) != _TMC5160_FRAME_BYTES:
            raise ValueError("Quadro SPI do TMC5160 deve conter 5 bytes")
        data = [b & 0xFF for b in frame]
        return list(self._spi.xfer2(data))

    def flush_pipeline(self) -> None:
        """Descarta dados pendentes na pipeline de leitura."""

        self._xfer([0x00] * _TMC5160_FRAME_BYTES)

    @staticmethod
    def _pack_value(value: int) -> List[int]:
        return [
            (value >> 24) & 0xFF,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF,
        ]

    @staticmethod
    def _unpack_value(raw: Iterable[int]) -> int:
        bytes_list = list(raw)
        if len(bytes_list) != 4:
            raise ValueError("Valor de registro deve conter 4 bytes")
        value = 0
        for byte in bytes_list:
            value = (value << 8) | (byte & 0xFF)
        return value & 0xFFFFFFFF

    def write_register(self, address: int, value: int) -> TMC5160Reply:
        if not 0 <= address <= _TMC5160_ADDR_MASK:
            raise ValueError("Endereço de registro inválido para o TMC5160")

        frame = [address & _TMC5160_ADDR_MASK] + self._pack_value(value)
        resp = self._xfer(frame)
        status = resp[0] & 0xFF
        prev_value = self._unpack_value(resp[1:])
        return TMC5160Reply(status=status, value=prev_value)

    def read_register(self, address: int) -> TMC5160Reply:
        if not 0 <= address <= _TMC5160_ADDR_MASK:
            raise ValueError("Endereço de registro inválido para o TMC5160")

        read_cmd = (address & _TMC5160_ADDR_MASK) | _TMC5160_READ_BIT
        self._xfer([read_cmd, 0x00, 0x00, 0x00, 0x00])
        resp = self._xfer([0x00] * _TMC5160_FRAME_BYTES)
        status = resp[0] & 0xFF
        value = self._unpack_value(resp[1:])
        return TMC5160Reply(status=status, value=value)

    def clear_gstat(self) -> TMC5160Reply:
        """Zera GSTAT (bits latched de erro)."""

        return self.write_register(TMC5160_GSTAT, 0x00000007)

    def read_drv_status(self) -> TMC5160DrvStatus:
        """Lê e decodifica o registro DRV_STATUS (0x6F)."""

        reply = self.read_register(TMC5160_DRV_STATUS)
        raw = reply.value
        return TMC5160DrvStatus(
            status_byte=reply.status,
            raw=raw,
            sg_result=(raw >> 24) & 0xFF,
            fsactive=bool((raw >> 23) & 0x1),
            s2ga=bool((raw >> 22) & 0x1),
            s2gb=bool((raw >> 21) & 0x1),
            s2vsa=bool((raw >> 20) & 0x1),
            s2vsb=bool((raw >> 19) & 0x1),
            ola=bool((raw >> 18) & 0x1),
            olb=bool((raw >> 17) & 0x1),
            stallguard=bool((raw >> 16) & 0x1),
            ot=bool((raw >> 15) & 0x1),
            otpw=bool((raw >> 14) & 0x1),
            driver_error=bool((raw >> 13) & 0x1),
            uv_cp=bool((raw >> 12) & 0x1),
        )


__all__ = [
    "TMC5160_GSTAT",
    "TMC5160_DRV_STATUS",
    "TMC5160Reply",
    "TMC5160DrvStatus",
    "TMC5160Spi",
]

