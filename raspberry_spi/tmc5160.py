"""Configuração de drivers TMC5160 a partir do Raspberry Pi."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

try:  # pragma: no cover - dependência externa
    import spidev  # type: ignore
except Exception:  # pragma: no cover - ambiente sem spidev
    spidev = None


# Endereços de registradores relevantes do TMC5160
REG_GCONF = 0x00
REG_GSTAT = 0x01
REG_IHOLD_IRUN = 0x10
REG_TPOWERDOWN = 0x11
REG_TPWMTHRS = 0x13
REG_CHOPCONF = 0x6C
REG_PWMCONF = 0x70


@dataclass(frozen=True)
class TMC5160RegisterPreset:
    """Coleção ordenada de escritas de registradores para preparar o driver."""

    writes: Tuple[Tuple[int, int], ...]

    @staticmethod
    def default() -> "TMC5160RegisterPreset":
        """Preset usado para habilitar STEP/DIR e corrente nominal."""

        return TMC5160RegisterPreset(
            writes=(
                # Limpa falhas latentes (UVLO, overtemp, short). Escrever 1 limpa o bit.
                (REG_GSTAT, 0x00000007),
                # Ativa o modo PWM de controle de corrente para operação Step/Dir.
                (REG_GCONF, 0x00000004),
                # Correntes: IHOLD=10/32 (~31%), IRUN=31/32 (100%), demora 6 * 2^n clock
                (REG_IHOLD_IRUN, 0x00061F0A),
                # Tempo até desligar bobina após parada (20 * 16 ciclos de clock interno)
                (REG_TPOWERDOWN, 0x00000014),
                # Limiar para modo StealthChop (velocidades abaixo de ~rpm) — 500 ticks
                (REG_TPWMTHRS, 0x000001F4),
                # Chopper configuration: toff=3, hstrt=5, hend=0, tbl=2, mres=16 µsteps
                (REG_CHOPCONF, 0x10410150),
                # PWM configuration: autoscale/autograd, freq=2, ofs=36, grad=14
                (REG_PWMCONF, 0xC10D0024),
            )
        )


class TMC5160Configurator:
    """Configura um driver TMC5160 ligado ao barramento SPI do Raspberry Pi."""

    def __init__(
        self,
        *,
        bus: int = 1,
        device: int = 1,
        speed_hz: int = 4_000_000,
        register_preset: TMC5160RegisterPreset | None = None,
        spi_device_factory=None,
    ) -> None:
        self._bus = bus
        self._device = device
        self._speed_hz = speed_hz
        self._registers = register_preset or TMC5160RegisterPreset.default()
        self._spi = None
        self._spi_device_factory = spi_device_factory

    def _ensure_spi(self):
        if self._spi is not None:
            return self._spi

        factory = self._spi_device_factory
        if factory is None:
            if spidev is None:  # pragma: no cover - ambiente sem hardware
                raise RuntimeError(
                    "Biblioteca spidev não disponível; instale-a no Raspberry Pi."
                )
            factory = spidev.SpiDev  # type: ignore

        spi_dev = factory()
        spi_dev.open(self._bus, self._device)
        spi_dev.max_speed_hz = self._speed_hz
        spi_dev.mode = 0b11  # MODE 3: CPOL=1, CPHA=1 (2nd edge)
        spi_dev.bits_per_word = 8
        if hasattr(spi_dev, "lsbfirst"):
            spi_dev.lsbfirst = False
        if hasattr(spi_dev, "cshigh"):
            spi_dev.cshigh = False
        self._spi = spi_dev
        return spi_dev

    def close(self) -> None:
        if self._spi is not None:
            try:
                close = getattr(self._spi, "close", None)
                if callable(close):
                    close()
            finally:
                self._spi = None

    def __enter__(self) -> "TMC5160Configurator":
        self._ensure_spi()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    @staticmethod
    def _build_frame(address: int, value: int) -> List[int]:
        if not 0 <= address <= 0x7F:
            raise ValueError("Endereço de registrador deve estar entre 0x00 e 0x7F")
        if not 0 <= value <= 0xFFFFFFFF:
            raise ValueError("Valor de registrador deve caber em 32 bits")
        frame = [address & 0x7F]
        for shift in (24, 16, 8, 0):
            frame.append((value >> shift) & 0xFF)
        return frame

    def write_register(self, address: int, value: int) -> Sequence[int]:
        spi = self._ensure_spi()
        frame = self._build_frame(address, value)
        return spi.xfer2(frame)

    def configure(self) -> None:
        spi = self._ensure_spi()
        for address, value in self._registers.writes:
            spi.xfer2(self._build_frame(address, value))

    def apply_registers(self, writes: Iterable[Tuple[int, int]]) -> None:
        spi = self._ensure_spi()
        for address, value in writes:
            spi.xfer2(self._build_frame(address, value))


__all__ = [
    "REG_GCONF",
    "REG_GSTAT",
    "REG_IHOLD_IRUN",
    "REG_TPOWERDOWN",
    "REG_TPWMTHRS",
    "REG_CHOPCONF",
    "REG_PWMCONF",
    "TMC5160RegisterPreset",
    "TMC5160Configurator",
]
