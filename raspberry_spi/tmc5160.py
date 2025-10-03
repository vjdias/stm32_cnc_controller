"""Configuração de drivers TMC5160 a partir do Raspberry Pi."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Tuple

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


STATUS_FLAG_DESCRIPTIONS = {
    7: ("stallguard", "StallGuard detectado"),
    6: ("ot", "Sobretemperatura (OT)"),
    5: ("otpw", "Pré-aviso de sobretemperatura (OTPW)"),
    4: ("s2ga", "Curto à terra na fase A (S2GA)"),
    3: ("s2gb", "Curto à terra na fase B (S2GB)"),
    2: ("s2vsa", "Curto à alimentação na fase A (S2VSA)"),
    1: ("s2vsb", "Curto à alimentação na fase B (S2VSB)"),
    0: ("uv_cp", "Subtensão do charge pump (UV_CP)"),
}

FAULT_BITS = (6, 5, 4, 3, 2, 1, 0)


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


@dataclass(frozen=True)
class TMC5160Status:
    """Representa o byte de status retornado pelo TMC5160."""

    raw: int

    def flag(self, bit: int) -> bool:
        return bool(self.raw & (1 << bit))

    @property
    def stallguard(self) -> bool:
        return self.flag(7)

    def active_faults(self) -> List[str]:
        return [
            STATUS_FLAG_DESCRIPTIONS[bit][1]
            for bit in FAULT_BITS
            if self.flag(bit)
        ]

    def has_fault(self) -> bool:
        return bool(self.active_faults())

    def summary(self) -> str:
        stall_text = (
            "StallGuard detectado (SG=1)."
            if self.stallguard
            else "StallGuard inativo (SG=0)."
        )
        faults = self.active_faults()
        if faults:
            fault_text = "Alertas ativos: {}.".format(
                ", ".join(faults)
            )
        else:
            fault_text = (
                "Alertas ativos: nenhum (OT/OTPW/S2GA/S2GB/S2VSA/S2VSB/UV_CP limpos)."
            )
        return f"{stall_text} {fault_text}"


@dataclass(frozen=True)
class TMC5160TransferResult:
    """Resultado de uma transferência SPI com o TMC5160."""

    address: int
    value: int
    response: Tuple[int, int, int, int, int]

    def __post_init__(self) -> None:
        if len(self.response) != 5:
            raise ValueError("Resposta do TMC5160 deve conter 5 bytes")

    @property
    def status(self) -> TMC5160Status:
        return TMC5160Status(self.response[0])

    @property
    def previous_data(self) -> int:
        _, b1, b2, b3, b4 = self.response
        return (b1 << 24) | (b2 << 16) | (b3 << 8) | b4

    @property
    def raw_hex(self) -> str:
        return " ".join(f"0x{byte:02X}" for byte in self.response)

    def raise_on_faults(self) -> None:
        faults = self.status.active_faults()
        if faults:
            fault_list = ", ".join(faults)
            raise RuntimeError(
                "Driver TMC5160 sinalizou alertas: {faults} (status=0x{status:02X}, "
                "resposta={raw})".format(
                    faults=fault_list,
                    status=self.status.raw,
                    raw=self.raw_hex,
                )
            )


class TMC5160Configurator:
    """Configura um driver TMC5160 ligado ao barramento SPI do Raspberry Pi."""

    def __init__(
        self,
        *,
        bus: int = 0,
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

    def _transfer(self, address: int, value: int) -> TMC5160TransferResult:
        spi = self._ensure_spi()
        frame = self._build_frame(address, value)
        response = spi.xfer2(frame)
        if len(response) != 5:
            raise RuntimeError(
                "Resposta SPI inválida do TMC5160: esperado 5 bytes, recebido {}".format(
                    len(response)
                )
            )
        return TMC5160TransferResult(address, value, tuple(response))

    def write_register(self, address: int, value: int) -> TMC5160TransferResult:
        return self._transfer(address, value)

    def configure(self) -> List[TMC5160TransferResult]:
        results: List[TMC5160TransferResult] = []
        for address, value in self._registers.writes:
            results.append(self._transfer(address, value))
        return results

    def apply_registers(self, writes: Iterable[Tuple[int, int]]) -> List[TMC5160TransferResult]:
        results: List[TMC5160TransferResult] = []
        for address, value in writes:
            results.append(self._transfer(address, value))
        return results


__all__ = [
    "REG_GCONF",
    "REG_GSTAT",
    "REG_IHOLD_IRUN",
    "REG_TPOWERDOWN",
    "REG_TPWMTHRS",
    "REG_CHOPCONF",
    "REG_PWMCONF",
    "STATUS_FLAG_DESCRIPTIONS",
    "TMC5160Status",
    "TMC5160TransferResult",
    "TMC5160RegisterPreset",
    "TMC5160Configurator",
]
