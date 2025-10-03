"""Testes para o configurador do driver TMC5160."""
from __future__ import annotations

from pathlib import Path
import sys

import pytest

MODULE_DIR = Path(__file__).resolve().parent
if str(MODULE_DIR) not in sys.path:
    sys.path.insert(0, str(MODULE_DIR))

from tmc5160 import (
    REG_CHOPCONF,
    REG_GCONF,
    REG_GSTAT,
    REG_IHOLD_IRUN,
    REG_PWMCONF,
    REG_TPOWERDOWN,
    REG_TPWMTHRS,
    TMC5160Configurator,
    TMC5160RegisterPreset,
)


class DummySpi:
    def __init__(self) -> None:
        self.open_args = []
        self.closed = False
        self.max_speed_hz = None
        self.mode = None
        self.bits_per_word = None
        self.lsbfirst = None
        self.cshigh = None
        self.frames = []

    def open(self, bus, dev):
        self.open_args.append((bus, dev))

    def close(self):
        self.closed = True

    def xfer2(self, frame):
        self.frames.append(frame)
        return [0] * len(frame)


def dummy_factory():
    return DummySpi()


def test_configure_uses_default_preset_and_spi_settings():
    configurator = TMC5160Configurator(
        bus=1,
        device=2,
        speed_hz=2_000_000,
        spi_device_factory=dummy_factory,
    )

    configurator.configure()

    spi = configurator._spi  # type: ignore[attr-defined]
    assert isinstance(spi, DummySpi)
    assert spi.open_args == [(1, 2)]
    assert spi.max_speed_hz == 2_000_000
    assert spi.mode == 0b11
    assert spi.bits_per_word == 8
    assert spi.lsbfirst is False
    assert spi.cshigh is False

    expected_frames = [
        [REG_GSTAT, 0x00, 0x00, 0x00, 0x07],
        [REG_GCONF, 0x00, 0x00, 0x00, 0x04],
        [REG_IHOLD_IRUN, 0x00, 0x06, 0x1F, 0x0A],
        [REG_TPOWERDOWN, 0x00, 0x00, 0x00, 0x14],
        [REG_TPWMTHRS, 0x00, 0x00, 0x01, 0xF4],
        [REG_CHOPCONF, 0x10, 0x41, 0x01, 0x50],
        [REG_PWMCONF, 0xC1, 0x0D, 0x00, 0x24],
    ]
    assert spi.frames == expected_frames


def test_write_register_validates_inputs():
    configurator = TMC5160Configurator(
        spi_device_factory=dummy_factory,
    )

    with pytest.raises(ValueError):
        configurator.write_register(0x80, 0x00)

    with pytest.raises(ValueError):
        configurator.write_register(0x01, 0x1_0000_0000)


def test_apply_custom_registers_sequence():
    custom_preset = TMC5160RegisterPreset(
        writes=((0x20, 0x12345678),)
    )
    configurator = TMC5160Configurator(
        register_preset=custom_preset,
        spi_device_factory=dummy_factory,
    )

    configurator.configure()

    spi = configurator._spi  # type: ignore[attr-defined]
    assert spi.frames == [[0x20, 0x12, 0x34, 0x56, 0x78]]

    configurator.apply_registers(((0x21, 0x0ABCDEF0),))
    assert spi.frames == [
        [0x20, 0x12, 0x34, 0x56, 0x78],
        [0x21, 0x0A, 0xBC, 0xDE, 0xF0],
    ]


def test_context_manager_closes_spi():
    configurator = TMC5160Configurator(spi_device_factory=dummy_factory)
    with configurator as cfg:
        cfg.configure()
        spi = cfg._spi  # type: ignore[attr-defined]
        assert isinstance(spi, DummySpi)
        assert spi.closed is False
    assert spi.closed is True
