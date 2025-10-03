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
    TMC5160Status,
    TMC5160TransferResult,
)
from tmc5160_cli import run as tmc_cli_run


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
        self.response_queue: list[list[int]] = []

    def open(self, bus, dev):
        self.open_args.append((bus, dev))

    def close(self):
        self.closed = True

    def xfer2(self, frame):
        self.frames.append(frame)
        if self.response_queue:
            return self.response_queue.pop(0)
        return [0] * len(frame)

    def queue_response(self, response):
        self.response_queue.append(response)


def dummy_factory():
    return DummySpi()


def test_configure_uses_default_preset_and_spi_settings():
    configurator = TMC5160Configurator(
        bus=1,
        device=2,
        speed_hz=2_000_000,
        spi_device_factory=dummy_factory,
    )

    results = configurator.configure()

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
    assert [res.response for res in results] == [
        (0, 0, 0, 0, 0)
        for _ in expected_frames
    ]


def test_write_register_validates_inputs():
    configurator = TMC5160Configurator(
        spi_device_factory=dummy_factory,
    )

    with pytest.raises(ValueError):
        configurator.write_register(0x80, 0x00)

    with pytest.raises(ValueError):
        configurator.write_register(0x01, 0x1_0000_0000)


def test_write_register_returns_result_and_validates_response():
    configurator = TMC5160Configurator(
        spi_device_factory=dummy_factory,
    )

    spi = configurator._ensure_spi()  # type: ignore[attr-defined]
    assert isinstance(spi, DummySpi)
    spi.queue_response([0x00, 0xAA, 0xBB, 0xCC, 0xDD])
    result = configurator.write_register(0x05, 0x12345678)
    assert result.address == 0x05
    assert result.value == 0x12345678
    assert result.raw_hex == "0x00 0xAA 0xBB 0xCC 0xDD"
    assert result.previous_data == 0xAABBCCDD


def test_write_register_detects_invalid_response_length():
    configurator = TMC5160Configurator(
        spi_device_factory=dummy_factory,
    )

    spi = configurator._ensure_spi()  # type: ignore[attr-defined]
    assert isinstance(spi, DummySpi)
    spi.queue_response([0x00])
    with pytest.raises(RuntimeError) as exc:
        configurator.write_register(0x02, 0x0)
    assert "esperado 5 bytes" in str(exc.value)


def test_apply_custom_registers_sequence():
    custom_preset = TMC5160RegisterPreset(
        writes=((0x20, 0x12345678),)
    )
    configurator = TMC5160Configurator(
        register_preset=custom_preset,
        spi_device_factory=dummy_factory,
    )

    results = configurator.configure()

    spi = configurator._spi  # type: ignore[attr-defined]
    assert spi.frames == [[0x20, 0x12, 0x34, 0x56, 0x78]]
    assert [res.response for res in results] == [(0, 0, 0, 0, 0)]

    more = configurator.apply_registers(((0x21, 0x0ABCDEF0),))
    assert spi.frames == [
        [0x20, 0x12, 0x34, 0x56, 0x78],
        [0x21, 0x0A, 0xBC, 0xDE, 0xF0],
    ]
    assert [res.response for res in more] == [(0, 0, 0, 0, 0)]


def test_context_manager_closes_spi():
    configurator = TMC5160Configurator(spi_device_factory=dummy_factory)
    with configurator as cfg:
        cfg.configure()
        spi = cfg._spi  # type: ignore[attr-defined]
        assert isinstance(spi, DummySpi)
        assert spi.closed is False
    assert spi.closed is True


def test_transfer_result_flags_and_validation():
    status = TMC5160Status(0b1100_0011)
    assert status.stallguard is True
    assert "Sobretemperatura" in status.summary()
    result = TMC5160TransferResult(0x00, 0x0, (0b0100_0001, 0, 0, 0, 0))
    with pytest.raises(RuntimeError) as exc:
        result.raise_on_faults()
    assert "Sobretemperatura" in str(exc.value)
    assert "status=0x41" in str(exc.value)


def test_cli_with_defaults_executes_preset_and_no_overrides(capsys):
    created = []

    class RecordingConfigurator:
        def __init__(self, *, bus, device, speed_hz, register_preset):
            self.bus = bus
            self.device = device
            self.speed_hz = speed_hz
            self.register_preset = register_preset
            self.configure_calls = 0
            self.applied: list[list[tuple[int, int]]] = []
            self.closed = False
            self.configure_results = [
                TMC5160TransferResult(address, value, (0, 0, 0, 0, 0))
                for address, value in register_preset.writes
            ]

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            self.closed = True

        def configure(self):
            self.configure_calls += 1
            return self.configure_results

        def apply_registers(self, writes):
            self.applied.append(list(writes))
            return [
                TMC5160TransferResult(address, value, (0, 0, 0, 0, 0))
                for address, value in writes
            ]

    def factory(**kwargs):
        cfg = RecordingConfigurator(**kwargs)
        created.append(cfg)
        return cfg

    exit_code = tmc_cli_run([], configurator_factory=factory)

    assert exit_code == 0
    assert len(created) == 1
    cfg = created[0]
    assert cfg.configure_calls == 1
    assert cfg.applied == []
    assert cfg.closed is True

    captured = capsys.readouterr()
    assert "preset padrão" in captured.out
    assert "Nenhum ajuste adicional" in captured.out
    assert "Respostas do TMC5160" in captured.out
    assert "Resposta bruta" in captured.out


def test_cli_accepts_overrides_and_skips_defaults(capsys):
    created = []

    class RecordingConfigurator:
        def __init__(self, *, bus, device, speed_hz, register_preset):
            self.bus = bus
            self.device = device
            self.speed_hz = speed_hz
            self.register_preset = register_preset
            self.configure_calls = 0
            self.applied: list[list[tuple[int, int]]] = []
            self.closed = False
            self.configure_results = [
                TMC5160TransferResult(address, value, (0, 0, 0, 0, 0))
                for address, value in register_preset.writes
            ]

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            self.closed = True

        def configure(self):
            self.configure_calls += 1
            return self.configure_results

        def apply_registers(self, writes):
            self.applied.append(list(writes))
            return [
                TMC5160TransferResult(address, value, (0, 0, 0, 0, 0))
                for address, value in writes
            ]

    def factory(**kwargs):
        cfg = RecordingConfigurator(**kwargs)
        created.append(cfg)
        return cfg

    args = [
        "--no-defaults",
        "--gconf",
        "0x05",
        "--write",
        "0x20=0x12345678",
    ]

    exit_code = tmc_cli_run(args, configurator_factory=factory)

    assert exit_code == 0
    assert len(created) == 1
    cfg = created[0]
    assert cfg.configure_calls == 0
    assert cfg.register_preset.writes == ()
    assert cfg.applied == [[(REG_GCONF, 0x05), (0x20, 0x12345678)]]
    assert cfg.closed is True

    captured = capsys.readouterr()
    assert "Aplicando ajustes adicionais" in captured.out
    assert "0x00000005" in captured.out
    assert "0x12345678" in captured.out
    assert "Resposta bruta" in captured.out


def test_cli_reports_missing_spi_device(capsys):
    class FailingConfigurator:
        def __init__(self, **_):
            pass

        def __enter__(self):
            raise FileNotFoundError("No such file")

        def __exit__(self, exc_type, exc, tb):
            return False

    exit_code = tmc_cli_run(
        [],
        configurator_factory=lambda **kwargs: FailingConfigurator(),
        device_finder=lambda: [Path("/dev/spidev0.0"), Path("/dev/spidev0.1")],
    )

    assert exit_code == 2
    captured = capsys.readouterr()
    assert "dispositivo SPI" in captured.err
    assert "/dev/spidev0.1" in captured.err
    assert "spi=on" in captured.err


