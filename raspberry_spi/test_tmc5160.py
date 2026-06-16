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
    TMC5160ReadResult,
    TMC5160RegisterPreset,
    TMC5160Status,
    TMC5160TransferResult,
    decode_register_value,
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


class ModeErrorSpi:
    def __init__(self) -> None:
        self.open_args = []
        self.closed = False
        self.max_speed_hz = None
        self._mode = None
        self.bits_per_word = None
        self.lsbfirst = None
        self.cshigh = None

    def open(self, bus, dev):
        self.open_args.append((bus, dev))

    def close(self):
        self.closed = True

    def xfer2(self, frame):
        raise AssertionError("Modo SPI não deveria permitir transferências ao falhar")

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, value):
        raise OSError(22, "Invalid argument")


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
        [REG_IHOLD_IRUN, 0x00, 0x06, 0x01, 0x01],
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


def test_mode_configuration_error_is_reported_with_hint():
    configurator = TMC5160Configurator(
        bus=1,
        device=1,
        spi_device_factory=ModeErrorSpi,
    )

    with pytest.raises(RuntimeError) as excinfo:
        configurator.configure()

    message = str(excinfo.value)
    assert "/dev/spidev1.1" in message
    assert "modo SPI 3" in message
    assert "Invalid argument" in message


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
    status = TMC5160Status(0b0000_0100)
    assert status.stallguard is True
    # driver_error deve acionar exce��o na valida��o
    result = TMC5160TransferResult(0x00, 0x0, (0b0000_0010, 0, 0, 0, 0))
    with pytest.raises(RuntimeError) as exc:
        result.raise_on_faults()
    assert "driver_error" in str(exc.value)
    assert "status=0x02" in str(exc.value)


def test_read_register_performs_two_transfers_and_returns_value():
    configurator = TMC5160Configurator(spi_device_factory=dummy_factory)
    spi = configurator._ensure_spi()  # type: ignore[attr-defined]
    assert isinstance(spi, DummySpi)

    spi.queue_response([0x00, 0, 0, 0, 0])
    spi.queue_response([0x00, 0x11, 0x22, 0x33, 0x44])

    result = configurator.read_register(REG_CHOPCONF)

    assert isinstance(result, TMC5160ReadResult)
    assert spi.frames == [
        [0x80 | REG_CHOPCONF, 0x00, 0x00, 0x00, 0x00],
        [0x00, 0x00, 0x00, 0x00, 0x00],
    ]
    assert result.value == 0x11223344
    assert result.raw_hex == "0x00 0x11 0x22 0x33 0x44"


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
            self.read_requests = []

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

        def read_registers(self, addresses):
            self.read_requests.append(list(addresses))
            return []

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


def test_cli_status_lists_registers_and_values(capsys):
    created = []

    class RecordingConfigurator:
        def __init__(self, *, bus, device, speed_hz, register_preset):
            self.bus = bus
            self.device = device
            self.speed_hz = speed_hz
            self.register_preset = register_preset
            self.closed = False
            self.read_requests: list[list[int]] = []

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            self.closed = True

        def read_registers(self, addresses):
            self.read_requests.append(list(addresses))
            results = []
            for idx, address in enumerate(addresses):
                value = 0x11111111 * (idx + 1)
                request = TMC5160TransferResult(0x80 | address, 0, (0, 0, 0, 0, 0))
                b1 = (value >> 24) & 0xFF
                b2 = (value >> 16) & 0xFF
                b3 = (value >> 8) & 0xFF
                b4 = value & 0xFF
                reply = TMC5160TransferResult(0x00, 0, (0, b1, b2, b3, b4))
                results.append(TMC5160ReadResult(address, request, reply))
            return results

        def configure(self):
            raise AssertionError("configure não deve ser chamado em status")

        def apply_registers(self, writes):
            raise AssertionError("apply_registers não deve ser chamado em status")

    def factory(**kwargs):
        cfg = RecordingConfigurator(**kwargs)
        created.append(cfg)
        return cfg

    exit_code = tmc_cli_run(["status"], configurator_factory=factory)

    assert exit_code == 0
    assert len(created) == 1
    cfg = created[0]
    assert cfg.read_requests == [
        [REG_GSTAT, 0x6F, REG_GCONF, REG_IHOLD_IRUN, REG_TPOWERDOWN, REG_TPWMTHRS, REG_CHOPCONF, REG_PWMCONF]
    ]
    assert cfg.closed is True

    captured = capsys.readouterr()
    assert "Consultando registradores" in captured.out
    assert "Respostas do TMC5160" in captured.out
    assert "Resposta útil" in captured.out
    assert "Tradução" in captured.out
    assert "0x11111111" in captured.out


def test_cli_loop_test_repeats_specified_iterations_and_can_be_quiet(capsys):
    created = []

    class RecordingConfigurator:
        def __init__(self, *, bus, device, speed_hz, register_preset):
            self.bus = bus
            self.device = device
            self.speed_hz = speed_hz
            self.register_preset = register_preset
            self.closed = False
            self.write_calls: list[tuple[int, int]] = []

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, tb):
            self.closed = True

        def write_register(self, address, value):
            self.write_calls.append((address, value))
            return TMC5160TransferResult(address, value, (0, 0, 0, 0, 0))

    def factory(**kwargs):
        cfg = RecordingConfigurator(**kwargs)
        created.append(cfg)
        return cfg

    exit_code = tmc_cli_run(
        [
            "loop-test",
            "--iterations",
            "3",
            "--interval",
            "0",
            "--quiet",
            "--address",
            "gconf",
            "--value",
            "0x4",
        ],
        configurator_factory=factory,
    )

    assert exit_code == 0
    assert len(created) == 1
    cfg = created[0]
    assert cfg.write_calls == [(REG_GCONF, 0x4)] * 3
    assert cfg.closed is True

    captured = capsys.readouterr()
    assert "Loop concluído" in captured.out
    assert "status=" not in captured.out


@pytest.mark.parametrize(
    "address,value,expected",
    [
        (REG_GSTAT, 0x00, "Nenhum bit relevante ativo."),
        (REG_GSTAT, 0x05, "RESET"),
        (REG_GCONF, 0x00000004, "StealthChop"),
        (REG_IHOLD_IRUN, 0x00060101, "IHOLD=1"),
        (REG_CHOPCONF, 0x10410150, "TOFF"),
        (REG_PWMCONF, 0xC10D0024, "PWM_OFS"),
    ],
)
def test_decode_register_value_produces_human_readable_text(address, value, expected):
    details = decode_register_value(address, value)
    assert any(expected in item for item in details)


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


