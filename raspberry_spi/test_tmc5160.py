import sys
import types
import unittest
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .tmc5160 import TMC5160Spi
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from tmc5160 import TMC5160Spi  # type: ignore


class _DummySpi:
    def __init__(self, responses):
        self._queue = [list(r) for r in responses]
        self.calls = []
        self.open_args = None
        self.closed = False
        self.max_speed_hz = 0
        self.mode = 0
        self.bits_per_word = 0

    def open(self, bus: int, dev: int) -> None:  # pragma: no cover - no-op
        self.open_args = (bus, dev)

    def close(self) -> None:  # pragma: no cover - no-op
        self.closed = True

    def xfer2(self, frame):
        self.calls.append(list(frame))
        if self._queue:
            return list(self._queue.pop(0))
        return [0] * len(frame)


class TMC5160SpiTests(unittest.TestCase):
    def _make_driver(self, responses):
        dummy_module = types.SimpleNamespace()
        dummy_spi = _DummySpi(responses)
        dummy_module.SpiDev = lambda: dummy_spi

        module_name = TMC5160Spi.__module__

        from unittest.mock import patch

        patcher = patch(f"{module_name}.spidev", dummy_module)
        self.addCleanup(patcher.stop)
        patcher.start()

        driver = TMC5160Spi()
        self.addCleanup(driver.close)
        return driver, dummy_spi

    def test_clear_gstat_writes_expected_frame(self) -> None:
        driver, spi = self._make_driver(responses=[[0x80, 0, 0, 0, 0]])

        reply = driver.clear_gstat()

        self.assertEqual(spi.calls, [[0x01, 0x00, 0x00, 0x00, 0x07]])
        self.assertEqual(reply.status, 0x80)
        self.assertEqual(reply.value, 0)
        self.assertEqual(reply.raw_bytes, (0x80, 0x00, 0x00, 0x00, 0x00))

    def test_read_register_pipeline(self) -> None:
        responses = [
            [0x00, 0, 0, 0, 0],
            [0xC0, 0x12, 0x34, 0x56, 0x78],
        ]
        driver, spi = self._make_driver(responses=responses)

        reply = driver.read_register(0x6F)

        self.assertEqual(
            spi.calls,
            [
                [0xEF, 0x00, 0x00, 0x00, 0x00],
                [0x00, 0x00, 0x00, 0x00, 0x00],
            ],
        )
        self.assertEqual(reply.status, 0xC0)
        self.assertEqual(reply.value, 0x12345678)
        self.assertEqual(reply.raw_bytes, (0xC0, 0x12, 0x34, 0x56, 0x78))

    def test_read_drv_status_decodes_flags(self) -> None:
        raw = (0x55 << 24)
        raw |= 1 << 23  # fsactive
        raw |= 1 << 22  # s2ga
        raw |= 1 << 20  # s2vsa
        raw |= 1 << 18  # ola
        raw |= 1 << 16  # stallguard
        raw |= 1 << 14  # otpw
        raw |= 1 << 12  # uv_cp

        responses = [
            [0x00, 0, 0, 0, 0],
            [0x90, (raw >> 24) & 0xFF, (raw >> 16) & 0xFF, (raw >> 8) & 0xFF, raw & 0xFF],
        ]
        driver, _ = self._make_driver(responses=responses)

        status = driver.read_drv_status()

        self.assertEqual(status.status_byte, 0x90)
        self.assertEqual(status.raw, raw & 0xFFFFFFFF)
        self.assertEqual(
            status.raw_bytes,
            (
                0x90,
                (raw >> 24) & 0xFF,
                (raw >> 16) & 0xFF,
                (raw >> 8) & 0xFF,
                raw & 0xFF,
            ),
        )
        self.assertEqual(status.sg_result, 0x55)
        self.assertTrue(status.fsactive)
        self.assertTrue(status.s2ga)
        self.assertFalse(status.s2gb)
        self.assertTrue(status.s2vsa)
        self.assertFalse(status.s2vsb)
        self.assertTrue(status.ola)
        self.assertFalse(status.olb)
        self.assertTrue(status.stallguard)
        self.assertFalse(status.ot)
        self.assertTrue(status.otpw)
        self.assertFalse(status.driver_error)
        self.assertTrue(status.uv_cp)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
