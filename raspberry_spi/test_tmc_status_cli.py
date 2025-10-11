import io
import unittest
from contextlib import redirect_stdout
from unittest.mock import MagicMock, patch

from cnc_spi_client import build_parser, tmc_status
from tmc5160 import TMC5160DrvStatus, TMC5160Reply


class TmcStatusCliTests(unittest.TestCase):
    def test_tmc_index_overrides_dev(self) -> None:
        parser = build_parser()
        args = parser.parse_args(["tmc-status", "--tmc-index", "4"])

        fake_gstat = TMC5160Reply(
            status=0x80,
            value=0x00000007,
            raw_bytes=(0x80, 0x00, 0x00, 0x00, 0x00),
        )
        fake_drv = TMC5160DrvStatus(
            status_byte=0x91,
            raw=0x12345678,
            raw_bytes=(0x91, 0x12, 0x34, 0x56, 0x78),
            sg_result=0x12,
            fsactive=False,
            s2ga=False,
            s2gb=False,
            s2vsa=False,
            s2vsb=False,
            ola=False,
            olb=False,
            stallguard=False,
            ot=False,
            otpw=False,
            driver_error=False,
            uv_cp=False,
        )

        with patch("cnc_spi_client.TMC5160Spi") as mock_spi:
            driver = MagicMock()
            driver.clear_gstat.return_value = fake_gstat
            driver.read_drv_status.return_value = fake_drv
            mock_spi.return_value.__enter__.return_value = driver

            buf = io.StringIO()
            with redirect_stdout(buf):
                tmc_status(args)

        mock_spi.assert_called_once_with(bus=0, dev=4, speed_hz=1_000_000)
        driver.clear_gstat.assert_called_once_with()
        driver.read_drv_status.assert_called_once_with()

        output = buf.getvalue()
        self.assertIn("'bytes': ['0x80', '0x00', '0x00', '0x00', '0x00']", output)
        self.assertIn("'raw_bytes': ['0x91', '0x12', '0x34', '0x56', '0x78']", output)


if __name__ == "__main__":  # pragma: no cover
    unittest.main()

