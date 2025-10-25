import argparse
import sys
import unittest
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_spi_client import _common_args, build_parser
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_spi_client import _common_args, build_parser  # type: ignore


class CNCSPICLITests(unittest.TestCase):
    def setUp(self) -> None:
        self.parser = build_parser()

    def test_queue_add_accepts_settle_delay(self) -> None:
        args = self.parser.parse_args(
            [
                "queue-add",
                "--frame-id",
                "10",
                "--dir",
                "0x01",
                "--vx",
                "1500",
                "--sx",
                "20000",
                "--vy",
                "0",
                "--sy",
                "0",
                "--vz",
                "0",
                "--sz",
                "0",
                "--settle-delay",
                "2",
            ]
        )
        self.assertEqual(args.command, "queue-add")
        self.assertAlmostEqual(args.settle_delay, 2.0)

    def test_queue_status_accepts_settle_delay(self) -> None:
        args = self.parser.parse_args(
            [
                "queue-status",
                "--frame-id",
                "5",
                "--settle-delay",
                "0.005",
            ]
        )
        self.assertEqual(args.command, "queue-status")
        self.assertAlmostEqual(args.settle_delay, 0.005)

    def test_hello_accepts_settle_delay(self) -> None:
        args = self.parser.parse_args(["hello", "--tries", "1", "--settle-delay", "0.1"])
        self.assertEqual(args.command, "hello")
        self.assertAlmostEqual(args.settle_delay, 0.1)

    def test_common_args_can_update_settle_delay_default(self) -> None:
        parser = argparse.ArgumentParser(prog="test")
        parser.add_argument("--settle-delay", type=float, default=0.001)

        _common_args(parser, include_settle_delay=True, default_settle_delay=0.123)

        args = parser.parse_args([])
        self.assertAlmostEqual(args.settle_delay, 0.123)

        args = parser.parse_args(["--settle-delay", "0.5"])
        self.assertAlmostEqual(args.settle_delay, 0.5)


if __name__ == "__main__":
    unittest.main()
