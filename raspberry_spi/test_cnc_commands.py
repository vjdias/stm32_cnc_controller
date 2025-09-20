import argparse
import sys
import unittest
from pathlib import Path
from typing import List

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_commands import CNCCommandExecutor
    from .cnc_requests import CNCRequestBuilder
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_commands import CNCCommandExecutor  # type: ignore
    from cnc_requests import CNCRequestBuilder  # type: ignore


class _TimeoutClient:
    def exchange(self, request_type: int, request: List[int], **_: int) -> List[int]:
        raise TimeoutError("Resposta SPI não recebida/validada no prazo.")


class CNCCommandExecutorErrorTests(unittest.TestCase):
    def test_timeout_error_contains_context_information(self) -> None:
        executor = CNCCommandExecutor(_TimeoutClient())
        args = argparse.Namespace(command="led-control", tries=1)
        request = CNCRequestBuilder.led_control(2, 0x01, 0, 0)

        with self.assertRaisesRegex(
            TimeoutError,
            r"(?s)cnc_client\.exchange.*led-control.*0x07.*AA 07 02 01 00 00 00 04 55",
        ):
            executor._execute_request(0x07, request, args)


if __name__ == "__main__":
    unittest.main()
