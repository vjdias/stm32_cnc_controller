import sys
import unittest
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_protocol import (
        REQ_HEADER,
        REQ_LED_CTRL,
        REQ_TAIL,
        SPI_DMA_FRAME_LEN,
        SPI_DMA_MAX_PAYLOAD,
        parity_check_byte_1N,
    )
    from .cnc_requests import CNCRequestBuilder
    from .cnc_client import _build_spi_dma_frame
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_protocol import (  # type: ignore
        REQ_HEADER,
        REQ_LED_CTRL,
        REQ_TAIL,
        SPI_DMA_FRAME_LEN,
        SPI_DMA_MAX_PAYLOAD,
        parity_check_byte_1N,
    )
    from cnc_requests import CNCRequestBuilder  # type: ignore
    from cnc_client import _build_spi_dma_frame  # type: ignore


class LedFrameEncodingTests(unittest.TestCase):
    def test_led_request_padded_payload(self) -> None:
        frame_id = 0x12
        mask = 0x01
        mode = 0x02
        freq_hz = 25
        payload = CNCRequestBuilder.led_control(frame_id, mask, mode, freq_hz)

        self.assertEqual(len(payload), SPI_DMA_MAX_PAYLOAD)
        self.assertEqual(payload[0], REQ_HEADER)
        self.assertEqual(payload[1], REQ_LED_CTRL)
        self.assertEqual(payload[2], frame_id & 0xFF)
        self.assertEqual(payload[3], mask & 0xFF)
        self.assertEqual(payload[4], mode & 0xFF)
        self.assertEqual(payload[5], (freq_hz >> 8) & 0xFF)
        self.assertEqual(payload[6], freq_hz & 0xFF)
        self.assertTrue(parity_check_byte_1N(payload, 6, 7))
        self.assertTrue(all(b == 0 for b in payload[8:-1]))
        core = payload[:8] + [payload[-1]]
        self.assertEqual(core[0], REQ_HEADER)
        self.assertEqual(core[-1], REQ_TAIL)
        self.assertTrue(parity_check_byte_1N(core, 6, 7))

    def test_dma_frame_wrapper_preserves_payload(self) -> None:
        payload = CNCRequestBuilder.led_control(0x2A, 0x01, 0x01, 0)
        frame = _build_spi_dma_frame(payload)

        self.assertEqual(len(frame), SPI_DMA_FRAME_LEN)
        self.assertEqual(frame[0], 0x00)
        self.assertEqual(frame[1:], payload)


if __name__ == "__main__":
    unittest.main()
