import sys
import unittest
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_client import _build_spi_dma_frame, _validate_handshake_frame
    from .cnc_protocol import (
        REQ_HEADER,
        REQ_LED_CTRL,
        REQ_TAIL,
        SPI_DMA_FRAME_LEN,
        SPI_DMA_HANDSHAKE_BUSY,
        SPI_DMA_HANDSHAKE_READY,
    )
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_client import _build_spi_dma_frame, _validate_handshake_frame  # type: ignore
    from cnc_protocol import (  # type: ignore
        REQ_HEADER,
        REQ_LED_CTRL,
        REQ_TAIL,
        SPI_DMA_FRAME_LEN,
        SPI_DMA_HANDSHAKE_BUSY,
        SPI_DMA_HANDSHAKE_READY,
    )


class HandshakeValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        payload = [REQ_HEADER, REQ_LED_CTRL, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, REQ_TAIL]
        self.payload = payload
        self.frame = _build_spi_dma_frame(payload)

    def test_accepts_ready_handshake_for_entire_frame(self) -> None:
        handshake = [SPI_DMA_HANDSHAKE_READY] * SPI_DMA_FRAME_LEN
        _validate_handshake_frame(self.frame, handshake, len(self.payload))

    def test_busy_handshake_raises_buffer_error_with_payload_context(self) -> None:
        handshake = [SPI_DMA_HANDSHAKE_READY] * SPI_DMA_FRAME_LEN
        handshake[-1] = SPI_DMA_HANDSHAKE_BUSY

        with self.assertRaises(BufferError) as ctx:
            _validate_handshake_frame(self.frame, handshake, len(self.payload))

        msg = str(ctx.exception)
        self.assertIn("payload[", msg)
        self.assertIn("0x5A", msg)

    def test_unknown_handshake_raises_runtime_error_on_padding(self) -> None:
        handshake = [SPI_DMA_HANDSHAKE_READY] * SPI_DMA_FRAME_LEN
        handshake[0] = 0xE1

        with self.assertRaises(RuntimeError) as ctx:
            _validate_handshake_frame(self.frame, handshake, len(self.payload))

        msg = str(ctx.exception)
        self.assertIn("preenchimento[0]", msg)
        self.assertIn("0xE1", msg)


if __name__ == "__main__":
    unittest.main()
