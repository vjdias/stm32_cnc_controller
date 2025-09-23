import sys
import unittest
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_client import (
        _build_spi_dma_frame,
        _extract_response_frame,
        _validate_handshake_frame,
    )
    from .cnc_protocol import (
        REQ_HEADER,
        REQ_LED_CTRL,
        REQ_TAIL,
        RESP_HEADER,
        RESP_TAIL,
        SPI_DMA_FRAME_LEN,
        SPI_DMA_HANDSHAKE_BUSY,
        SPI_DMA_HANDSHAKE_NO_COMM,
        SPI_DMA_HANDSHAKE_READY,
    )
    from .cnc_responses import CNCResponseDecoder
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_client import (  # type: ignore
        _build_spi_dma_frame,
        _extract_response_frame,
        _validate_handshake_frame,
    )
    from cnc_protocol import (  # type: ignore
        REQ_HEADER,
        REQ_LED_CTRL,
        REQ_TAIL,
        RESP_HEADER,
        RESP_TAIL,
        SPI_DMA_FRAME_LEN,
        SPI_DMA_HANDSHAKE_BUSY,
        SPI_DMA_HANDSHAKE_NO_COMM,
        SPI_DMA_HANDSHAKE_READY,
    )
    from cnc_responses import CNCResponseDecoder  # type: ignore


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

    def test_busy_handshake_for_entire_frame_reports_dma_span(self) -> None:
        handshake = [SPI_DMA_HANDSHAKE_BUSY] * SPI_DMA_FRAME_LEN

        with self.assertRaises(BufferError) as ctx:
            _validate_handshake_frame(self.frame, handshake, len(self.payload))

        msg = str(ctx.exception)
        self.assertIn("frame DMA", msg)
        self.assertIn(str(SPI_DMA_FRAME_LEN), msg)

    def test_unknown_handshake_raises_runtime_error_on_padding(self) -> None:
        handshake = [SPI_DMA_HANDSHAKE_READY] * SPI_DMA_FRAME_LEN
        handshake[0] = 0xE1

        with self.assertRaises(RuntimeError) as ctx:
            _validate_handshake_frame(self.frame, handshake, len(self.payload))

        msg = str(ctx.exception)
        self.assertIn("preenchimento[0]", msg)
        self.assertIn("0xE1", msg)

    def test_zero_handshake_raises_connection_error(self) -> None:
        handshake = [SPI_DMA_HANDSHAKE_NO_COMM] * SPI_DMA_FRAME_LEN

        with self.assertRaises(ConnectionError) as ctx:
            _validate_handshake_frame(self.frame, handshake, len(self.payload))

        msg = str(ctx.exception)
        self.assertIn("0x00", msg)
        self.assertIn("Comunicação SPI não ocorreu", msg)


class ResponsePollingValidationTests(unittest.TestCase):
    def setUp(self) -> None:
        self.spec = CNCResponseDecoder.SPECS[REQ_LED_CTRL]
        payload = [
            RESP_HEADER,
            self.spec.response_type,
            0x01,
            0x02,
            0x03,
            0x04,
            RESP_TAIL,
        ]
        frame = [SPI_DMA_HANDSHAKE_READY] * SPI_DMA_FRAME_LEN
        start = 5
        for offset, byte in enumerate(payload):
            frame[start + offset] = byte
        self.response_frame = frame
        self.header_idx = start
        self.tail_idx = start + len(payload) - 1

    def test_busy_frame_without_header_raises_buffer_error(self) -> None:
        busy_frame = [SPI_DMA_HANDSHAKE_BUSY] * SPI_DMA_FRAME_LEN
        with self.assertRaises(BufferError):
            _extract_response_frame(busy_frame, self.spec.length, self.spec.response_type)

    def test_busy_byte_before_header_raises_buffer_error(self) -> None:
        frame = list(self.response_frame)
        frame[self.header_idx - 1] = SPI_DMA_HANDSHAKE_BUSY
        with self.assertRaises(BufferError) as ctx:
            _extract_response_frame(frame, self.spec.length, self.spec.response_type)
        self.assertIn("antes do header", str(ctx.exception))

    def test_busy_byte_after_tail_raises_buffer_error(self) -> None:
        frame = list(self.response_frame)
        frame[self.tail_idx + 1] = SPI_DMA_HANDSHAKE_BUSY
        with self.assertRaises(BufferError) as ctx:
            _extract_response_frame(frame, self.spec.length, self.spec.response_type)
        self.assertIn("após o tail", str(ctx.exception))

    def test_busy_byte_inside_payload_is_ignored(self) -> None:
        frame = list(self.response_frame)
        payload_idx = self.header_idx + 3
        frame[payload_idx] = SPI_DMA_HANDSHAKE_BUSY
        extracted = _extract_response_frame(frame, self.spec.length, self.spec.response_type)
        self.assertIsNotNone(extracted)
        self.assertEqual(extracted[3], SPI_DMA_HANDSHAKE_BUSY)


if __name__ == "__main__":
    unittest.main()
