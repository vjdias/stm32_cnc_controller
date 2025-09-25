import sys
import types
import unittest
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_client import CNCClient, _build_spi_dma_frame
    from .cnc_protocol import (
        REQ_LED_CTRL,
        RESP_HEADER,
        RESP_LED_CTRL,
        RESP_TAIL,
        SPI_DMA_FRAME_LEN,
        SPI_DMA_HANDSHAKE_READY,
        SPI_DMA_POLL_BYTE,
    )
    from .cnc_requests import CNCRequestBuilder
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_client import CNCClient, _build_spi_dma_frame  # type: ignore
    from cnc_protocol import (  # type: ignore
        REQ_LED_CTRL,
        RESP_HEADER,
        RESP_LED_CTRL,
        RESP_TAIL,
        SPI_DMA_FRAME_LEN,
        SPI_DMA_HANDSHAKE_READY,
        SPI_DMA_POLL_BYTE,
    )
    from cnc_requests import CNCRequestBuilder  # type: ignore


class _DummySpi:
    def __init__(self, responses):
        self._queue = [list(r) for r in responses]
        self.calls = []
        self.max_speed_hz = 0
        self.mode = 0
        self.bits_per_word = 0

    def open(self, bus: int, dev: int) -> None:  # pragma: no cover - no-op
        self.bus = bus
        self.dev = dev

    def close(self) -> None:  # pragma: no cover - no-op
        pass

    def xfer2(self, data):
        self.calls.append(list(data))
        if not self._queue:
            raise AssertionError("Sem resposta configurada para xfer2")
        return list(self._queue.pop(0))


class CNCClientExchangeTests(unittest.TestCase):
    def _make_client(self, responses):
        dummy_module = types.SimpleNamespace()
        dummy_spi = _DummySpi(responses)
        dummy_module.SpiDev = lambda: dummy_spi

        module_name = CNCClient.__module__

        from unittest.mock import patch

        patcher = patch(f"{module_name}.spidev", dummy_module)
        self.addCleanup(patcher.stop)
        patcher.start()

        client = CNCClient()
        self.addCleanup(client.close)
        return client, dummy_spi

    def test_exchange_reads_tail_of_full_dma_frame(self) -> None:
        request = CNCRequestBuilder.led_control(2, 0x01, 0, 0)
        dma_frame = _build_spi_dma_frame(request)
        handshake = [SPI_DMA_HANDSHAKE_READY] * len(dma_frame)
        payload = [
            RESP_HEADER,
            RESP_LED_CTRL,
            0x02,
            0x01,
            0x00,
            0x00,
            RESP_TAIL,
        ]
        response_frame = [
            SPI_DMA_HANDSHAKE_READY
        ] * (SPI_DMA_FRAME_LEN - len(payload)) + payload

        client, spi = self._make_client([handshake, response_frame])

        frame = client.exchange(REQ_LED_CTRL, request, tries=1, settle_delay_s=0.0)

        self.assertEqual(frame, payload)
        self.assertEqual(len(spi.calls), 2)
        self.assertEqual(spi.calls[0], dma_frame)
        self.assertEqual(len(spi.calls[1]), SPI_DMA_FRAME_LEN)
        self.assertTrue(all(b == SPI_DMA_POLL_BYTE for b in spi.calls[1]))

    def test_exchange_retries_until_response_is_available(self) -> None:
        request = CNCRequestBuilder.led_control(3, 0x01, 1, 0)
        dma_frame = _build_spi_dma_frame(request)
        handshake = [SPI_DMA_HANDSHAKE_READY] * len(dma_frame)
        empty_poll = [SPI_DMA_HANDSHAKE_READY] * SPI_DMA_FRAME_LEN
        payload = [
            RESP_HEADER,
            RESP_LED_CTRL,
            0x03,
            0x01,
            0x01,
            0x00,
            RESP_TAIL,
        ]
        response_frame = [
            SPI_DMA_HANDSHAKE_READY
        ] * (SPI_DMA_FRAME_LEN - len(payload)) + payload

        client, spi = self._make_client([handshake, empty_poll, response_frame])

        frame = client.exchange(REQ_LED_CTRL, request, tries=3, settle_delay_s=0.0)

        self.assertEqual(frame, payload)
        self.assertEqual(len(spi.calls), 3)
        self.assertEqual(spi.calls[0], dma_frame)
        self.assertEqual(len(spi.calls[1]), SPI_DMA_FRAME_LEN)
        self.assertEqual(len(spi.calls[2]), SPI_DMA_FRAME_LEN)


if __name__ == "__main__":
    unittest.main()
