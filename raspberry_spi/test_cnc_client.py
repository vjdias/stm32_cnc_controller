import unittest

from cnc_client import CNCClient
from cnc_protocol import (
    REQ_LED_CTRL,
    RESP_HEADER,
    RESP_LED_CTRL,
    RESP_TAIL,
    SPI_DMA_FRAME_LEN,
    SPI_DMA_HANDSHAKE_READY,
    SPI_DMA_POLL_BYTE,
)
from cnc_requests import CNCRequestBuilder


class _FakeSpi:
    def __init__(self, response):
        self.calls = []
        self.response = response

    def xfer2(self, data):
        frame = [d & 0xFF for d in data]
        self.calls.append(frame)
        if len(self.calls) == 1:
            return [SPI_DMA_HANDSHAKE_READY] * len(frame)
        prefix_len = max(0, len(frame) - len(self.response))
        return [SPI_DMA_HANDSHAKE_READY] * prefix_len + self.response[:]


class CNCClientExchangeTests(unittest.TestCase):
    def test_poll_frames_preserve_dma_prefix(self) -> None:
        client = object.__new__(CNCClient)
        response = [
            RESP_HEADER,
            RESP_LED_CTRL,
            0x02,
            0x01,
            0x00,
            0x00,
            RESP_TAIL,
        ]
        fake_spi = _FakeSpi(response)
        client.spi = fake_spi

        request = CNCRequestBuilder.led_control(2, 0x01, 0, 0)
        frame = client.exchange(REQ_LED_CTRL, request, tries=3, settle_delay_s=0.0)

        self.assertEqual(frame, response)
        self.assertGreaterEqual(len(fake_spi.calls), 2)
        self.assertLessEqual(len(fake_spi.calls), 1 + max(1, 3))

        handshake_tx = fake_spi.calls[0]
        self.assertEqual(len(handshake_tx), SPI_DMA_FRAME_LEN)
        prefix_len = len(handshake_tx) - len(request)
        self.assertTrue(prefix_len >= 0)
        self.assertEqual(handshake_tx[:prefix_len], [SPI_DMA_POLL_BYTE] * prefix_len)

        for poll_tx in fake_spi.calls[1:]:
            self.assertEqual(len(poll_tx), SPI_DMA_FRAME_LEN)
            self.assertEqual(poll_tx[:prefix_len], [SPI_DMA_POLL_BYTE] * prefix_len)


if __name__ == "__main__":
    unittest.main()
