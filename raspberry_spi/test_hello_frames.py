import unittest

from cnc_protocol import (
    REQ_HEADER,
    REQ_TAIL,
    REQ_TEST_HELLO,
    RESP_HEADER,
    RESP_TAIL,
    RESP_TEST_HELLO,
)
from cnc_requests import CNCRequestBuilder
from cnc_responses import CNCResponseDecoder


class HelloFrameTests(unittest.TestCase):
    def test_hello_request_layout(self) -> None:
        req = CNCRequestBuilder.hello()
        suffix = [ord(c) for c in "ello"]
        expected = [REQ_HEADER, REQ_TEST_HELLO] + suffix + [REQ_TAIL]
        self.assertEqual(req, expected)

    def test_hello_response_decoder(self) -> None:
        resp = [RESP_HEADER, RESP_TEST_HELLO] + [ord(c) for c in "ello"] + [RESP_TAIL]
        spec = CNCResponseDecoder.SPECS[REQ_TEST_HELLO]
        self.assertEqual(spec.length, len(resp))
        decoded = spec.decoder(resp)
        self.assertEqual(decoded["type"], RESP_TEST_HELLO)
        self.assertEqual(decoded["payload"], "hello")


if __name__ == "__main__":
    unittest.main()
