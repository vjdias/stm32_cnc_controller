"""Decodificadores de respostas do protocolo CNC SPI."""

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_protocol import (
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        RESP_HEADER,
        RESP_HOME_STATUS,
        RESP_LED_CTRL,
        RESP_MOVE_END,
        RESP_MOVE_HOME,
        RESP_MOVE_PROBE_LEVEL,
        RESP_MOVE_QUEUE_ADD_ACK,
        RESP_MOVE_QUEUE_STATUS,
        RESP_START_MOVE,
        RESP_TAIL,
        parity_check_bit_1N,
        parity_check_byte_1N,
    )
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_protocol import (  # type: ignore
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        RESP_HEADER,
        RESP_HOME_STATUS,
        RESP_LED_CTRL,
        RESP_MOVE_END,
        RESP_MOVE_HOME,
        RESP_MOVE_PROBE_LEVEL,
        RESP_MOVE_QUEUE_ADD_ACK,
        RESP_MOVE_QUEUE_STATUS,
        RESP_START_MOVE,
        RESP_TAIL,
        parity_check_bit_1N,
        parity_check_byte_1N,
    )


@dataclass(frozen=True)
class ResponseSpec:
    response_type: int
    length: int
    decoder: Callable[[List[int]], Dict[str, Any]]


class CNCResponseDecoder:
    """Decodificadores das mensagens recebidas do STM32."""

    @staticmethod
    def _require_frame(raw: List[int], header: int, tail: int, min_len: int) -> None:
        if not raw or len(raw) < min_len or raw[0] != header or raw[-1] != tail:
            raise ValueError("Frame inválido ou incompleto")

    @staticmethod
    def led(raw: List[int]) -> Dict[str, Any]:
        CNCResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 7)
        if raw[1] != RESP_LED_CTRL or not parity_check_byte_1N(raw, 4, 5):
            raise ValueError("LED response inválida/paridade")
        return {"type": raw[1], "frameId": raw[2], "ledMask": raw[3], "status": raw[4]}

    @staticmethod
    def queue_add_ack(raw: List[int]) -> Dict[str, Any]:
        CNCResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 6)
        if raw[1] != RESP_MOVE_QUEUE_ADD_ACK or not parity_check_bit_1N(raw, 3, 4):
            raise ValueError("QueueAdd ACK inválida/paridade")
        return {"type": raw[1], "frameId": raw[2], "status": raw[3]}

    @staticmethod
    def queue_status(raw: List[int]) -> Dict[str, Any]:
        CNCResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 12)
        if raw[1] != RESP_MOVE_QUEUE_STATUS or not parity_check_bit_1N(raw, 9, 10):
            raise ValueError("QueueStatus inválida/paridade")
        return {
            "type": raw[1],
            "frameId": raw[2],
            "status": raw[3],
            "pidErrX": raw[4],
            "pidErrY": raw[5],
            "pidErrZ": raw[6],
            "pctX": raw[7],
            "pctY": raw[8],
            "pctZ": raw[9],
        }

    @staticmethod
    def start_move(raw: List[int]) -> Dict[str, Any]:
        CNCResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 4)
        if raw[1] != RESP_START_MOVE:
            raise ValueError("StartMove inválida")
        return {"type": raw[1], "frameId": raw[2]}

    @staticmethod
    def move_end(raw: List[int]) -> Dict[str, Any]:
        CNCResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 4)
        if raw[1] != RESP_MOVE_END:
            raise ValueError("MoveEnd inválida")
        return {"type": raw[1], "frameId": raw[2]}

    @staticmethod
    def move_home(raw: List[int]) -> Dict[str, Any]:
        CNCResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 8)
        if raw[1] != RESP_MOVE_HOME or not parity_check_byte_1N(raw, 5, 6):
            raise ValueError("MoveHome inválida/paridade")
        return {
            "type": raw[1],
            "frameId": raw[2],
            "status": raw[3],
            "axisHomeMask": raw[4],
            "errorFlags": raw[5],
        }

    @staticmethod
    def probe_level(raw: List[int]) -> Dict[str, Any]:
        CNCResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 20)
        if raw[1] != RESP_MOVE_PROBE_LEVEL or not parity_check_byte_1N(raw, 17, 18):
            raise ValueError("ProbeLevel inválida/paridade")

        def be32_read(i: int) -> int:
            return (raw[i] << 24) | (raw[i + 1] << 16) | (raw[i + 2] << 8) | raw[i + 3]

        return {
            "type": raw[1],
            "frameId": raw[2],
            "status": raw[3],
            "axisDoneMask": raw[4],
            "errorFlags": raw[5],
            "latchedPosX": be32_read(6),
            "latchedPosY": be32_read(10),
            "latchedPosZ": be32_read(14),
        }

    @staticmethod
    def home_status(raw: List[int]) -> Dict[str, Any]:
        CNCResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 18)
        if raw[1] != RESP_HOME_STATUS or not parity_check_byte_1N(raw, 15, 16):
            raise ValueError("HomeStatus inválida/paridade")

        def be16_read(i: int) -> int:
            return (raw[i] << 8) | raw[i + 1]

        return {
            "type": raw[1],
            "frameId": raw[2],
            "axisMask": raw[3],
            "posRelX": be16_read(4),
            "homeOffX": be16_read(6),
            "posRelY": be16_read(8),
            "homeOffY": be16_read(10),
            "posRelZ": be16_read(12),
            "homeOffZ": be16_read(14),
        }

    SPECS: Dict[int, ResponseSpec] = {
        REQ_LED_CTRL: ResponseSpec(RESP_LED_CTRL, 7, led.__func__),
        REQ_MOVE_QUEUE_ADD: ResponseSpec(RESP_MOVE_QUEUE_ADD_ACK, 6, queue_add_ack.__func__),
        REQ_MOVE_QUEUE_STATUS: ResponseSpec(RESP_MOVE_QUEUE_STATUS, 12, queue_status.__func__),
        REQ_START_MOVE: ResponseSpec(RESP_START_MOVE, 4, start_move.__func__),
        REQ_MOVE_HOME: ResponseSpec(RESP_MOVE_HOME, 8, move_home.__func__),
        REQ_MOVE_PROBE_LEVEL: ResponseSpec(RESP_MOVE_PROBE_LEVEL, 20, probe_level.__func__),
        REQ_MOVE_END: ResponseSpec(RESP_MOVE_END, 4, move_end.__func__),
        # REQ_FPGA_STATUS: ResponseSpec(RESP_FPGA_STATUS, ?, decoder)  # não definido no firmware atual
    }


__all__ = ["ResponseSpec", "CNCResponseDecoder"]
