"""Decodificadores de respostas do protocolo STM32 (CNC SPI)."""

import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Callable, Dict, List

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .stm32_protocol import (
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        REQ_TEST_HELLO,
        REQ_SET_ORIGIN,
        REQ_ENCODER_STATUS,
        RESP_HEADER,
        RESP_HOME_STATUS,
        RESP_LED_CTRL,
        RESP_MOVE_END,
        RESP_MOVE_HOME,
        RESP_MOVE_PROBE_LEVEL,
        RESP_MOVE_QUEUE_ADD_ACK,
        RESP_MOVE_QUEUE_STATUS,
        RESP_START_MOVE,
        RESP_TEST_HELLO,
        RESP_SET_ORIGIN,
        RESP_ENCODER_STATUS,
        RESP_TAIL,
        parity_check_bit_1N,
        parity_check_byte_1N,
    )
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from stm32_protocol import (  # type: ignore
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        REQ_TEST_HELLO,
        REQ_SET_ORIGIN,
        REQ_ENCODER_STATUS,
        RESP_HEADER,
        RESP_HOME_STATUS,
        RESP_LED_CTRL,
        RESP_MOVE_END,
        RESP_MOVE_HOME,
        RESP_MOVE_PROBE_LEVEL,
        RESP_MOVE_QUEUE_ADD_ACK,
        RESP_MOVE_QUEUE_STATUS,
        RESP_START_MOVE,
        RESP_TEST_HELLO,
        RESP_SET_ORIGIN,
        RESP_ENCODER_STATUS,
        RESP_TAIL,
        parity_check_bit_1N,
        parity_check_byte_1N,
    )


@dataclass(frozen=True)
class ResponseSpec:
    response_type: int
    length: int
    decoder: Callable[[List[int]], Dict[str, Any]]


class STM32ResponseDecoder:
    """Decodificadores das mensagens recebidas do STM32."""

    @staticmethod
    def _require_frame(raw: List[int], header: int, tail: int, min_len: int) -> None:
        if not raw or len(raw) < min_len or raw[0] != header or raw[-1] != tail:
            raise ValueError("Frame inválido ou incompleto")

    @staticmethod
    def led(raw: List[int]) -> Dict[str, Any]:
        STM32ResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 7)
        if raw[1] != RESP_LED_CTRL or not parity_check_byte_1N(raw, 4, 5):
            raise ValueError("LED response inválida/paridade")
        return {"type": raw[1], "frameId": raw[2], "ledMask": raw[3], "status": raw[4]}

    @staticmethod
    def hello(raw: List[int]) -> Dict[str, Any]:
        STM32ResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 7)
        if raw[1] != RESP_TEST_HELLO:
            raise ValueError("Hello response inválida")
        suffix = [ord(c) for c in "ello"]
        if raw[2 : 2 + len(suffix)] != suffix:
            raise ValueError("Hello payload inválido")
        payload = "".join([chr(raw[1])] + [chr(b) for b in raw[2:-1]])
        return {"type": raw[1], "payload": payload}

    @staticmethod
    def queue_add_ack(raw: List[int]) -> Dict[str, Any]:
        STM32ResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 6)
        if raw[1] != RESP_MOVE_QUEUE_ADD_ACK or not parity_check_bit_1N(raw, 3, 4):
            raise ValueError("QueueAdd ACK inválida/paridade")
        return {"type": raw[1], "frameId": raw[2], "status": raw[3]}

    @staticmethod
    def start_move(raw: List[int]) -> Dict[str, Any]:
        # Aceita 4 (legado), 5 (status) ou 6 bytes (status + depth)
        if not raw or raw[0] != RESP_HEADER or raw[-1] != RESP_TAIL or raw[1] != RESP_START_MOVE:
            raise ValueError("StartMove inválida")
        status = raw[3] if len(raw) >= 5 else 0
        depth = raw[4] if len(raw) >= 6 else None
        out = {"type": raw[1], "frameId": raw[2], "status": status}
        if depth is not None:
            out["depth"] = depth
        return out

    @staticmethod
    def move_end(raw: List[int]) -> Dict[str, Any]:
        # Aceita 4 (legado) ou 5 bytes (com status)
        if not raw or raw[0] != RESP_HEADER or raw[-1] != RESP_TAIL or raw[1] != RESP_MOVE_END:
            raise ValueError("MoveEnd inválida")
        status = raw[3] if len(raw) >= 5 else 0
        return {"type": raw[1], "frameId": raw[2], "status": status}

    SPECS: Dict[int, ResponseSpec] = {
        REQ_LED_CTRL: ResponseSpec(RESP_LED_CTRL, 7, led.__func__),
        REQ_MOVE_QUEUE_ADD: ResponseSpec(RESP_MOVE_QUEUE_ADD_ACK, 6, queue_add_ack.__func__),
        # Comprimento esperado usado como pista; extração é tolerante (4, 5 ou 6)
        REQ_START_MOVE: ResponseSpec(RESP_START_MOVE, 6, start_move.__func__),
        REQ_MOVE_END: ResponseSpec(RESP_MOVE_END, 5, move_end.__func__),
        # Placeholder comprimentos alinhados com os decoders abaixo.
        # set_origin: [HDR,TYPE,frameId, x0(4), y0(4), z0(4), TAIL] => 16 bytes
        REQ_SET_ORIGIN: ResponseSpec(RESP_SET_ORIGIN, 16, None),
        # encoder_status: [HDR,TYPE,frameId, pidX, pidY, pidZ, delta, absX(4), absY(4), absZ(4), TAIL] => 20 bytes
        REQ_ENCODER_STATUS: ResponseSpec(RESP_ENCODER_STATUS, 20, None),
        # Outros tipos podem ser acrescentados aqui quando necessários.
    }

    @staticmethod
    def set_origin(raw: List[int]) -> Dict[str, Any]:
        # Espera 16 bytes: [HDR,TYPE,frameId,x0(4),y0(4),z0(4),TAIL]
        STM32ResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 16)
        if raw[1] != RESP_SET_ORIGIN:
            raise ValueError("SetOrigin resp inválida")
        def be32(i: int) -> int:
            return (raw[i] << 24) | (raw[i+1] << 16) | (raw[i+2] << 8) | raw[i+3]
        return {
            "type": raw[1],
            "frameId": raw[2],
            "x0": be32(3),
            "y0": be32(7),
            "z0": be32(11),
        }

    @staticmethod
    def encoder_status(raw: List[int]) -> Dict[str, Any]:
        # Espera 20 bytes: [HDR,TYPE,frameId,pidX,pidY,pidZ,delta,absX(4),absY(4),absZ(4),TAIL]
        STM32ResponseDecoder._require_frame(raw, RESP_HEADER, RESP_TAIL, 20)
        if raw[1] != RESP_ENCODER_STATUS:
            raise ValueError("EncoderStatus resp inválida")
        def be32(i: int) -> int:
            return (raw[i] << 24) | (raw[i+1] << 16) | (raw[i+2] << 8) | raw[i+3]
        def s8(b: int) -> int:
            b &= 0xFF
            return b - 256 if b > 127 else b
        return {
            "type": raw[1],
            "frameId": raw[2],
            "pidErr": {"x": s8(raw[3]), "y": s8(raw[4]), "z": s8(raw[5])},
            "delta": raw[6],
            "abs": {"x": be32(7), "y": be32(11), "z": be32(15)},
        }


__all__ = ["ResponseSpec", "STM32ResponseDecoder"]
