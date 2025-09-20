"""Montagem de requisições para o protocolo CNC SPI."""

import sys
from pathlib import Path
from typing import List

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .cnc_protocol import (
        REQ_FPGA_STATUS,
        REQ_HEADER,
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        REQ_TAIL,
        be16_bytes,
        be32_bytes,
        pad_request,
        parity_set_bit_1N,
        parity_set_byte_1N,
    )
else:
    if str(MODULE_DIR) not in sys.path:
        sys.path.insert(0, str(MODULE_DIR))
    from cnc_protocol import (  # type: ignore
        REQ_FPGA_STATUS,
        REQ_HEADER,
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        REQ_TAIL,
        be16_bytes,
        be32_bytes,
        pad_request,
        parity_set_bit_1N,
        parity_set_byte_1N,
    )


class CNCRequestBuilder:
    """Factory centralizada das mensagens enviadas ao STM32."""

    @staticmethod
    def led_control(frame_id: int, led_mask: int, led1_mode: int, led1_freq_hz: int,
                    led2_mode: int, led2_freq_hz: int) -> List[int]:
        raw = [0] * 12
        raw[0] = REQ_HEADER
        raw[1] = REQ_LED_CTRL
        raw[2] = frame_id & 0xFF
        raw[3] = led_mask & 0xFF
        raw[4] = led1_mode & 0xFF
        f1_hi, f1_lo = be16_bytes(led1_freq_hz & 0xFFFF)
        raw[5], raw[6] = f1_hi, f1_lo
        raw[7] = led2_mode & 0xFF
        f2_hi, f2_lo = be16_bytes(led2_freq_hz & 0xFFFF)
        raw[8], raw[9] = f2_hi, f2_lo
        parity_set_byte_1N(raw, 9, 10)
        raw[11] = REQ_TAIL
        return pad_request(raw)

    @staticmethod
    def move_home(frame_id: int, axis_mask: int, dir_mask: int, vhome: int) -> List[int]:
        raw = [0] * 9
        raw[0] = REQ_HEADER
        raw[1] = REQ_MOVE_HOME
        raw[2] = frame_id & 0xFF
        raw[3] = axis_mask & 0xFF
        raw[4] = dir_mask & 0xFF
        vhi, vlo = be16_bytes(vhome)
        raw[5], raw[6] = vhi, vlo
        parity_set_byte_1N(raw, 6, 7)
        raw[8] = REQ_TAIL
        return pad_request(raw)

    @staticmethod
    def probe_level(frame_id: int, axis_mask: int, vprobe: int) -> List[int]:
        raw = [0] * 8
        raw[0] = REQ_HEADER
        raw[1] = REQ_MOVE_PROBE_LEVEL
        raw[2] = frame_id & 0xFF
        raw[3] = axis_mask & 0xFF
        vhi, vlo = be16_bytes(vprobe)
        raw[4], raw[5] = vhi, vlo
        parity_set_byte_1N(raw, 5, 6)
        raw[7] = REQ_TAIL
        return pad_request(raw)

    @staticmethod
    def move_queue_add(frame_id: int, dir_mask: int,
                       vx: int, sx: int, vy: int, sy: int, vz: int, sz: int,
                       kp_x: int, ki_x: int, kd_x: int,
                       kp_y: int, ki_y: int, kd_y: int,
                       kp_z: int, ki_z: int, kd_z: int) -> List[int]:
        raw = [0] * 42
        raw[0] = REQ_HEADER
        raw[1] = REQ_MOVE_QUEUE_ADD
        raw[2] = frame_id & 0xFF
        raw[3] = dir_mask & 0xFF
        raw[4:6] = list(be16_bytes(vx))
        raw[6:10] = list(be32_bytes(sx))
        raw[10:12] = list(be16_bytes(vy))
        raw[12:16] = list(be32_bytes(sy))
        raw[16:18] = list(be16_bytes(vz))
        raw[18:22] = list(be32_bytes(sz))
        raw[22:24] = list(be16_bytes(kp_x))
        raw[24:26] = list(be16_bytes(ki_x))
        raw[26:28] = list(be16_bytes(kd_x))
        raw[28:30] = list(be16_bytes(kp_y))
        raw[30:32] = list(be16_bytes(ki_y))
        raw[32:34] = list(be16_bytes(kd_y))
        raw[34:36] = list(be16_bytes(kp_z))
        raw[36:38] = list(be16_bytes(ki_z))
        raw[38:40] = list(be16_bytes(kd_z))
        parity_set_bit_1N(raw, 39, 40)
        raw[41] = REQ_TAIL
        return pad_request(raw)

    @staticmethod
    def start_move(frame_id: int) -> List[int]:
        raw = [REQ_HEADER, REQ_START_MOVE, frame_id & 0xFF, REQ_TAIL]
        return pad_request(raw)

    @staticmethod
    def move_end(frame_id: int) -> List[int]:
        raw = [REQ_HEADER, REQ_MOVE_END, frame_id & 0xFF, REQ_TAIL]
        return pad_request(raw)

    @staticmethod
    def queue_status(frame_id: int) -> List[int]:
        raw = [REQ_HEADER, REQ_MOVE_QUEUE_STATUS, frame_id & 0xFF, REQ_TAIL]
        return pad_request(raw)

    @staticmethod
    def fpga_status(frame_id: int) -> List[int]:
        raw = [REQ_HEADER, REQ_FPGA_STATUS, frame_id & 0xFF, REQ_TAIL]
        return pad_request(raw)


__all__ = ["CNCRequestBuilder"]
