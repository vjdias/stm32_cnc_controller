"""Montagem de requisições para o protocolo STM32 (CNC SPI)."""

import sys
from pathlib import Path
from typing import List

MODULE_DIR = Path(__file__).resolve().parent

if __package__:
    from .stm32_protocol import (
        REQ_STM32_STATUS,
        REQ_HEADER,
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        REQ_TEST_HELLO,
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
    from stm32_protocol import (  # type: ignore
        REQ_STM32_STATUS,
        REQ_HEADER,
        REQ_LED_CTRL,
        REQ_MOVE_END,
        REQ_MOVE_HOME,
        REQ_MOVE_PROBE_LEVEL,
        REQ_MOVE_QUEUE_ADD,
        REQ_MOVE_QUEUE_STATUS,
        REQ_START_MOVE,
        REQ_TEST_HELLO,
        REQ_TAIL,
        be16_bytes,
        be32_bytes,
        pad_request,
        parity_set_bit_1N,
        parity_set_byte_1N,
    )


class STM32RequestBuilder:
    """Factory centralizada das mensagens enviadas ao STM32."""

    @staticmethod
    def led_control(
        frame_id: int, led_mask: int, led1_mode: int, led1_freq_centihz: int
    ) -> List[int]:
        raw = [0] * 9
        raw[0] = REQ_HEADER
        raw[1] = REQ_LED_CTRL
        raw[2] = frame_id & 0xFF
        raw[3] = led_mask & 0xFF
        raw[4] = led1_mode & 0xFF
        f1_hi, f1_lo = be16_bytes(led1_freq_centihz & 0xFFFF)
        raw[5], raw[6] = f1_hi, f1_lo
        parity_set_byte_1N(raw, 6, 7)
        raw[8] = REQ_TAIL
        return pad_request(raw)

    @staticmethod
    def hello() -> List[int]:
        suffix = [ord(c) for c in "ello"]
        return [REQ_HEADER, REQ_TEST_HELLO] + suffix + [REQ_TAIL]

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
        # OBSERVAÇÃO:
        # - O firmware atual espera kp/ki/kd como inteiros de 16 bits JÁ
        #   escalados (Q8). Não há multiplicação por 256 no destino.
        # - FUTURO: um request alternativo poderia carregar ganhos em 32 bits
        #   com escala 10^4 (analogia ao LED em centi-Hz) ou IEEE‑754, mantendo
        #   este request para compatibilidade.
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
        raw = [REQ_HEADER, REQ_STM32_STATUS, frame_id & 0xFF, REQ_TAIL]
        return pad_request(raw)

    @staticmethod
    def set_origin(frame_id: int, axis_mask: int, mode: int = 0) -> List[int]:
        """Solicita ao firmware fixar g_encoder_origin = g_encoder_position para os eixos no mask.
        mode: 0=start, 1=initial (semântica definida no firmware)."""
        raw = [0] * 6
        raw[0] = REQ_HEADER
        raw[1] = REQ_SET_ORIGIN
        raw[2] = frame_id & 0xFF
        raw[3] = axis_mask & 0x07
        raw[4] = mode & 0xFF
        raw[5] = REQ_TAIL
        return pad_request(raw)

    @staticmethod
    def encoder_status(frame_id: int) -> List[int]:
        """Solicita ao firmware posições de encoder e erros PID atuais.
        Estrutura da resposta definida em STM32ResponseDecoder."""
        raw = [REQ_HEADER, REQ_ENCODER_STATUS, frame_id & 0xFF, REQ_TAIL]
        return pad_request(raw)

    @staticmethod
    def set_microsteps(frame_id: int, microsteps: int) -> List[int]:
        raw = [0] * 5
        raw[0] = REQ_HEADER
        raw[1] = REQ_SET_MICROSTEPS
        raw[2] = frame_id & 0xFF
        raw[3] = microsteps & 0xFF
        raw[4] = REQ_TAIL
        return pad_request(raw)

    @staticmethod
    def set_microsteps_axes(frame_id: int, ms_x: int, ms_y: int, ms_z: int) -> List[int]:
        raw = [0] * 7
        raw[0] = REQ_HEADER
        raw[1] = REQ_SET_MICROSTEPS_AX
        raw[2] = frame_id & 0xFF
        raw[3] = ms_x & 0xFF
        raw[4] = ms_y & 0xFF
        raw[5] = ms_z & 0xFF
        raw[6] = REQ_TAIL
        return pad_request(raw)


__all__ = ["STM32RequestBuilder"]
