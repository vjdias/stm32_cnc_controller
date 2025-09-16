#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import argparse
from typing import Dict, Any, Optional, Tuple, List

try:
    import spidev  # type: ignore
except Exception as e:  # pragma: no cover
    spidev = None


# Framing bytes
REQ_HEADER = 0xAA
REQ_TAIL = 0x55
RESP_HEADER = 0xAB
RESP_TAIL = 0x54

# Request types
REQ_MOVE_QUEUE_ADD = 0x01
REQ_MOVE_QUEUE_STATUS = 0x02
REQ_START_MOVE = 0x03
REQ_MOVE_HOME = 0x04
REQ_MOVE_PROBE_LEVEL = 0x05
REQ_MOVE_END = 0x06
REQ_LED_CTRL = 0x07
REQ_FPGA_STATUS = 0x20

# Response types
RESP_MOVE_QUEUE_ADD_ACK = 0x01
RESP_MOVE_QUEUE_STATUS = 0x02
RESP_START_MOVE = 0x03
RESP_MOVE_HOME = 0x04
RESP_MOVE_PROBE_LEVEL = 0x05
RESP_MOVE_END = 0x06
RESP_LED_CTRL = 0x07
RESP_FPGA_STATUS = 0x20
RESP_HOME_STATUS = 0x21


def xor_reduce_bytes(bs: List[int]) -> int:
    x = 0
    for b in bs:
        x ^= (b & 0xFF)
    return x & 0xFF


def xor_bit_reduce_bytes(bs: List[int]) -> int:
    x = xor_reduce_bytes(bs)
    x ^= (x >> 4)
    x ^= (x >> 2)
    x ^= (x >> 1)
    return x & 0x1


def be16_bytes(v: int) -> Tuple[int, int]:
    return ((v >> 8) & 0xFF, v & 0xFF)


def be32_bytes(v: int) -> Tuple[int, int, int, int]:
    return ((v >> 24) & 0xFF, (v >> 16) & 0xFF, (v >> 8) & 0xFF, v & 0xFF)


def parity_set_byte_1N(raw: List[int], last_index: int, parity_index: int) -> None:
    # XOR dos bytes de [1..last_index] inclusive
    raw[parity_index] = xor_reduce_bytes(raw[1:last_index + 1])


def parity_check_byte_1N(raw: List[int], last_index: int, parity_index: int) -> bool:
    return (raw[parity_index] & 0xFF) == xor_reduce_bytes(raw[1:last_index + 1])


def parity_set_bit_1N(raw: List[int], last_index: int, parity_index: int) -> None:
    raw[parity_index] = xor_bit_reduce_bytes(raw[1:last_index + 1]) & 0x1


def parity_check_bit_1N(raw: List[int], last_index: int, parity_index: int) -> bool:
    return (raw[parity_index] & 0x1) == xor_bit_reduce_bytes(raw[1:last_index + 1])


# ==========================
# Request encoders
# ==========================


def enc_led_ctrl(frame_id: int, led_mask: int, r: int, g: int, b: int) -> List[int]:
    raw = [0] * 9
    raw[0] = REQ_HEADER
    raw[1] = REQ_LED_CTRL
    raw[2] = frame_id & 0xFF
    raw[3] = led_mask & 0xFF
    raw[4] = r & 0xFF
    raw[5] = g & 0xFF
    raw[6] = b & 0xFF
    parity_set_byte_1N(raw, 6, 7)
    raw[8] = REQ_TAIL
    return raw


def enc_move_home(frame_id: int, axis_mask: int, dir_mask: int, vhome: int) -> List[int]:
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
    return raw


def enc_probe_level(frame_id: int, axis_mask: int, vprobe: int) -> List[int]:
    raw = [0] * 8
    raw[0] = REQ_HEADER
    raw[1] = REQ_MOVE_PROBE_LEVEL
    raw[2] = frame_id & 0xFF
    raw[3] = axis_mask & 0xFF
    vhi, vlo = be16_bytes(vprobe)
    raw[4], raw[5] = vhi, vlo
    parity_set_byte_1N(raw, 5, 6)
    raw[7] = REQ_TAIL
    return raw


def enc_move_queue_add(
    frame_id: int,
    dir_mask: int,
    vx: int, sx: int,
    vy: int, sy: int,
    vz: int, sz: int,
    kp_x: int, ki_x: int, kd_x: int,
    kp_y: int, ki_y: int, kd_y: int,
    kp_z: int, ki_z: int, kd_z: int,
) -> List[int]:
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
    # Paridade: bit-reduce no LSB, cobrindo bytes [1..39], armazenado em [40]
    parity_set_bit_1N(raw, 39, 40)
    raw[41] = REQ_TAIL
    return raw


def enc_start_move(frame_id: int) -> List[int]:
    return [REQ_HEADER, REQ_START_MOVE, frame_id & 0xFF, REQ_TAIL]


def enc_move_end(frame_id: int) -> List[int]:
    return [REQ_HEADER, REQ_MOVE_END, frame_id & 0xFF, REQ_TAIL]


def enc_queue_status(frame_id: int) -> List[int]:
    return [REQ_HEADER, REQ_MOVE_QUEUE_STATUS, frame_id & 0xFF, REQ_TAIL]


def enc_fpga_status(frame_id: int) -> List[int]:
    return [REQ_HEADER, REQ_FPGA_STATUS, frame_id & 0xFF, REQ_TAIL]


# ==========================
# Response decoders (host-side)
# ==========================


def _require_frame(raw: List[int], header: int, tail: int, min_len: int) -> None:
    if not raw or len(raw) < min_len or raw[0] != header or raw[-1] != tail:
        raise ValueError("Frame inválido ou incompleto")


def dec_led_resp(raw: List[int]) -> Dict[str, Any]:
    _require_frame(raw, RESP_HEADER, RESP_TAIL, 7)
    if raw[1] != RESP_LED_CTRL or not parity_check_byte_1N(raw, 4, 5):
        raise ValueError("LED response inválida/paridade")
    return {"type": raw[1], "frameId": raw[2], "ledMask": raw[3], "status": raw[4]}


def dec_queue_add_ack(raw: List[int]) -> Dict[str, Any]:
    _require_frame(raw, RESP_HEADER, RESP_TAIL, 6)
    if raw[1] != RESP_MOVE_QUEUE_ADD_ACK or not parity_check_bit_1N(raw, 3, 4):
        raise ValueError("QueueAdd ACK inválida/paridade")
    return {"type": raw[1], "frameId": raw[2], "status": raw[3]}


def dec_queue_status(raw: List[int]) -> Dict[str, Any]:
    _require_frame(raw, RESP_HEADER, RESP_TAIL, 12)
    if raw[1] != RESP_MOVE_QUEUE_STATUS or not parity_check_bit_1N(raw, 9, 10):
        raise ValueError("QueueStatus inválida/paridade")
    return {
        "type": raw[1], "frameId": raw[2], "status": raw[3],
        "pidErrX": raw[4], "pidErrY": raw[5], "pidErrZ": raw[6],
        "pctX": raw[7], "pctY": raw[8], "pctZ": raw[9],
    }


def dec_start_move(raw: List[int]) -> Dict[str, Any]:
    _require_frame(raw, RESP_HEADER, RESP_TAIL, 4)
    if raw[1] != RESP_START_MOVE:
        raise ValueError("StartMove inválida")
    return {"type": raw[1], "frameId": raw[2]}


def dec_move_end(raw: List[int]) -> Dict[str, Any]:
    _require_frame(raw, RESP_HEADER, RESP_TAIL, 4)
    if raw[1] != RESP_MOVE_END:
        raise ValueError("MoveEnd inválida")
    return {"type": raw[1], "frameId": raw[2]}


def dec_move_home(raw: List[int]) -> Dict[str, Any]:
    _require_frame(raw, RESP_HEADER, RESP_TAIL, 8)
    if raw[1] != RESP_MOVE_HOME or not parity_check_byte_1N(raw, 5, 6):
        raise ValueError("MoveHome inválida/paridade")
    return {
        "type": raw[1], "frameId": raw[2], "status": raw[3],
        "axisHomeMask": raw[4], "errorFlags": raw[5],
    }


def dec_probe_level(raw: List[int]) -> Dict[str, Any]:
    _require_frame(raw, RESP_HEADER, RESP_TAIL, 20)
    if raw[1] != RESP_MOVE_PROBE_LEVEL or not parity_check_byte_1N(raw, 17, 18):
        raise ValueError("ProbeLevel inválida/paridade")
    def be32_read(i: int) -> int:
        return (raw[i] << 24) | (raw[i+1] << 16) | (raw[i+2] << 8) | raw[i+3]
    return {
        "type": raw[1], "frameId": raw[2], "status": raw[3],
        "axisDoneMask": raw[4], "errorFlags": raw[5],
        "latchedPosX": be32_read(6), "latchedPosY": be32_read(10), "latchedPosZ": be32_read(14),
    }


def dec_home_status(raw: List[int]) -> Dict[str, Any]:
    _require_frame(raw, RESP_HEADER, RESP_TAIL, 18)
    if raw[1] != RESP_HOME_STATUS or not parity_check_byte_1N(raw, 15, 16):
        raise ValueError("HomeStatus inválida/paridade")
    def be16_read(i: int) -> int:
        return (raw[i] << 8) | raw[i+1]
    return {
        "type": raw[1], "frameId": raw[2], "axisMask": raw[3],
        "posRelX": be16_read(4), "homeOffX": be16_read(6),
        "posRelY": be16_read(8), "homeOffY": be16_read(10),
        "posRelZ": be16_read(12), "homeOffZ": be16_read(14),
    }


RESP_SPECS: Dict[int, Tuple[int, Any]] = {
    REQ_LED_CTRL: (7, dec_led_resp),
    REQ_MOVE_QUEUE_ADD: (6, dec_queue_add_ack),
    REQ_MOVE_QUEUE_STATUS: (12, dec_queue_status),
    REQ_START_MOVE: (4, dec_start_move),
    REQ_MOVE_HOME: (8, dec_move_home),
    REQ_MOVE_PROBE_LEVEL: (20, dec_probe_level),
    REQ_MOVE_END: (4, dec_move_end),
    # REQ_FPGA_STATUS: (?, dec_fpga_status)  # não definido no firmware atual
}


class CNCClient:
    def __init__(self, bus: int = 0, dev: int = 0, speed_hz: int = 1_000_000, mode: int = 0b11):
        if spidev is None:
            raise RuntimeError("spidev não disponível. Instale `python3-spidev` no Raspberry.")
        self.spi = spidev.SpiDev()
        self.spi.open(bus, dev)
        self.spi.max_speed_hz = int(speed_hz)
        self.spi.mode = mode  # MODE 3: 0b11
        self.spi.bits_per_word = 8

    def close(self) -> None:
        try:
            self.spi.close()
        except Exception:
            pass

    def _xfer(self, data: List[int]) -> List[int]:
        return self.spi.xfer2([d & 0xFF for d in data])

    def exchange(self, req: List[int], expected_type: int, expected_len: int,
                 tries: int = 8, settle_delay_s: float = 0.001) -> List[int]:
        # Envia request (não depende do retorno full-duplex)
        self._xfer(req)
        time.sleep(settle_delay_s)
        # Lê resposta com clock gerado pelo master
        for _ in range(max(1, tries)):
            rx = self._xfer([0x00] * expected_len)
            # Alinha no header se necessário
            try:
                idx = rx.index(RESP_HEADER)
            except ValueError:
                time.sleep(settle_delay_s)
                continue
            if idx + expected_len <= len(rx):
                frame = rx[idx:idx + expected_len]
                if frame[0] == RESP_HEADER and frame[-1] == RESP_TAIL and frame[1] == expected_type:
                    return frame
            time.sleep(settle_delay_s)
        raise TimeoutError("Resposta SPI não recebida/validada no prazo.")

    def read_boot_hello(self, tries: int = 16, settle_delay_s: float = 0.002,
                         chunk_len: int = 32) -> List[int]:
        """Lê o frame de teste "AB 'hello' 54" enfileirado no boot do STM32.
        Gera clocks (dummy bytes) e procura pelo padrão no fluxo de MISO.
        Retorna a lista de bytes do frame encontrado.
        """
        pattern = [RESP_HEADER, ord('h'), ord('e'), ord('l'), ord('l'), ord('o'), RESP_TAIL]
        for _ in range(max(1, tries)):
            rx = self._xfer([0x00] * chunk_len)
            # Procura header e tail subsequentes dentro de janela curta
            for i, b in enumerate(rx):
                if b == RESP_HEADER:
                    end = i + len(pattern)
                    if end <= len(rx):
                        candidate = rx[i:end]
                        if candidate == pattern:
                            return candidate
            time.sleep(settle_delay_s)
        raise TimeoutError("Frame 'hello' não encontrado. Reinicie o STM32 e tente novamente.")

    def print_until_zero_after_activity(self, chunk_len: int = 32,
                                        settle_delay_s: float = 0.0) -> None:
        """Gera clocks e imprime os bytes recebidos.
        Ao detectar qualquer byte != 0x00, continua imprimindo até
        que um 0x00 seja recebido (em qualquer posição do chunk).
        """
        saw_activity = False
        while True:
            rx = self._xfer([0x00] * chunk_len)
            # imprime linha em hex
            print(' '.join(f"{b:02X}" for b in rx))
            if any(b != 0x00 for b in rx):
                saw_activity = True
            if saw_activity and any(b == 0x00 for b in rx):
                break
            if settle_delay_s > 0:
                time.sleep(settle_delay_s)


def _common_args(p: argparse.ArgumentParser) -> None:
    p.add_argument("--bus", type=int, default=0)
    p.add_argument("--dev", type=int, default=0)
    p.add_argument("--speed", type=int, default=1_000_000)


def main() -> int:
    ap = argparse.ArgumentParser(description="Cliente SPI (Raspberry) para CNC_Controller (STM32 SPI1 Slave)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    # LED
    ap_led = sub.add_parser("led", help="LED RGB control")
    _common_args(ap_led)
    ap_led.add_argument("--frame-id", type=int, required=True)
    ap_led.add_argument("--mask", type=lambda x: int(x, 0), required=True)
    ap_led.add_argument("--r", type=int, default=0)
    ap_led.add_argument("--g", type=int, default=0)
    ap_led.add_argument("--b", type=int, default=0)

    # Queue Add (parâmetros principais; demais com padrão 0)
    ap_qadd = sub.add_parser("queue-add", help="Adicionar movimento à fila")
    _common_args(ap_qadd)
    ap_qadd.add_argument("--frame-id", type=int, required=True)
    ap_qadd.add_argument("--dir", type=lambda x: int(x, 0), required=True)
    ap_qadd.add_argument("--vx", type=int, required=True)
    ap_qadd.add_argument("--sx", type=int, required=True)
    ap_qadd.add_argument("--vy", type=int, required=True)
    ap_qadd.add_argument("--sy", type=int, required=True)
    ap_qadd.add_argument("--vz", type=int, required=True)
    ap_qadd.add_argument("--sz", type=int, required=True)
    for axis in ("x", "y", "z"):
        ap_qadd.add_argument(f"--kp-{axis}", type=int, default=0)
        ap_qadd.add_argument(f"--ki-{axis}", type=int, default=0)
        ap_qadd.add_argument(f"--kd-{axis}", type=int, default=0)

    # Queue Status
    ap_qst = sub.add_parser("queue-status", help="Consultar status da fila")
    _common_args(ap_qst)
    ap_qst.add_argument("--frame-id", type=int, required=True)

    # Start/End move
    ap_sm = sub.add_parser("start-move", help="Iniciar execução")
    _common_args(ap_sm)
    ap_sm.add_argument("--frame-id", type=int, required=True)

    ap_em = sub.add_parser("end-move", help="Finalizar execução")
    _common_args(ap_em)
    ap_em.add_argument("--frame-id", type=int, required=True)

    # Home
    ap_home = sub.add_parser("home", help="Sequência de homing")
    _common_args(ap_home)
    ap_home.add_argument("--frame-id", type=int, required=True)
    ap_home.add_argument("--axes", type=lambda x: int(x, 0), required=True)
    ap_home.add_argument("--dirs", type=lambda x: int(x, 0), required=True)
    ap_home.add_argument("--vhome", type=lambda x: int(x, 0), required=True)

    # Probe level
    ap_probe = sub.add_parser("probe-level", help="Sequência de probe level")
    _common_args(ap_probe)
    ap_probe.add_argument("--frame-id", type=int, required=True)
    ap_probe.add_argument("--axes", type=lambda x: int(x, 0), required=True)
    ap_probe.add_argument("--vprobe", type=lambda x: int(x, 0), required=True)

    # Hello test (boot frame AB 'hello' 54)
    ap_hello = sub.add_parser("hello", help="Ler frame de teste 'hello' do STM32 (enfileirado no boot)")
    _common_args(ap_hello)

    args = ap.parse_args()

    client = CNCClient(bus=args.bus, dev=args.dev, speed_hz=args.speed)
    try:
        if args.cmd == "led":
            req = enc_led_ctrl(args["frame_id"] if isinstance(args, dict) else args.frame_id,
                               args.mask, args.r, args.g, args.b)
            exp_len, decoder = RESP_SPECS[REQ_LED_CTRL]
            resp = client.exchange(req, RESP_LED_CTRL, exp_len)
            print(decoder(resp))

        elif args.cmd == "queue-add":
            req = enc_move_queue_add(
                args.frame_id, args.dir,
                args.vx, args.sx, args.vy, args.sy, args.vz, args.sz,
                args.kp_x, args.ki_x, args.kd_x,
                args.kp_y, args.ki_y, args.kd_y,
                args.kp_z, args.ki_z, args.kd_z)
            exp_len, decoder = RESP_SPECS[REQ_MOVE_QUEUE_ADD]
            resp = client.exchange(req, RESP_MOVE_QUEUE_ADD_ACK, exp_len)
            print(decoder(resp))

        elif args.cmd == "queue-status":
            req = enc_queue_status(args.frame_id)
            exp_len, decoder = RESP_SPECS[REQ_MOVE_QUEUE_STATUS]
            resp = client.exchange(req, RESP_MOVE_QUEUE_STATUS, exp_len)
            print(decoder(resp))

        elif args.cmd == "start-move":
            req = enc_start_move(args.frame_id)
            exp_len, decoder = RESP_SPECS[REQ_START_MOVE]
            resp = client.exchange(req, RESP_START_MOVE, exp_len)
            print(decoder(resp))

        elif args.cmd == "end-move":
            req = enc_move_end(args.frame_id)
            exp_len, decoder = RESP_SPECS[REQ_MOVE_END]
            resp = client.exchange(req, RESP_MOVE_END, exp_len)
            print(decoder(resp))

        elif args.cmd == "home":
            req = enc_move_home(args.frame_id, args.axes, args.dirs, args.vhome)
            exp_len, decoder = RESP_SPECS[REQ_MOVE_HOME]
            resp = client.exchange(req, RESP_MOVE_HOME, exp_len)
            print(decoder(resp))

        elif args.cmd == "probe-level":
            req = enc_probe_level(args.frame_id, args.axes, args.vprobe)
            exp_len, decoder = RESP_SPECS[REQ_MOVE_PROBE_LEVEL]
            resp = client.exchange(req, RESP_MOVE_PROBE_LEVEL, exp_len)
            print(decoder(resp))

        elif args.cmd == "hello":
            # Imprime continuamente o que o STM32 coloca no MISO.
            # Ao detectar bytes != 0x00, segue imprimindo até encontrar 0x00.
            client.print_until_zero_after_activity()

        else:
            raise SystemExit(2)

    finally:
        client.close()

    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
