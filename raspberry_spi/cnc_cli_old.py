#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CLI 'cnc_cli' para executar G-code (G0/G1) via STM32 + TMC5160 com monitoramento e sinalização por LED.

Estados do LED (LED1 via STM32):
 - Em execução normal: pisca (modo=2) a 1 Hz
 - Falha: aceso (modo=1)
 - Sucesso: apagado (modo=0)

Observações sobre o G-code aceito (mínimo):
 - Suporta apenas movimentos lineares G0/G1 com eixos X/Y/Z e feed F.
 - Assume unidades em milímetros; F interpretado como mm/s (fallback: --feed).
 - Modos G90 (absoluto) e G91 (relativo) reconhecidos. Demais códigos são ignorados.
"""
from __future__ import annotations

import argparse
import logging
from datetime import datetime
import time
import json
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

try:
    from .stm32.stm32_cli import STM32Client  # type: ignore
    from .stm32.stm32_requests import STM32RequestBuilder  # type: ignore
    from .stm32.stm32_responses import STM32ResponseDecoder  # type: ignore
    from .stm32.stm32_protocol import (
        REQ_MOVE_QUEUE_ADD,
        REQ_START_MOVE,
        REQ_MOVE_END,
        REQ_SET_ORIGIN,
        REQ_ENCODER_STATUS,
        REQ_SET_MICROSTEPS,
    )  # type: ignore
    from .tmc5160.tmc5160 import (
        TMC5160Configurator,
        TMC5160RegisterPreset,
        REG_GSTAT,
        REG_DRV_STATUS,
        REG_IHOLD_IRUN,
        REG_CHOPCONF,
        REG_PWMCONF,
    )  # type: ignore
except Exception:  # execução direta fora do pacote
    from raspberry_spi.stm32.stm32_cli import STM32Client  # type: ignore
    from raspberry_spi.stm32.stm32_requests import STM32RequestBuilder  # type: ignore
    from raspberry_spi.stm32.stm32_responses import STM32ResponseDecoder  # type: ignore
    from raspberry_spi.stm32.stm32_protocol import (  # type: ignore
        REQ_MOVE_QUEUE_ADD,
        REQ_START_MOVE,
        REQ_MOVE_END,
        REQ_SET_ORIGIN,
        REQ_ENCODER_STATUS,
        REQ_SET_MICROSTEPS,
    )
    from raspberry_spi.tmc5160.tmc5160 import (  # type: ignore
        TMC5160Configurator,
        TMC5160RegisterPreset,
        REG_GSTAT,
        REG_DRV_STATUS,
        REG_IHOLD_IRUN,
        REG_CHOPCONF,
        REG_PWMCONF,
    )


# ---------------- G-code mínimo (G0/G1) ----------------
@dataclass
class Triangle:
    # Mantido apenas para compatibilidade de imports antigos (não usado)
    v1: Tuple[float, float, float]
    v2: Tuple[float, float, float]
    v3: Tuple[float, float, float]


def parse_gcode_linear(path: Path, *, default_feed: float) -> Tuple[List["Segment"], Tuple[float,float,float,float,float,float]]:
    """Extrai segmentos lineares de um arquivo G-code.

    Suporta G0/G1 com X/Y/Z (float) e F (feed). Assume mm/s no F; quando ausente,
    usa default_feed. Reconhece G90/G91 (absoluto/relativo). Ignora demais comandos.

    Retorna (segments, (x0,x1,y0,y1,z0,z1)).
    """
    segs: List[Segment] = []
    # Posição atual e feed corrente
    x = y = z = 0.0
    feed = float(default_feed)
    abs_mode = True  # G90
    min_x = max_x = x
    min_y = max_y = y
    min_z = max_z = z

    def update_bbox(nx: float, ny: float, nz: float) -> None:
        nonlocal min_x, max_x, min_y, max_y, min_z, max_z
        min_x = min(min_x, nx); max_x = max(max_x, nx)
        min_y = min(min_y, ny); max_y = max(max_y, ny)
        min_z = min(min_z, nz); max_z = max(max_z, nz)

    for raw in path.read_text(errors="replace").splitlines():
        line = raw.strip()
        if not line:
            continue
        # Remove comentários iniciados por ';' ou linha inteira entre parênteses
        if line.startswith("("):
            continue
        if ";" in line:
            line = line.split(";", 1)[0].strip()
        if not line:
            continue
        up = line.upper()
        # Modos
        if "G90" in up:
            abs_mode = True
            # Continua para permitir G1 na mesma linha (raro)
        if "G91" in up:
            abs_mode = False
        # Movimento linear
        if ("G0" in up) or ("G1" in up) or ("G00" in up) or ("G01" in up):
            nx, ny, nz = x, y, z
            nf = feed
            # Tokeniza por espaços
            for tok in up.split():
                if not tok:
                    continue
                c = tok[0]
                try:
                    val = float(tok[1:])
                except Exception:
                    continue
                if c == "X":
                    nx = (val if abs_mode else (x + val))
                elif c == "Y":
                    ny = (val if abs_mode else (y + val))
                elif c == "Z":
                    nz = (val if abs_mode else (z + val))
                elif c == "F":
                    nf = float(val)
            dx, dy, dz = (nx - x), (ny - y), (nz - z)
            if dx != 0.0 or dy != 0.0 or dz != 0.0:
                segs.append(Segment(dx=dx, dy=dy, dz=dz, feed=(nf if nf > 0 else default_feed)))
                x, y, z = nx, ny, nz
                update_bbox(x, y, z)
        # Demais linhas ignoradas
        else:
            continue

    # Garante bbox válido ao menos no ponto inicial
    bbox = (min_x, max_x, min_y, max_y, min_z, max_z)
    return segs, bbox


# ---------------- Path simples (raster) ----------------
@dataclass
class Segment:
    dx: float
    dy: float
    dz: float
    feed: float  # mm/s por eixo (mesma base)


def plan_raster(
    bb: Tuple[float, float, float, float, float, float], *, layer_h: float, line_step: float, travel: float
) -> List[Segment]:
    x0, x1, y0, y1, z0, z1 = bb
    if layer_h <= 0 or line_step <= 0:
        raise ValueError("layer_h e line_step devem ser positivos")
    path: List[Segment] = []
    z = z0
    direction = 1
    while z <= z1:
        y = y0
        while y <= y1:
            if direction > 0:
                path.append(Segment(dx=x1 - x0, dy=0, dz=0, feed=travel))
            else:
                path.append(Segment(dx=-(x1 - x0), dy=0, dz=0, feed=travel))
            # próximo traço em Y
            y_next = y + line_step
            if y_next <= y1:
                path.append(Segment(dx=0, dy=line_step, dz=0, feed=travel))
            y = y_next
            direction *= -1
        # próxima camada
        z_next = z + layer_h
        if z_next <= z1:
            path.append(Segment(dx=0, dy=0, dz=layer_h, feed=travel))
        z = z_next
    return path


# ---------------- Conversão para STM32 ----------------
def mm_to_steps(v: float, steps_per_mm: float) -> int:
    return int(round(v * steps_per_mm))


def _subdivide_with_ramp(seg: Segment, accel: float, decel: float, parts: int) -> List[Segment]:
    if accel <= 0 and decel <= 0 or parts <= 0:
        return [seg]
    # Comprimento do segmento em mm
    L = (seg.dx**2 + seg.dy**2 + seg.dz**2) ** 0.5
    if L <= 0:
        return [seg]
    a = accel if accel > 0 else 0.0
    d = decel if decel > 0 else a
    v = max(0.0, float(seg.feed))
    if a <= 0 and d <= 0 or v <= 0:
        return [seg]
    # Distâncias de rampa (perfil trapezoidal simplificado)
    s_acc = (v * v) / (2.0 * max(a, 1e-9))
    s_dec = (v * v) / (2.0 * max(d, 1e-9))
    sub: List[Segment] = []
    if s_acc + s_dec <= L:
        # Perfil trapezoidal: acc, const, dec
        # Accel: dividir s_acc em 'parts' com velocidades crescentes
        for k in range(1, parts + 1):
            frac = (s_acc / L) / parts
            sub.append(Segment(dx=seg.dx * frac, dy=seg.dy * frac, dz=seg.dz * frac, feed=v * k / parts))
        # Constante: um bloco
        s_const = L - s_acc - s_dec
        if s_const > 1e-9:
            frac_c = s_const / L
            sub.append(Segment(dx=seg.dx * frac_c, dy=seg.dy * frac_c, dz=seg.dz * frac_c, feed=v))
        # Decel: dividir s_dec em 'parts' com velocidades decrescentes
        for k in range(parts, 0, -1):
            frac = (s_dec / L) / parts
            sub.append(Segment(dx=seg.dx * frac, dy=seg.dy * frac, dz=seg.dz * frac, feed=v * k / parts))
    else:
        # Perfil triangular: calcula v_peak
        # Aproximação simétrica usando a = d = max(a,d)
        a_eff = max(a, d)
        v_peak = (2.0 * a_eff * L) ** 0.5
        # Accel
        for k in range(1, parts + 1):
            frac = (0.5 * L) / parts / L
            sub.append(Segment(dx=seg.dx * frac, dy=seg.dy * frac, dz=seg.dz * frac, feed=v_peak * k / parts))
        # Decel
        for k in range(parts, 0, -1):
            frac = (0.5 * L) / parts / L
            sub.append(Segment(dx=seg.dx * frac, dy=seg.dy * frac, dz=seg.dz * frac, feed=v_peak * k / parts))
    # Ajuste final para corrigir arredondamentos: normalizar soma de dx/dy/dz
    sum_dx = sum(s.dx for s in sub)
    sum_dy = sum(s.dy for s in sub)
    sum_dz = sum(s.dz for s in sub)
    err_dx = seg.dx - sum_dx
    err_dy = seg.dy - sum_dy
    err_dz = seg.dz - sum_dz
    if sub:
        sub[-1] = Segment(dx=sub[-1].dx + err_dx, dy=sub[-1].dy + err_dy, dz=sub[-1].dz + err_dz, feed=sub[-1].feed)
    return sub


def enqueue_segments(
    client: STM32Client,
    segments: List[Segment],
    *,
    steps_x: float,
    steps_y: float,
    steps_z: float,
    frame_id: int = 1,
    settle_delay_s: float = 0.002,
    accel: float = 0.0,
    decel: float = 0.0,
    ramp_segments: int = 8,
    pid: Optional[Tuple[int,int,int,int,int,int,int,int,int]] = None,
) -> None:
    for base in segments:
        segs = _subdivide_with_ramp(base, accel, decel, ramp_segments) if (accel > 0 or decel > 0) else [base]
        for seg in segs:
            sx = abs(mm_to_steps(seg.dx, steps_x))
            sy = abs(mm_to_steps(seg.dy, steps_y))
            sz = abs(mm_to_steps(seg.dz, steps_z))
            # Dir mask: bit0=X+, bit1=Y+, bit2=Z+ (exemplo; ajuste conforme firmware)
            dir_mask = 0
            if seg.dx > 0:
                dir_mask |= 0x01
            if seg.dy > 0:
                dir_mask |= 0x02
            if seg.dz > 0:
                dir_mask |= 0x04
            # Velocidades simples (mm/s → steps/s). Limitar 16 bits (0..65535)
            vx = min(65535, abs(mm_to_steps(seg.feed, steps_x)))
            vy = min(65535, abs(mm_to_steps(seg.feed, steps_y)))
            vz = min(65535, abs(mm_to_steps(seg.feed, steps_z)))
            # Gains PID por eixo (se fornecidos)
            if pid is None:
                kp_x = ki_x = kd_x = kp_y = ki_y = kd_y = kp_z = ki_z = kd_z = 0
            else:
                kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, kp_z, ki_z, kd_z = [int(max(0, min(0xFFFF, v))) for v in pid]

            req = STM32RequestBuilder.move_queue_add(
                frame_id,
                dir_mask,
                vx,
                sx,
                vy,
                sy,
                vz,
                sz,
                kp_x,
                ki_x,
                kd_x,
                kp_y,
                ki_y,
                kd_y,
                kp_z,
                ki_z,
                kd_z,
            )
            _ = client.exchange(REQ_MOVE_QUEUE_ADD, req, tries=3, settle_delay_s=settle_delay_s)


# ---------------- Monitoramento TMC + security off ----------------
def read_tmc_errors(bus: int, dev: int, speed_hz: int) -> List[str]:
    cfg = TMC5160Configurator(
        bus=bus, device=dev, speed_hz=speed_hz, register_preset=TMC5160RegisterPreset.default()
    )
    with cfg as driver:  # type: ignore
        g = driver.read_register(REG_GSTAT)
        d = driver.read_register(REG_DRV_STATUS)
        errs: List[str] = []
        if g.value & (1 << 2):
            errs.append("GSTAT.UV_CP")
        if g.value & (1 << 1):
            errs.append("GSTAT.DRV_ERR")
        if d.value & (1 << 26):
            errs.append("DRV_STATUS.OT")
        if d.value & (1 << 25):
            errs.append("DRV_STATUS.OTPW")
        for bit, name in ((24, "S2GA"), (23, "S2GB"), (22, "S2VSA"), (21, "S2VSB"), (20, "OLA"), (19, "OLB")):
            if d.value & (1 << bit):
                errs.append(f"DRV_STATUS.{name}")
    return errs


def tmc_security_off(bus: int, dev: int, speed_hz: int, *, freewheel: int = 1) -> None:
    cfg = TMC5160Configurator(
        bus=bus, device=dev, speed_hz=speed_hz, register_preset=TMC5160RegisterPreset(writes=tuple())
    )
    with cfg as driver:  # type: ignore
        # IHOLD/IRUN=0; CHOPCONF.TOFF=0; PWMCONF.FREEWHEEL=freewheel
        writes = [
            (REG_IHOLD_IRUN, 0x00000000),
            (REG_CHOPCONF, (driver.read_register(REG_CHOPCONF).value & ~0x0F)),
            (
                REG_PWMCONF,
                ((driver.read_register(REG_PWMCONF).value & ~(0x3 << 20)) | ((freewheel & 0x3) << 20)),
            ),
        ]
        driver.apply_registers(writes)


def tmc_apply_cfg(bus: int, dev: int, speed_hz: int, cfg: dict, logger: Optional[logging.Logger] = None) -> None:
    """Aplica configuração básica do TMC5160 a partir do JSON de cfg.

    Chaves suportadas em cfg:
      - globalscaler (int, 0 ou 32..255)
      - ihold (0..31), irun (0..31), iholddelay (0..15)
      - tpowerdown (0..255)
      - pwm: { autoscale: bool, autograd: bool, ofs: int(0..255), grad: int(0..255), freq: int(0..3), freewheel: int(0..3) }
      - chop: { toff: int(0..15) }
    """
    if not isinstance(cfg, dict):
        return
    preset = TMC5160RegisterPreset(writes=tuple())
    with TMC5160Configurator(bus=bus, device=dev, speed_hz=speed_hz, register_preset=preset) as d:  # type: ignore
        writes: List[Tuple[int,int]] = []
        gs = cfg.get("globalscaler")
        if gs is not None:
            gsv = int(gs)
            if gsv == 0 or 32 <= gsv <= 255:
                writes.append((0x0B, gsv & 0xFF))
        ihold = cfg.get("ihold")
        irun = cfg.get("irun")
        ihd = cfg.get("iholddelay", cfg.get("ihold_delay"))
        if ihold is not None or irun is not None or ihd is not None:
            _ih = max(0, min(31, int(0 if ihold is None else ihold)))
            _ir = max(0, min(31, int(0 if irun is None else irun)))
            _dl = max(0, min(15, int(6 if ihd is None else ihd)))
            writes.append((0x10, (_ih & 0x1F) | ((_ir & 0x1F) << 8) | ((_dl & 0x0F) << 16)))
        tp = cfg.get("tpowerdown")
        if tp is not None:
            tpv = max(0, min(255, int(tp)))
            writes.append((0x11, tpv))
        pwm = cfg.get("pwm") or {}
        if isinstance(pwm, dict) and pwm:
            val = 0
            if pwm.get("autoscale"):
                val |= (1 << 18)
            if pwm.get("autograd"):
                val |= (1 << 19)
            if pwm.get("freq") is not None:
                val |= ((int(pwm["freq"]) & 0x3) << 16)
            if pwm.get("freewheel") is not None:
                val |= ((int(pwm["freewheel"]) & 0x3) << 20)
            if pwm.get("reg") is not None:
                val |= ((max(1, min(15, int(pwm["reg"]))) & 0x0F) << 24)
            if pwm.get("lim") is not None:
                val |= ((max(0, min(15, int(pwm["lim"]))) & 0x0F) << 28)
            if pwm.get("grad") is not None:
                val |= ((max(0, min(255, int(pwm["grad"]))) & 0xFF) << 8)
            if pwm.get("ofs") is not None:
                val |= (max(0, min(255, int(pwm["ofs"]))) & 0xFF)
            writes.append((0x70, val))
        # CHOPCONF (toff, mres, interpolate)
        old = None
        chop = cfg.get("chop") or {}
        need_chop = False
        nv = 0
        if isinstance(chop, dict) and chop.get("toff") is not None:
            if old is None:
                old = d.read_register(0x6C).value
            nv = (old & ~0x0F) | (max(0, min(15, int(chop["toff"]))) & 0x0F)
            need_chop = True
        micro = cfg.get("microsteps")
        if micro is not None:
            # microsteps -> MRES mapping
            ms = int(micro)
            table = {256:0, 128:1, 64:2, 32:3, 16:4, 8:5, 4:6, 2:7, 1:8}
            if ms in table:
                if old is None:
                    old = d.read_register(0x6C).value
                    nv = old
                mres = table[ms] & 0x0F
                nv = (nv & ~(0x0F << 24)) | (mres << 24)
                need_chop = True
        interpolate = cfg.get("interpolate")
        if interpolate is not None:
            if old is None:
                old = d.read_register(0x6C).value
                nv = old
            if bool(interpolate):
                nv |= (1 << 28)
            else:
                nv &= ~(1 << 28)
            need_chop = True
        if need_chop:
            writes.append((0x6C, nv))
        if writes:
            if logger:
                try:
                    logger.info(f"Aplicando TMC5160 cfg em spidev{bus}.{dev}: {len(writes)} writes")
                except Exception:
                    pass
            d.apply_registers(writes)


class _StopFlag:
    def __init__(self) -> None:
        self.done = False


# ---------------- LED helpers ----------------
def set_led(client: STM32Client, *, on: Optional[bool] = None, blink_hz: Optional[float] = None) -> None:
    """Controla LED1 via STM32: on=True/False ou blink a blink_hz.
    blink tem precedência sobre on.
    """
    led_mask = 0x01
    if blink_hz is not None:
        mode = 2
        freq_centi = int(round(float(blink_hz) * 100))
    else:
        mode = 1 if on else 0
        freq_centi = 0
    req = STM32RequestBuilder.led_control(0, led_mask, mode, freq_centi)
    try:
        client.exchange(0x07, req, tries=1, settle_delay_s=0.002)  # 0x07 = REQ_LED_CTRL
    except Exception:
        pass


# ---------------- Admin helpers (config e ponte STM32) ----------------
def _load_cfg_with_fallback(path: str) -> tuple[dict, str]:
    cfg_path = path or "application.cfg"
    candidates = [cfg_path]
    if cfg_path == "application.cfg":
        try:
            here = Path(__file__).resolve().parent
            pkg = here / "application.cfg"
            candidates.append(str(pkg))
        except Exception:
            pass
    for p in candidates:
        try:
            if os.path.exists(p):
                with open(p, "r", encoding="utf-8") as f:
                    return json.load(f), p
        except Exception:
            continue
    return {}, cfg_path


def _admin_build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="cnc-cli", description="Administração: cfg/test e ponte STM32")
    sub = p.add_subparsers(dest="command", required=True)

    pcfg = sub.add_parser("config", help="Atualiza application.cfg (test.start/initial)")
    pcfg.add_argument("--file", type=str, default="application.cfg", help="Caminho do application.cfg")
    pcfg.add_argument("--set-start", nargs=3, metavar=("X","Y","Z"), type=float, default=None,
                      help="Atualiza test.start = (X,Y,Z)")
    pcfg.add_argument("--set-initial", nargs=3, metavar=("X","Y","Z"), type=float, default=None,
                      help="Atualiza test.initial = (X,Y,Z)")

    porg = sub.add_parser("origin-set", help="Fixar origem dos encoders (g_encoder_origin=g_encoder_position)")
    porg.add_argument("--bus", type=int, default=0)
    porg.add_argument("--dev", type=int, default=0)
    porg.add_argument("--speed", type=int, default=1_000_000)
    porg.add_argument("--frame-id", type=int, default=0x42)
    porg.add_argument("--mask", type=lambda x: int(x,0), default=0x07, help="Axes mask (bit0 X, bit1 Y, bit2 Z)")
    porg.add_argument("--mode", type=str, choices=["start","initial"], default="start")

    penc = sub.add_parser("enc-status", help="Consultar encoders absolutos/relativos, PID error e delta")
    penc.add_argument("--bus", type=int, default=0)
    penc.add_argument("--dev", type=int, default=0)
    penc.add_argument("--speed", type=int, default=1_000_000)
    penc.add_argument("--frame-id", type=int, default=0x43)

    return p


def _admin_dispatch(argv: Sequence[str]) -> Optional[int]:
    if not argv:
        return None
    if str(argv[0]) not in ("config", "origin-set", "enc-status"):
        return None
    ap = _admin_build_parser()
    args = ap.parse_args(list(argv))
    if args.command == "config":
        cfg, used = _load_cfg_with_fallback(str(getattr(args, "file", "application.cfg") or "application.cfg"))
        if not isinstance(cfg.get("test"), dict):
            cfg["test"] = {}
        changed = False
        if getattr(args, "set_start", None) is not None:
            sx, sy, sz = [float(v) for v in args.set_start]
            cfg["test"]["start"] = {"x": sx, "y": sy, "z": sz}
            print("Atualizado test.start:", cfg["test"]["start"])
            changed = True
        if getattr(args, "set_initial", None) is not None:
            ix, iy, iz = [float(v) for v in args.set_initial]
            cfg["test"]["initial"] = {"x": ix, "y": iy, "z": iz}
            print("Atualizado test.initial:", cfg["test"]["initial"])
            changed = True
        out = str(getattr(args, "file", used) or used)
        if not changed:
            print("Nada para atualizar em application.cfg")
            return 0
        try:
            with open(out, "w", encoding="utf-8") as f:
                json.dump(cfg, f, indent=2)
            print("Salvo:", out)
            return 0
        except Exception as exc:
            print("Erro ao salvar cfg:", exc)
            return 2
    if args.command == "origin-set":
        try:
            client = STM32Client(bus=int(args.bus), dev=int(args.dev), speed_hz=int(args.speed))
        except Exception as exc:
            print("Falha ao abrir SPI STM32:", exc)
            return 3
        try:
            mode = 0 if str(getattr(args, "mode", "start")).lower() == "start" else 1
            req = STM32RequestBuilder.set_origin(int(args.frame_id), int(args.mask), mode)
            resp = client.exchange(REQ_SET_ORIGIN, req, tries=3, settle_delay_s=0.002)
            try:
                data = STM32ResponseDecoder.set_origin(resp)
                print(data)
            except Exception:
                print("Resposta set_origin bruta:", resp)
            return 0
        finally:
            try:
                client.close()
            except Exception:
                pass
    if args.command == "enc-status":
        try:
            client = STM32Client(bus=int(args.bus), dev=int(args.dev), speed_hz=int(args.speed))
        except Exception as exc:
            print("Falha ao abrir SPI STM32:", exc)
            return 3
        try:
            req = STM32RequestBuilder.encoder_status(int(args.frame_id))
            resp = client.exchange(REQ_ENCODER_STATUS, req, tries=3, settle_delay_s=0.002)
            try:
                data = STM32ResponseDecoder.encoder_status(resp)
                print(data)
            except Exception:
                print("EncoderStatus não suportado; firmware precisa implementar o serviço.")
            return 0
        finally:
            try:
                client.close()
            except Exception:
                pass
    return 1


# ---------------- CLI (execução G-code) ----------------
def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Executa G-code (G0/G1) em STM32 + TMC5160 com LED de status")
    p.add_argument("gcode", help="Caminho do arquivo G-code (mínimo: G0/G1 X/Y/Z F; G90/G91)")
    p.add_argument("--config", type=str, default="application.cfg", help="Arquivo JSON com configurações (default: application.cfg)")
    # Mapeamento
    p.add_argument("--steps-x", type=float, default=None, help="steps/mm em X (cfg: steps-per-mm.x; default: 80)")
    p.add_argument("--steps-y", type=float, default=None, help="steps/mm em Y (cfg: steps-per-mm.y; default: 80)")
    p.add_argument("--steps-z", type=float, default=None, help="steps/mm em Z (cfg: steps-per-mm.z; default: 400)")
    p.add_argument("--feed", type=float, default=None, help="Velocidade padrão (mm/s) se linha não tiver F (cfg: motion.feed; default: 5.0)")
    # Rampas (perfil trapezoidal simplificado no cliente)
    p.add_argument("--accel", type=float, default=None, help="Aceleração (mm/s^2) (cfg: ramp.accel; default: 0=sem rampa)")
    p.add_argument("--decel", type=float, default=None, help="Desaceleração (mm/s^2) (cfg: ramp.decel; default: accel)")
    p.add_argument("--ramp-segments", type=int, default=None, help="Partições da rampa (cfg: ramp.segments; default: 8)")
    # Limites de máquina (soft-limits)
    p.add_argument("--max-x", type=float, default=None, help="Comprimento útil X em mm (cfg: machine.max.x)")
    p.add_argument("--max-y", type=float, default=None, help="Comprimento útil Y em mm (cfg: machine.max.y)")
    p.add_argument("--max-z", type=float, default=None, help="Curso útil Z em mm (cfg: machine.max.z)")
    # STM32 SPI
    p.add_argument("--bus", type=int, default=None, help="STM32 SPI bus (cfg: stm32.bus; default: 0)")
    p.add_argument("--dev", type=int, default=None, help="STM32 SPI dev (cfg: stm32.dev; default: 0)")
    p.add_argument("--speed", type=int, default=None, help="STM32 SPI speed Hz (cfg: stm32.speed; default: 1_000_000)")
    # TMC5160 SPI
    p.add_argument("--tmc-bus", type=int, default=None, help="TMC5160 SPI bus (cfg: tmc5160.bus; default: 0)")
    p.add_argument("--tmc-devs", type=str, default=None, help="Lista de devices TMC (cfg: tmc5160.devs) ex.: 1,2,3")
    p.add_argument("--tmc-speed", type=int, default=None, help="TMC5160 SPI speed Hz (cfg: tmc5160.speed; default: 4_000_000)")
    p.add_argument(
        "--microsteps",
        type=int,
        choices=[1,2,4,8,16,32,64,128,256],
        default=None,
        help="Microsteps do TMC (cfg: tmc5160.microsteps; não ajusta steps/mm automaticamente)",
    )
    p.add_argument("--check-interval", type=float, default=None, help="Intervalo de checagem TMC em s (cfg: monitor.check_interval; default: 0.5)")
    p.add_argument("--dry-run", action="store_true", help="Somente planeja; não envia ao STM32/TMC")
    p.add_argument("--log-file", type=str, default=None, help="Caminho do arquivo de log (default: ./cnc_run-YYYYmmdd_HHMMSS.log)")
    # Opcional: envia origin-set antes de enfileirar movimentos
    p.add_argument(
        "--origin-set",
        type=str,
        default=None,
        help=(
            "Envia origin-set antes de executar. Aceita 'start', 'initial' ou 'test.initial' (alias de initial)."
        ),
    )
    return p


def _parse_tmc_devs(raw) -> List[int]:
    if isinstance(raw, (list, tuple)):
        out: List[int] = []
        for v in raw:
            try:
                out.append(int(v))
            except Exception:
                pass
        return out or [1]
    out: List[int] = []
    for tok in str(raw).split(','):
        tok = tok.strip()
        if not tok:
            continue
        try:
            out.append(int(tok))
        except Exception:
            pass
    return out or [1]


def _setup_logger(path: Optional[str]) -> logging.Logger:
    logger = logging.getLogger("cnc_cli")
    logger.setLevel(logging.INFO)
    if not logger.handlers:
        fmt = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
        if path is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = f"cnc_run-{ts}.log"
        fh = logging.FileHandler(path, encoding="utf-8")
        fh.setFormatter(fmt)
        logger.addHandler(fh)
    return logger


def _log_print(logger: logging.Logger, msg: str) -> None:
    print(msg)
    try:
        logger.info(msg)
    except Exception:
        pass


def main(argv: Optional[Sequence[str]] = None) -> int:
    import sys
    argv_list = list(argv) if argv is not None else sys.argv[1:]
    # Subcomandos administrativos (config/origin-set/enc-status)
    dispatched = _admin_dispatch(argv_list)
    if dispatched is not None:
        return dispatched
    args = build_parser().parse_args(argv_list)
    # Carregar configuração JSON (application.cfg por padrão)
    cfg_path = args.config or "application.cfg"
    cfg: dict = {}
    # Resolve caminho do cfg: primeiro o informado/atual; se não existir e for o default,
    # tenta o arquivo application.cfg ao lado deste módulo (raspberry_spi/application.cfg)
    cfg_paths_to_try: List[str] = []
    if cfg_path:
        cfg_paths_to_try.append(cfg_path)
    # fallback quando nome padrão
    if (cfg_path == "application.cfg"):
        try:
            here = Path(__file__).resolve().parent
            pkg_cfg = str(here / "application.cfg")
            if pkg_cfg not in cfg_paths_to_try:
                cfg_paths_to_try.append(pkg_cfg)
        except Exception:
            pass
    for cpath in cfg_paths_to_try:
        if cpath and os.path.exists(cpath):
            try:
                with open(cpath, "r", encoding="utf-8") as f:
                    cfg = json.load(f)
                    cfg_path = cpath
                    break
            except Exception:
                continue
    def _cfg(path: List[str], default=None):
        cur = cfg
        try:
            for k in path:
                if not isinstance(cur, dict):
                    return default
                cur = cur.get(k)
            return default if cur is None else cur
        except Exception:
            return default
    # Resolve parâmetros: CLI > CFG > default
    # Prefer steps-per-mm.*, mas aceita legacy steps.* para compatibilidade
    steps_x = float(args.steps_x) if args.steps_x is not None else float(_cfg(["steps-per-mm","x"], _cfg(["steps","x"], 80.0)))
    steps_y = float(args.steps_y) if args.steps_y is not None else float(_cfg(["steps-per-mm","y"], _cfg(["steps","y"], 80.0)))
    steps_z = float(args.steps_z) if args.steps_z is not None else float(_cfg(["steps-per-mm","z"], _cfg(["steps","z"], 400.0)))
    feed = float(args.feed) if args.feed is not None else float(_cfg(["motion","feed"], 5.0))
    accel = float(args.accel) if args.accel is not None else float(_cfg(["ramp","accel"], 0.0))
    decel = float(args.decel) if args.decel is not None else float(_cfg(["ramp","decel"], accel if accel > 0 else 0.0))
    ramp_segments = int(args.ramp_segments) if args.ramp_segments is not None else int(_cfg(["ramp","segments"], 8))
    max_x = float(args.max_x) if args.max_x is not None else _cfg(["machine","max","x"], None)
    max_y = float(args.max_y) if args.max_y is not None else _cfg(["machine","max","y"], None)
    max_z = float(args.max_z) if args.max_z is not None else _cfg(["machine","max","z"], None)
    bus = int(args.bus) if args.bus is not None else int(_cfg(["stm32","bus"], 0))
    dev = int(args.dev) if args.dev is not None else int(_cfg(["stm32","dev"], 0))
    speed = int(args.speed) if args.speed is not None else int(_cfg(["stm32","speed"], 1_000_000))
    tmc_bus = int(args.tmc_bus) if args.tmc_bus is not None else int(_cfg(["tmc5160","bus"], 0))
    tmc_devs_val = args.tmc_devs if args.tmc_devs is not None else _cfg(["tmc5160","devs"], [1])
    tmc_speed = int(args.tmc_speed) if args.tmc_speed is not None else int(_cfg(["tmc5160","speed"], 4_000_000))
    microsteps = int(args.microsteps) if args.microsteps is not None else _cfg(["tmc5160","microsteps"], None)
    check_interval = float(args.check_interval) if args.check_interval is not None else float(_cfg(["monitor","check_interval"], 0.5))
    log_file = args.log_file if args.log_file is not None else _cfg(["logging","file"], None)
    # Logger
    logger = _setup_logger(log_file)
    present = cfg_path if cfg else 'none'
    _log_print(logger, f"Iniciando cnc_cli (cfg={present})")
    gcode_path = Path(args.gcode)
    if not gcode_path.exists():
        _log_print(logger, f"G-code inexistente: {gcode_path}")
        return 2
    try:
        segments, bb = parse_gcode_linear(gcode_path, default_feed=feed)
    except Exception as exc:
        _log_print(logger, f"Falha ao ler G-code: {exc}")
        return 2
    x0, x1, y0, y1, z0, z1 = bb
    _log_print(logger, f"G-code bbox (x0,x1,y0,y1,z0,z1): {(x0,x1,y0,y1,z0,z1)}")
    _log_print(logger, f"Segmentos (G0/G1) extraídos: {len(segments)}")
    if accel > 0 or decel > 0:
        _log_print(logger, f"Rampa habilitada: accel={accel} mm/s^2, decel={decel} mm/s^2, parts={ramp_segments}")
    # Validação de soft-limits (tamanho do job)
    size_x = abs(x1 - x0)
    size_y = abs(y1 - y0)
    size_z = abs(z1 - z0)
    _log_print(logger, f"Job size (mm): X={size_x:.3f}, Y={size_y:.3f}, Z={size_z:.3f}")
    lim_errors = []
    if max_x is not None and size_x > float(max_x):
        lim_errors.append(f"X job={size_x:.3f} > max_x={float(max_x):.3f}")
    if max_y is not None and size_y > float(max_y):
        lim_errors.append(f"Y job={size_y:.3f} > max_y={float(max_y):.3f}")
    if max_z is not None and size_z > float(max_z):
        lim_errors.append(f"Z job={size_z:.3f} > max_z={float(max_z):.3f}")
    if lim_errors:
        _log_print(logger, "Soft-limit violado: " + "; ".join(lim_errors))
        _log_print(logger, "Ajuste o G-code, origem ou machine.max.* no application.cfg")
        return 4
    if args.dry_run:
        return 0

    # Preparar monitoramento de TMC
    tmc_devs = _parse_tmc_devs(tmc_devs_val)
    tmc_targets = [(tmc_bus, d) for d in tmc_devs]

    stop = _StopFlag()
    error_detected = False

    # Conectar STM32
    try:
        client = STM32Client(bus=bus, dev=dev, speed_hz=speed)
    except Exception as exc:
        _log_print(logger, f"Falha ao abrir SPI STM32: {exc}")
        return 3

    try:
        # Envia microsteps (se houver em cfg)
        try:
            ms_cfg = _cfg(["tmc5160","microsteps"], None)
            if ms_cfg is not None:
                ms = int(ms_cfg)
                if ms < 1:
                    ms = 1
                if ms > 256:
                    ms = 256
                req_ms = STM32RequestBuilder.set_microsteps(0x40, ms)
                _ = client.exchange(REQ_SET_MICROSTEPS, req_ms, tries=2, settle_delay_s=0.002)
                _log_print(logger, f"STM32 microsteps setado para {ms} (conversão PI/telemetria)")
        except Exception as exc:
            _log_print(logger, f"Aviso: não foi possível enviar microsteps ao STM32: {exc}")
        # Opcional: origin-set a partir do modo solicitado
        origin_mode_raw = (args.origin_set or '').strip().lower() if getattr(args, 'origin_set', None) else ''
        if origin_mode_raw:
            if origin_mode_raw in ("test.initial", "initial"):
                origin_mode = 1
                mode_label = "initial"
                # Loga se há valores em test.initial na configuração (não são enviados, apenas referência)
                try:
                    ti = cfg.get("test", {}).get("initial", {}) if isinstance(cfg, dict) else {}
                    if isinstance(ti, dict):
                        _log_print(logger, f"Origin-set (initial) solicitado; test.initial={ti}")
                except Exception:
                    pass
            elif origin_mode_raw in ("start",):
                origin_mode = 0
                mode_label = "start"
            else:
                _log_print(logger, f"Modo inválido em --origin-set: {origin_mode_raw}. Use 'start' ou 'initial'.")
                return 2
            try:
                req = STM32RequestBuilder.set_origin(0x42, 0x07, origin_mode)
                resp = client.exchange(REQ_SET_ORIGIN, req, tries=3, settle_delay_s=0.002)
                try:
                    data = STM32ResponseDecoder.set_origin(resp)
                    _log_print(logger, f"Origin-set enviado (mode={mode_label}): {data}")
                except Exception:
                    _log_print(logger, f"Origin-set (mode={mode_label}) enviado; resposta bruta: {resp}")
            except Exception as exc:
                _log_print(logger, f"Aviso: firmware pode não suportar set_origin: {exc}")

        # Aplicar configuração TMC (se existir em cfg)
        tmc_cfg = _cfg(["tmc5160"], {})
        # CLI override de microsteps
        if isinstance(tmc_cfg, dict) and microsteps is not None:
            tmc_cfg = dict(tmc_cfg)
            tmc_cfg["microsteps"] = int(microsteps)
        if isinstance(tmc_cfg, dict) and any(k in tmc_cfg for k in ("ihold","irun","iholddelay","globalscaler","tpowerdown","pwm","chop")):
            for b, dvid in tmc_targets:
                try:
                    tmc_apply_cfg(b, dvid, tmc_speed, tmc_cfg, logger)
                except Exception as exc:
                    _log_print(logger, f"Aviso: falha ao aplicar cfg TMC em spidev{b}.{dvid}: {exc}")

        # LED: piscar 1 Hz enquanto executa
        set_led(client, blink_hz=1.0)
        _log_print(logger, "LED -> blink 1.0 Hz (executando)")

        # Enfileirar movimentos
        enqueue_segments(
            client,
            segments,
            steps_x=steps_x,
            steps_y=steps_y,
            steps_z=steps_z,
            frame_id=1,
            accel=accel,
            decel=decel,
            ramp_segments=ramp_segments,
        )
        # Disparar execução
        start_req = STM32RequestBuilder.start_move(1)
        _ = client.exchange(REQ_START_MOVE, start_req, tries=3, settle_delay_s=0.002)
        _log_print(logger, "START_MOVE enviado")

        # Loop de monitoramento enquanto roda
        _log_print(logger, f"Executando movimentos; check TMC a cada {check_interval}s")
        last_check = 0.0
        start_time = time.time()
        while True:
            now = time.time()
            if now - last_check >= float(check_interval):
                last_check = now
                # Checa todos os TMC
                errs_all: List[str] = []
                for bus, dev in tmc_targets:
                    errs = read_tmc_errors(bus, dev, int(tmc_speed))
                    if errs:
                        errs_all.extend([f"spidev{bus}.{dev}:{e}" for e in errs])
                    else:
                        _log_print(logger, f"TMC spidev{bus}.{dev} OK")
                if errs_all:
                    error_detected = True
                    _log_print(logger, f"Erros TMC detectados: {errs_all}")
                    # LED: aceso
                    set_led(client, on=True)
                    _log_print(logger, "LED -> ON (erro)")
                    # Security off em todos os TMC
                    for bus, dev in tmc_targets:
                        try:
                            tmc_security_off(bus, dev, int(tmc_speed), freewheel=1)
                        except Exception as exc:
                            _log_print(logger, f"Aviso: falha ao aplicar security off em TMC spidev{bus}.{dev}: {exc}")
                    # MOVE_END
                    try:
                        end_req = STM32RequestBuilder.move_end(0)
                        _ = client.exchange(REQ_MOVE_END, end_req, tries=3, settle_delay_s=0.002)
                    except Exception:
                        pass
                    _log_print(logger, "Execução abortada por falhas nos drivers.")
                    return 1
            time.sleep(0.1)
            # Critério simples de término (demo): tempo máximo proporcional ao número de segmentos
            if now - start_time > max(2.0, 0.02 * len(segments)):
                break

        if not error_detected:
            # LED: apagado (sucesso)
            set_led(client, on=False)
            _log_print(logger, "LED -> OFF (sucesso)")
            _log_print(logger, "Fluxo concluído com sucesso.")
            return 0
        return 1
    finally:
        stop.done = True
        try:
            client.close()
        except Exception:
            pass


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
