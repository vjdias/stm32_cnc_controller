#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""CLI para executar uma sequência de movimentos/ajustes via Raspberry Pi.

Características
- Lê um arquivo JSON com a lista de movimentos (passos, velocidades, direções)
  e, opcionalmente, ajustes de PID e parâmetros TMC5160 (toff, ihold, irun,
  ihold_delay, tpowerdown, globalscaler, microsteps, interpolate).
- Valida todos os valores contra limites definidos em `application.cfg`.
- Em caso de violação, aborta a execução, registra o motivo em log e exibe a
  mensagem para o usuário.
- Se válido, aplica os ajustes TMC5160 (devs 1..3), envia os movimentos ao
  STM32 (dev 0), liga o LED em modo pisca durante a execução e monitora erros
  graves do TMC (temperatura/falhas). Em falha grave, interrompe e força TOFF=0.
- Oferece subcomando de status (consulta do `MOVE_QUEUE_STATUS`).

Formato do arquivo de sequência (JSON)
{
  // Passos executados em ordem. Não inclua frameId: ele é gerado internamente.
  "steps": [
    { "tmc": { "toff": 3, "ihold": 2, "irun": 8, "ihold_delay": 6, "globalscaler": 32, "microsteps": 16, "interpolate": true } },
    { "pid": { "x": {"kp": 800, "ki": 40, "kd": 120}, "y": {"kp": 800, "ki": 40, "kd": 120}, "z": {"kp": 800, "ki": 40, "kd": 120} } },
    { "move": { "pos": {"x": 10000, "y": 0, "z": 0}, "vel": {"x": 12000, "y": 0, "z": 0}, "dir": {"x": 1, "y": 1, "z": 1} } },
    { "move": { "pos": {"x": 0, "y": 5000, "z": 0}, "vel": {"x": 8000, "y": 8000, "z": 0}, "dir": {"x": 1, "y": -1, "z": 1},
                 "pid": {"y": {"kp": 900}} } }
  ]
}
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple
import math

from .stm32.stm32_cli import STM32Client  # type: ignore
from .stm32.stm32_requests import STM32RequestBuilder  # type: ignore
from .stm32.stm32_responses import STM32ResponseDecoder  # type: ignore
from .tmc5160.tmc5160 import (
    TMC5160Configurator,
    TMC5160RegisterPreset,
    REG_GSTAT,
    REG_DRV_STATUS,
    REG_IHOLD_IRUN,
    REG_CHOPCONF,
    REG_PWMCONF,
    REG_GLOBAL_SCALER,
    REG_TPOWERDOWN,
)


def _load_cfg(path: str | None) -> Dict[str, Any]:
    candidates = [path] if path else ["application.cfg"]
    if (not path) or path == "application.cfg":
        here = Path(__file__).resolve().parent
        candidates.append(str(here / "application.cfg"))
    for p in candidates:
        if not p:
            continue
        try:
            if os.path.exists(p):
                with open(p, "r", encoding="utf-8") as f:
                    return json.load(f)
        except Exception:
            continue
    return {}


def _mk_logger() -> logging.Logger:
    logs_dir = Path(__file__).resolve().parent / "run_logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    lf = logs_dir / f"sequence_run_{ts}.log"
    logger = logging.getLogger(f"sequence_cli.{ts}")
    logger.setLevel(logging.INFO)
    fh = logging.FileHandler(lf, encoding="utf-8")
    fh.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
    logger.addHandler(fh)
    sh = logging.StreamHandler(sys.stdout)
    sh.setFormatter(logging.Formatter("%(message)s"))
    logger.addHandler(sh)
    logger.info("Iniciando execução (log=%s)", str(lf))
    return logger


def _within(v: int, rng: Tuple[int, int]) -> bool:
    lo, hi = int(rng[0]), int(rng[1])
    return lo <= int(v) <= hi


def _validate_sequence(cfg: Dict[str, Any], seq: Dict[str, Any]) -> Tuple[bool, List[str]]:
    errs: List[str] = []
    limits = cfg.get("limits", {}) if isinstance(cfg.get("limits"), dict) else {}
    lim_pid = limits.get("pid", {}) if isinstance(limits.get("pid"), dict) else {}
    lim_tmc = limits.get("tmc5160", {}) if isinstance(limits.get("tmc5160"), dict) else {}
    lim_vel = limits.get("velocity", {}) if isinstance(limits.get("velocity"), dict) else {}

    # Velocidade máxima steps/s (por eixo)
    vmax = int(lim_vel.get("max_sps", 65535))

    def _chk_pid(axis: str, pid: Dict[str, Any], path: str) -> None:
        for k in ("kp", "ki", "kd"):
            v = int(pid.get(k, 0))
            rng = tuple(lim_pid.get(k, (0, 65535)))  # type: ignore
            if not _within(v, rng):
                errs.append(f"{path}.{k}={v} fora de limites {rng}")

    def _validate_tmc_patch(tmc: Dict[str, Any], *, path: str) -> None:
        # Campos e limites básicos
        ranges = {
            "toff": tuple(lim_tmc.get("toff", (0, 15))),
            "ihold": tuple(lim_tmc.get("ihold", (0, 31))),
            "irun": tuple(lim_tmc.get("irun", (0, 31))),
            "ihold_delay": tuple(lim_tmc.get("ihold_delay", lim_tmc.get("iholddelay", (0, 15)))),
            "tpowerdown": tuple(lim_tmc.get("tpowerdown", (0, 255))),
            "globalscaler": tuple(lim_tmc.get("globalscaler", (0, 255))),
            "hstrt": tuple(lim_tmc.get("hstrt", (0, 7))),
            "hend": tuple(lim_tmc.get("hend", (0, 15))),
            "tbl": tuple(lim_tmc.get("tbl", (0, 3))),
            "dedge": tuple(lim_tmc.get("dedge", (0, 1))),
            "diss2g": tuple(lim_tmc.get("diss2g", (0, 1))),
            "diss2vs": tuple(lim_tmc.get("diss2vs", (0, 1))),
        }
        for key, rng in ranges.items():
            if key in tmc:
                try:
                    val = int(tmc.get(key))
                except Exception:
                    errs.append(f"{path}.{key} inválido (inteiro esperado)")
                    continue
                if not _within(val, rng):
                    errs.append(f"{path}.{key}={val} fora de limites {rng}")
        if "microsteps" in tmc:
            allowed = lim_tmc.get("microsteps", [1, 2, 4, 8, 16, 32, 64, 128, 256])
            try:
                ms = int(tmc["microsteps"]) 
            except Exception:
                errs.append(f"{path}.microsteps inválido (inteiro esperado)")
                ms = None
            if ms is not None and ms not in allowed:
                errs.append(f"{path}.microsteps={tmc['microsteps']} não permitido (opções: {allowed})")

    # TMC (global da sequência)
    if isinstance(seq.get("tmc"), dict):
        _validate_tmc_patch(seq["tmc"], path="tmc")

    # PID default
    if isinstance(seq.get("pid"), dict):
        for ax in ("x", "y", "z"):
            if isinstance(seq["pid"].get(ax), dict):
                _chk_pid(ax, seq["pid"][ax], f"pid.{ax}")

    def _validate_move(mv: Dict[str, Any], *, path: str) -> None:
        # Novo formato: pos{ x,y,z } (passos), vel{ x,y,z } (steps/s)
        if isinstance(mv.get("pos"), dict) and isinstance(mv.get("vel"), dict):
            for k in ("x", "y", "z"):
                try:
                    sval = int(mv["pos"].get(k, 0))
                except Exception:
                    errs.append(f"{path}.pos.{k} inválido (inteiro esperado)")
                    continue
                if sval < 0:
                    errs.append(f"{path}.pos.{k} negativo")
                try:
                    vval = int(mv["vel"].get(k, 0))
                except Exception:
                    errs.append(f"{path}.vel.{k} inválido (inteiro esperado)")
                    continue
                if not (0 <= vval <= vmax):
                    errs.append(f"{path}.vel.{k}={vval} fora de 0..{vmax}")
        else:
            # Compat: campos planos sx/sy/sz e vx/vy/vz
            for field in ("sx", "sy", "sz", "vx", "vy", "vz"):
                if field not in mv:
                    errs.append(f"{path}.{field} ausente")
                    continue
                try:
                    val = int(mv[field])
                except Exception:
                    errs.append(f"{path}.{field} inválido (inteiro esperado)")
                    continue
                if field.startswith("s") and val < 0:
                    errs.append(f"{path}.{field} negativo")
                if field.startswith("v") and not (0 <= val <= vmax):
                    errs.append(f"{path}.{field}={val} fora de 0..{vmax}")
        if ("dir_mask" not in mv) and ("dir" not in mv):
            errs.append(f"{path}.dir ou {path}.dir_mask ausente")
        if isinstance(mv.get("pid"), dict):
            for ax in ("x", "y", "z"):
                if isinstance(mv["pid"].get(ax), dict):
                    _chk_pid(ax, mv["pid"][ax], f"{path}.pid.{ax}")

    # Suporta formato antigo (tmc/pid/moves) e novo (steps)
    steps = seq.get("steps")
    if isinstance(steps, list) and steps:
        for i, st in enumerate(steps):
            if not isinstance(st, dict):
                errs.append(f"steps[{i}] inválido (objeto esperado)")
                continue
            if "tmc" in st:
                if not isinstance(st["tmc"], dict):
                    errs.append(f"steps[{i}].tmc inválido (objeto)")
                else:
                    _validate_tmc_patch(st["tmc"], path=f"steps[{i}].tmc")
            if "pid" in st:
                pid_obj = st["pid"]
                if not isinstance(pid_obj, dict):
                    errs.append(f"steps[{i}].pid inválido (objeto)")
                else:
                    for ax in ("x", "y", "z"):
                        if isinstance(pid_obj.get(ax), dict):
                            _chk_pid(ax, pid_obj[ax], f"steps[{i}].pid.{ax}")
            if "move" in st:
                if not isinstance(st["move"], dict):
                    errs.append(f"steps[{i}].move inválido (objeto)")
                else:
                    _validate_move(st["move"], path=f"steps[{i}].move")
        return (len(errs) == 0), errs
    else:
        moves = seq.get("moves")
        if not isinstance(moves, list) or not moves:
            errs.append("lista 'moves' ausente ou vazia")
            return False, errs
        for i, mv in enumerate(moves):
            _validate_move(mv, path=f"moves[{i}]")
        return (len(errs) == 0), errs


def _dir_mask_from_obj(obj: Dict[str, Any]) -> int:
    dx = int(obj.get("x", 0))
    dy = int(obj.get("y", 0))
    dz = int(obj.get("z", 0))
    m = 0
    if dx > 0:
        m |= 0x01
    if dy > 0:
        m |= 0x02
    if dz > 0:
        m |= 0x04
    return m


def _set_led(client: STM32Client, *, on: Optional[bool] = None, blink_hz: Optional[float] = None) -> None:
    mask = 0x01
    if blink_hz is not None:
        mode = 2
        freq_centi = int(round(float(blink_hz) * 100))
    else:
        mode = 1 if on else 0
        freq_centi = 0
    req = STM32RequestBuilder.led_control(0, mask, mode, freq_centi)
    try:
        print(f"STM32 ← LED_CONTROL on={bool(on)} blink_hz={blink_hz if blink_hz is not None else 0}")
        raw = client.exchange(0x07, req, tries=1, settle_delay_s=5)
        try:
            data = STM32ResponseDecoder.led(raw)
            out = {
                "cmd": "led_control_ack",
                "frameId": int(data.get("frameId", 0)),
                "ledMask": int(data.get("ledMask", 0)),
                "status": int(data.get("status", 0)),
            }
            print(json.dumps(out, ensure_ascii=False))
        except Exception:
            pass
    except Exception:
        pass


def _tmc_security_off_all(cfg: Dict[str, Any], logger: logging.Logger) -> None:
    tmc = cfg.get("tmc5160", {})
    bus = int(tmc.get("bus", 0))
    devs = list(tmc.get("devs", [1, 2, 3]))
    speed = int(tmc.get("speed", 4_000_000))
    for dev in devs:
        try:
            with TMC5160Configurator(bus=bus, device=int(dev), speed_hz=speed, register_preset=TMC5160RegisterPreset(writes=tuple())) as d:  # type: ignore
                # IHOLD/IRUN=0; CHOPCONF.TOFF=0; PWMCONF.FREEWHEEL=1
                writes = [
                    (REG_IHOLD_IRUN, 0x00000000),
                    (REG_CHOPCONF, (d.read_register(REG_CHOPCONF).value & ~0x0F)),
                    (REG_PWMCONF, ((d.read_register(REG_PWMCONF).value & ~(0x3 << 20)) | (1 << 20))),
                ]
                d.apply_registers(writes)
                logger.info("TMC dev %s: segurança aplicada (TOFF=0, IHOLD/IRUN=0)", dev)
        except Exception as exc:
            logger.error("Falha ao aplicar segurança no TMC dev %s: %s", dev, exc)


def _tmc_apply(cfg: Dict[str, Any], patch: Dict[str, Any], logger: logging.Logger) -> None:
    tcfg = cfg.get("tmc5160", {})
    bus = int(tcfg.get("bus", 0))
    devs = list(tcfg.get("devs", [1, 2, 3]))
    speed = int(tcfg.get("speed", 4_000_000))
    for dev in devs:
        try:
            with TMC5160Configurator(bus=bus, device=int(dev), speed_hz=speed, register_preset=TMC5160RegisterPreset(writes=tuple())) as d:  # type: ignore
                writes: List[Tuple[int, int]] = []
                # Globalscaler
                if patch.get("globalscaler") is not None:
                    gs = int(patch["globalscaler"]) & 0xFF
                    writes.append((REG_GLOBAL_SCALER, gs))
                # IHOLD/IRUN/IHOLDDELAY
                if any(k in patch for k in ("ihold", "irun", "ihold_delay", "iholddelay")):
                    ih = int(patch.get("ihold", 0)) & 0x1F
                    ir = int(patch.get("irun", 0)) & 0x1F
                    ihd = int(patch.get("ihold_delay", patch.get("iholddelay", 6))) & 0x0F
                    writes.append((REG_IHOLD_IRUN, ih | (ir << 8) | (ihd << 16)))
                # TPOWERDOWN
                if patch.get("tpowerdown") is not None:
                    tp = int(patch["tpowerdown"]) & 0xFF
                    writes.append((REG_TPOWERDOWN, tp))
                # CHOPCONF: TOFF / microsteps / interpolate
                old = None
                nv = 0
                need_chop = False
                if patch.get("toff") is not None:
                    if old is None:
                        old = d.read_register(REG_CHOPCONF).value
                        nv = old
                    nv = (nv & ~0x0F) | (int(patch["toff"]) & 0x0F)
                    need_chop = True
                if patch.get("microsteps") is not None:
                    table = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
                    ms = int(patch["microsteps"])
                    if ms in table:
                        if old is None:
                            old = d.read_register(REG_CHOPCONF).value
                            nv = old
                        mres = table[ms] & 0x0F
                        nv = (nv & ~(0x0F << 24)) | (mres << 24)
                        need_chop = True
                if patch.get("interpolate") is not None:
                    if old is None:
                        old = d.read_register(REG_CHOPCONF).value
                        nv = old
                    if bool(patch.get("interpolate")):
                        nv |= (1 << 28)
                    else:
                        nv &= ~(1 << 28)
                    need_chop = True
                # HSTRT (3 bits) e HEND (4 bits)
                if patch.get("hstrt") is not None:
                    if old is None:
                        old = d.read_register(REG_CHOPCONF).value
                        nv = old
                    hs = max(0, min(7, int(patch["hstrt"]))) & 0x07
                    nv = (nv & ~(0x07 << 4)) | (hs << 4)
                    need_chop = True
                if patch.get("hend") is not None:
                    if old is None:
                        old = d.read_register(REG_CHOPCONF).value
                        nv = old
                    he = max(0, min(15, int(patch["hend"]))) & 0x0F
                    nv = (nv & ~(0x0F << 7)) | (he << 7)
                    need_chop = True
                # TBL (2 bits)
                if patch.get("tbl") is not None:
                    if old is None:
                        old = d.read_register(REG_CHOPCONF).value
                        nv = old
                    tb = max(0, min(3, int(patch["tbl"]))) & 0x03
                    nv = (nv & ~(0x03 << 15)) | (tb << 15)
                    need_chop = True
                # VSENSE: não configurável via parâmetro neste projeto (fixo por hardware)
                # DEDGE (bit 29)
                if patch.get("dedge") is not None:
                    if old is None:
                        old = d.read_register(REG_CHOPCONF).value
                        nv = old
                    if int(patch["dedge"]) != 0:
                        nv |= (1 << 29)
                    else:
                        nv &= ~(1 << 29)
                    need_chop = True
                # DISS2G (bit 30) / DISS2VS (bit 31)
                if patch.get("diss2g") is not None:
                    if old is None:
                        old = d.read_register(REG_CHOPCONF).value
                        nv = old
                    if int(patch["diss2g"]) != 0:
                        nv |= (1 << 30)
                    else:
                        nv &= ~(1 << 30)
                    need_chop = True
                if patch.get("diss2vs") is not None:
                    if old is None:
                        old = d.read_register(REG_CHOPCONF).value
                        nv = old
                    if int(patch["diss2vs"]) != 0:
                        nv |= (1 << 31)
                    else:
                        nv &= ~(1 << 31)
                    need_chop = True
                if need_chop:
                    writes.append((REG_CHOPCONF, nv))
                if writes:
                    logger.info("TMC bus=%s dev=%s: enviando %d writes:", bus, dev, len(writes))
                    for addr, val in writes:
                        logger.info("  -> reg 0x%02X = 0x%08X", int(addr) & 0xFF, int(val) & 0xFFFFFFFF)
                    d.apply_registers(writes)
                    logger.info("TMC dev %s: %d writes aplicados", dev, len(writes))
        except Exception as exc:
            logger.error("Falha aplicando TMC em dev %s: %s", dev, exc)


def _seq_dir_mask(mv: Dict[str, Any]) -> int:
    if "dir_mask" in mv:
        return int(mv["dir_mask"]) & 0x07
    if isinstance(mv.get("dir"), dict):
        return _dir_mask_from_obj(mv["dir"]) & 0x07
    return 0


def _pid_for_move(cfg: Dict[str, Any], seq_default_pid: Dict[str, Any], mv: Dict[str, Any]) -> Tuple[int, int, int, int, int, int, int, int, int]:
    def _get(ax: str, k: str) -> int:
        # prioridade: mv.pid.ax.k > seq.pid.ax.k > cfg.pid.ax.k > 0
        if isinstance(mv.get("pid"), dict) and isinstance(mv["pid"].get(ax), dict) and (k in mv["pid"][ax]):
            return int(mv["pid"][ax][k])
        if isinstance(seq_default_pid, dict) and isinstance(seq_default_pid.get(ax), dict) and (k in seq_default_pid[ax]):
            return int(seq_default_pid[ax][k])
        if isinstance(cfg.get("pid"), dict) and isinstance(cfg["pid"].get(ax), dict) and (k in cfg["pid"][ax]):
            return int(cfg["pid"][ax][k])
        return 0
    kp_x, ki_x, kd_x = _get("x", "kp"), _get("x", "ki"), _get("x", "kd")
    kp_y, ki_y, kd_y = _get("y", "kp"), _get("y", "ki"), _get("y", "kd")
    kp_z, ki_z, kd_z = _get("z", "kp"), _get("z", "ki"), _get("z", "kd")
    return (kp_x, ki_x, kd_x, kp_y, ki_y, kd_y, kp_z, ki_z, kd_z)


class _FrameSeq:
    def __init__(self, start: int = 1) -> None:
        self._cur = (start - 1) & 0xFF
    def next(self) -> int:
        self._cur = (self._cur + 1) & 0xFF
        if self._cur == 0:
            self._cur = 1
        return self._cur


def _spi_params(cfg: dict, kind: str) -> tuple[int, float]:
    """Resolve (tries, settle) para uma operação SPI.

    Regras:
    - Se '{kind}_tries' for informado no cfg, respeita-o (>=1) e NÃO amplifica pelo min_wait_s.
    - Caso contrário, calcula tries para cobrir pelo menos min_wait_s (>=5s) com o settle informado.
    - Settle padrão = 5.0 s.
    """
    spi = cfg.get("spi", {}) if isinstance(cfg.get("spi"), dict) else {}
    tries_key = f"{kind}_tries"
    settle_key = f"{kind}_settle"
    dsettle = 5.0
    settle = float(spi.get(settle_key, dsettle))
    settle = max(0.001, settle)
    specified_tries = spi.get(tries_key, None)
    if specified_tries is not None:
        tries = max(1, int(specified_tries))
        return tries, settle
    # Deriva tries a partir de min_wait_s
    min_wait = float(spi.get("min_wait_s", 5.0))
    if not math.isfinite(min_wait) or min_wait < 5.0:
        min_wait = 5.0
    tries = int(math.ceil(min_wait / settle))
    return max(1, tries), settle


def _spi_busy_cooldown(cfg: dict) -> float:
    spi = cfg.get("spi", {}) if isinstance(cfg.get("spi"), dict) else {}
    try:
        val = float(spi.get("busy_cooldown_s", 5.0))
        return max(0.1, val)
    except Exception:
        return 5.0


def _process_steps(
    client: STM32Client,
    cfg: Dict[str, Any],
    seq: Dict[str, Any],
    steps: List[Dict[str, Any]],
    logger: logging.Logger,
) -> int:
    # PID default parte de cfg.pid e opcionalmente de seq.pid
    cur_pid = {ax: dict(cfg.get("pid", {}).get(ax, {})) for ax in ("x", "y", "z")}
    if isinstance(seq.get("pid"), dict):
        for ax in ("x", "y", "z"):
            if isinstance(seq["pid"].get(ax), dict):
                cur_pid[ax].update({k: int(v) for k, v in seq["pid"][ax].items()})

    sent = 0
    qadd_tries, qadd_settle = _spi_params(cfg, "queue_add")
    # Ajuste solicitado: settle-delay dos QUEUE_ADD = 2s (mantém semântica simples)
    qadd_settle = 2.0
    qadd_tries = 1
    start_tries, start_settle = _spi_params(cfg, "start_move")
    fid = _FrameSeq(1)
    last_frame_id: Optional[int] = None

    for i, st in enumerate(steps):
        if "tmc" in st and isinstance(st["tmc"], dict):
            _tmc_apply(cfg, st["tmc"], logger)
        if "pid" in st and isinstance(st["pid"], dict):
            for ax in ("x", "y", "z"):
                if isinstance(st["pid"].get(ax), dict):
                    cur_pid[ax].update({k: int(v) for k, v in st["pid"][ax].items()})
        if "move" in st and isinstance(st["move"], dict):
            mv = st["move"]
            dir_mask = _seq_dir_mask(mv)
            # Novo formato: vel{x,y,z} / pos{x,y,z}
            if isinstance(mv.get("vel"), dict) and isinstance(mv.get("pos"), dict):
                vx = int(mv["vel"].get("x", 0)); vy = int(mv["vel"].get("y", 0)); vz = int(mv["vel"].get("z", 0))
                sx = int(mv["pos"].get("x", 0)); sy = int(mv["pos"].get("y", 0)); sz = int(mv["pos"].get("z", 0))
            else:
                vx, vy, vz = int(mv.get("vx", 0)), int(mv.get("vy", 0)), int(mv.get("vz", 0))
                sx, sy, sz = int(mv.get("sx", 0)), int(mv.get("sy", 0)), int(mv.get("sz", 0))
            pid9 = _pid_for_move(cfg, cur_pid, mv)
            frame_id = fid.next()
            req = STM32RequestBuilder.move_queue_add(
                frame_id,
                dir_mask,
                vx, sx,
                vy, sy,
                vz, sz,
                *pid9,
            )
            try:
                logger.info(
                    "STM32 ← QUEUE_ADD [frame=%d] dir=0x%02X v=(%d,%d,%d) s=(%d,%d,%d) | aguardará %.3fs (tries=%d) pela resposta",
                    frame_id,
                    dir_mask, vx, vy, vz, sx, sy, sz, qadd_tries * qadd_settle if qadd_tries > 1 else qadd_settle, qadd_tries,
                )
                resp = client.exchange(0x01, req, tries=qadd_tries, settle_delay_s=qadd_settle)
            except Exception as exc:
                cooldown = _spi_busy_cooldown(cfg)
                msg = str(exc).lower()
                if "busy" in msg or "polling" in msg or "resposta" in msg:
                    logger.warning(
                        "QueueAdd: sem ACK/BUSY após %.3fs (tries=%d). Aguardando %.3fs e repetindo com settle=%.3fs...",
                        qadd_tries * qadd_settle if qadd_tries > 1 else qadd_settle, qadd_tries, cooldown, max(qadd_settle, cooldown),
                    )
                    time.sleep(cooldown)
                    resp = client.exchange(
                        0x01, req, tries=max(qadd_tries, int(max(80, cooldown / max(0.001, qadd_settle)))) ,
                        settle_delay_s=max(qadd_settle, cooldown)
                    )
                else:
                    raise
            try:
                data_ack = STM32ResponseDecoder.queue_add_ack(resp)
                status_code = int(data_ack.get("status", 0))
                status_label = {0: "ok", 1: "invalid", 2: "queue_full"}.get(status_code, "unknown")
                out = {
                    "cmd": "queue_add_ack",
                    "frameId": int(data_ack.get("frameId", 0)),
                    "status": status_code,
                    "status_label": status_label,
                }
                print(json.dumps(out, ensure_ascii=False))
            except Exception:
                pass
            sent += 1
            last_frame_id = frame_id
    # Inicia execução se houve ao menos um movimento
    if sent > 0:
        try:
            # Pequena folga entre o último queue_add e o start_move (configurável)
            gap = float(cfg.get("spi", {}).get("start_move_gap_s", 0.5)) if isinstance(cfg.get("spi"), dict) else 0.5
            if gap > 0:
                time.sleep(gap)
            frame_id = last_frame_id if last_frame_id is not None else fid._cur
            # Ajuste solicitado: settle do START_MOVE = 2 * número de itens movidos
            start_settle = max(0.001, 2.0 * float(sent))
            start_tries = 1
            logger.info(
                "STM32 ← START_MOVE [frame=%d] | aguardará %.3fs (tries=%d) pela resposta",
                frame_id,
                start_tries * start_settle if start_tries > 1 else start_settle,
                start_tries,
            )
            ack = client.exchange(0x03, STM32RequestBuilder.start_move(frame_id), tries=start_tries, settle_delay_s=start_settle)
        except Exception as exc:
            cooldown = _spi_busy_cooldown(cfg)
            logger.warning("StartMove: sem ACK/BUSY; aguardando %.3fs e repetindo...", cooldown)
            time.sleep(cooldown)
            ack = client.exchange(0x03, STM32RequestBuilder.start_move(fid.next()), tries=max(start_tries, 80), settle_delay_s=max(start_settle, cooldown))
        try:
            data = STM32ResponseDecoder.start_move(ack)
            status_code = int(data.get("status", 0))
            status_label = {0: "started", 1: "ignored"}.get(status_code, "unknown")
            out = {
                "cmd": "start_move_ack",
                "frameId": int(data.get("frameId", 0)),
                "status": status_code,
                "status_label": status_label,
                "depth": data.get("depth"),
            }
            print(json.dumps(out, ensure_ascii=False))
        except Exception:
            logger.info("StartMove: ACK bruto=%s", ack)
    return sent


def _monitor_until_end(client: STM32Client, cfg: Dict[str, Any], *, timeout_s: float, logger: logging.Logger) -> Tuple[bool, str]:
    tmc = cfg.get("tmc5160", {})
    bus = int(tmc.get("bus", 0))
    devs = list(tmc.get("devs", [1, 2, 3]))
    speed = int(tmc.get("speed", 4_000_000))
    check_iv = float(cfg.get("monitor", {}).get("check_interval", 10))
    disable_status = bool(cfg.get("spi", {}).get("disable_status_poll", True)) if isinstance(cfg.get("spi"), dict) else True

    start_t = time.time()
    last_warn = 0.0
    frame_id = 0x41
    if not disable_status:
        qst_tries, qst_settle = _spi_params(cfg, "queue_status")
    while True:
        # 1) (Opcional) Consulta status da fila
        if not disable_status:
            try:
                logger.info(
                    "STM32 ← QUEUE_STATUS [frame=%d] | aguardará %.3fs (tries=%d) pela resposta",
                    frame_id,
                    qst_tries * qst_settle if qst_tries > 1 else qst_settle,
                    qst_tries,
                )
                resp = client.exchange(0x02, STM32RequestBuilder.queue_status(frame_id), tries=qst_tries, settle_delay_s=qst_settle)
                data = STM32ResponseDecoder.queue_status(resp)
                pct = data.get("pct", {})
                pmin = min(int(pct.get("x", 0)), int(pct.get("y", 0)), int(pct.get("z", 0)))
                logger.info("Progresso: X=%d%% Y=%d%% Z=%d%%", pct.get("x", 0), pct.get("y", 0), pct.get("z", 0))
                # Emite JSON do status traduzido
                try:
                    out = {
                        "cmd": "queue_status",
                        "frameId": int(data.get("frameId", 0)),
                        "status": int(data.get("status", 0)),
                        "pidErr": data.get("pidErr", {}),
                        "pct": pct,
                    }
                    print(json.dumps(out, ensure_ascii=False))
                except Exception:
                    pass
                if pmin >= 100:
                    return True, "concluído"
            except Exception as exc:
                # Impressão descontínua para evitar flood
                now = time.time()
                if now - last_warn > 2.0:
                    logger.info("QueueStatus indisponível: %s", exc)
                    last_warn = now
        else:
            # Sem polling de status: operação cega até 'timeout_s'
            logger.debug("Status STM32 desativado (spi.disable_status_poll=true). Aguardando execução...")

        # 2) Verifica TMC por falhas graves
        try:
            with TMC5160Configurator(bus=bus, device=int(devs[0]), speed_hz=speed, register_preset=TMC5160RegisterPreset.default()) as d:  # type: ignore
                g = d.read_register(REG_GSTAT).value
                drv = d.read_register(REG_DRV_STATUS).value
                overtemp = bool(drv & (1 << 26))
                otpw = bool(drv & (1 << 25))
                drv_err = bool(g & (1 << 1))
                if overtemp or otpw or drv_err:
                    _tmc_security_off_all(cfg, logger)
                    # LED aceso para indicar erro
                    _set_led(client, on=True)
                    return False, "TMC5160 falha (OT/OTPW/DRV_ERR)"
        except Exception as exc:
            logger.info("Falha ao consultar TMC5160: %s", exc)

        if (time.time() - start_t) > timeout_s:
            # Se não estamos consultando status, considerar como concluído (sem erro)
            if disable_status:
                return True, "tempo de execução decorrido (sem status)"
            return False, "timeout aguardando conclusão"
        time.sleep(max(0.05, check_iv))


def run_sequence(args: argparse.Namespace) -> int:
    cfg = _load_cfg(args.config)
    logger = _mk_logger()
    # Carrega sequência
    try:
        with open(args.sequence, "r", encoding="utf-8") as f:
            seq = json.load(f)
    except Exception as exc:
        logger.error("Falha ao abrir sequência: %s", exc)
        return 2

    ok, problems = _validate_sequence(cfg, seq)
    if not ok:
        for p in problems:
            logger.error("Arquivo ignorado: %s", p)
        logger.error("Execução abortada: limites violados em application.cfg")
        return 3

    # Override opcional do tempo mínimo de espera (linha de comando)
    try:
        spi_cfg = cfg.get("spi") if isinstance(cfg.get("spi"), dict) else {}
        if not isinstance(spi_cfg, dict):
            spi_cfg = {}
        if getattr(args, "min_wait_s", None) is not None:
            spi_cfg["min_wait_s"] = float(args.min_wait_s)
        if getattr(args, "queue_add_settle", None) is not None:
            spi_cfg["queue_add_settle"] = float(args.queue_add_settle)
        if getattr(args, "start_move_settle", None) is not None:
            spi_cfg["start_move_settle"] = float(args.start_move_settle)
        if getattr(args, "queue_status_settle", None) is not None:
            spi_cfg["queue_status_settle"] = float(args.queue_status_settle)
        if spi_cfg:
            cfg["spi"] = spi_cfg
    except Exception:
        pass

    # Conecta STM32
    try:
        st = cfg.get("stm32", {})
        client = STM32Client(bus=int(st.get("bus", 0)), dev=int(st.get("dev", 0)), speed_hz=int(st.get("speed", 1_000_000)))
    except Exception as exc:
        logger.error("Falha ao abrir SPI para STM32: %s", exc)
        return 4

    try:
        steps = []
        if isinstance(seq.get("steps"), list) and seq["steps"]:
            steps = list(seq["steps"])  # já validado
        else:
            # Compat: aplica tmc global e empacota moves como steps
            if isinstance(seq.get("tmc"), dict) and seq["tmc"]:
                _tmc_apply(cfg, seq["tmc"], logger)
            steps = [{"move": mv} for mv in list(seq.get("moves", []))]

        # LED: pisca (em execução)
        _set_led(client, blink_hz=1.0)
        total = _process_steps(client, cfg, seq, steps, logger)
        logger.info("Movimentos enfileirados: %d", total)
        ok, reason = _monitor_until_end(client, cfg, timeout_s=float(args.timeout), logger=logger)
        if ok:
            logger.info("Execução concluída com sucesso.")
            # Segurança pós-execução: TOFF=0
            _tmc_security_off_all(cfg, logger)
            # LED: apagado
            _set_led(client, on=False)
            return 0
        else:
            logger.error("Execução interrompida: %s", reason)
            # LED: aceso (falha)
            _set_led(client, on=True)
            return 5
    except Exception as exc:
        # Qualquer erro de execução: LED aceso e segurança TMC, se possível
        try:
            _set_led(client, on=True)
        except Exception:
            pass
        try:
            _tmc_security_off_all(cfg, logger)
        except Exception:
            pass
        logger.error("Falha durante execução: %s", exc)
        return 6
    finally:
        try:
            client.close()
        except Exception:
            pass


def status(args: argparse.Namespace) -> int:
    cfg = _load_cfg(args.config)
    try:
        st = cfg.get("stm32", {})
        client = STM32Client(bus=int(st.get("bus", 0)), dev=int(st.get("dev", 0)), speed_hz=int(st.get("speed", 1_000_000)))
    except Exception as exc:
        print("Falha ao abrir SPI para STM32:", exc)
        return 4
    try:
        qst_tries, qst_settle = _spi_params(cfg, "queue_status")
        resp = client.exchange(0x02, STM32RequestBuilder.queue_status(int(args.frame_id)), tries=qst_tries, settle_delay_s=qst_settle)
        data = STM32ResponseDecoder.queue_status(resp)
        pct = data.get("pct", {})
        done = min(int(pct.get("x", 0)), int(pct.get("y", 0)), int(pct.get("z", 0))) >= 100
        # Estimativas de contagem (quando usado junto do mesmo arquivo):
        total = int(args.total) if args.total is not None else None
        executed = None
        pending = None
        if total is not None:
            avg_pct = int(round((int(pct.get("x", 0)) + int(pct.get("y", 0)) + int(pct.get("z", 0))) / 3.0))
            executed = int((avg_pct * total) / 100)
            executed = min(max(0, executed), total)
            pending = max(0, total - executed)
        out = {
            "status": int(data.get("status", 0)),
            "pidErr": data.get("pidErr", {}),
            "pct": pct,
            "done": bool(done),
        }
        if total is not None:
            out.update({"total": total, "executed_est": executed, "pending_est": pending})
        print(json.dumps(out, ensure_ascii=False))
        return 0
    except Exception as exc:
        print("Falha ao consultar status:", exc)
        return 5
    finally:
        try:
            client.close()
        except Exception:
            pass


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Executa arquivo de sequência (passos/velocidades/PID/TMC) com monitoramento e segurança")
    sub = p.add_subparsers(dest="command", required=True)

    prun = sub.add_parser("run", help="Executa a sequência JSON")
    prun.add_argument("sequence", help="Caminho do arquivo de sequência JSON")
    prun.add_argument("--config", default="application.cfg", help="application.cfg com limites e conexões")
    prun.add_argument("--timeout", type=float, default=120.0, help="Timeout aguardando conclusão (s)")
    prun.add_argument("--min-wait-s", type=float, default=None, help="Tempo mínimo total de espera para ACKs SPI (>=5s; sobrescreve cfg.spi.min_wait_s)")
    prun.add_argument("--queue-add-settle", type=float, default=None, help="Sobrescreve cfg.spi.queue_add_settle (s)")
    prun.add_argument("--start-move-settle", type=float, default=None, help="Sobrescreve cfg.spi.start_move_settle (s)")
    prun.add_argument("--queue-status-settle", type=float, default=None, help="Sobrescreve cfg.spi.queue_status_settle (s)")
    prun.set_defaults(handler=run_sequence)

    pstat = sub.add_parser("status", help="Consulta MOVE_QUEUE_STATUS e imprime progresso/erro PID")
    pstat.add_argument("--config", default="application.cfg")
    pstat.add_argument("--frame-id", type=int, default=0x41)
    pstat.add_argument("--total", type=int, default=None, help="(Opcional) total de movimentos para estimar executados/pending")
    pstat.set_defaults(handler=status)

    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    ap = build_parser()
    args = ap.parse_args(list(argv) if argv is not None else None)
    handler = getattr(args, "handler", None)
    if callable(handler):
        return int(handler(args))
    ap.print_help()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
