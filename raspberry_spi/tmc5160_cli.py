#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""CLI para configurar o TMC5160 (ajuste validado por datasheet).

Permite ajustar mútiplos parmetros de forma unitaria ou combinada, com
validação de faixas e checagem de segurança de corrente (I_rms <= 2 A por padrão).

Uso rapido:
  - Ajustar MRES=1/256 e TOFF=3, preservando demais campos:
      python3 -m raspberry_spi.tmc5160_cli set --bus 0 --dev 3 \
          --microsteps 256 --toff 3

  - Limitar corrente: GLOBALSCALER=32, IRUN=8, IHOLD=2, TPOWERDOWN=0x14:
      python3 -m raspberry_spi.tmc5160_cli set --bus 0 --dev 3 \
          --globalscaler 32 --irun 8 --ihold 2 --ihold-delay 6 --tpowerdown 0x14

  - Status resumido (legí­vel):
      python3 -m raspberry_spi.tmc5160_cli status --bus 0 --dev 3
"""
from __future__ import annotations

import argparse
import math
from typing import Iterable, List, Optional, Sequence, Tuple
import os, sys

# Garante que o diretório deste arquivo está no sys.path para evitar
# importar um tmc5160.py de outro local (ex.: diretório raiz do repo).
_HERE = os.path.dirname(__file__)
if _HERE and _HERE not in sys.path:
    sys.path.insert(0, _HERE)

try:
    import sys
    sys.stdout.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
    sys.stderr.reconfigure(encoding="utf-8")  # type: ignore[attr-defined]
except Exception:
    pass

try:
    from .tmc5160 import (
        REG_GCONF,
        REG_GSTAT,
        REG_GLOBAL_SCALER,
        REG_IHOLD_IRUN,
        REG_TPOWERDOWN,
        REG_TPWMTHRS,
        REG_CHOPCONF,
        REG_COOLCONF,
        REG_PWMCONF,
        REG_DRV_STATUS,
        REG_DRV_CONF,
        REG_SHORT_CONF,
        TMC5160Configurator,
        TMC5160RegisterPreset,
        TMC5160ReadResult,
        decode_register_value,
    )
except Exception:  # execuÃ§Ã£o direta
    from tmc5160 import (  # type: ignore
        REG_GCONF,
        REG_GSTAT,
        REG_GLOBAL_SCALER,
        REG_IHOLD_IRUN,
        REG_TPOWERDOWN,
        REG_TPWMTHRS,
        REG_CHOPCONF,
        REG_COOLCONF,
        REG_PWMCONF,
        REG_DRV_STATUS,
        REG_DRV_CONF,
        REG_SHORT_CONF,
        TMC5160Configurator,
        TMC5160RegisterPreset,
        TMC5160ReadResult,
        decode_register_value,
    )


# --------- Utilidades de conversÃ£o/validaÃ§Ã£o ---------
def clamp(v: int, lo: int, hi: int) -> int:
    return hi if v > hi else lo if v < lo else v


def microsteps_to_mres(ms: int) -> int:
    table = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}
    if ms not in table:
        raise ValueError("microsteps deve ser um de: 256,128,64,32,16,8,4,2,1")
    return table[ms]


def mres_to_microsteps(code: int) -> Optional[int]:
    rev = {0: 256, 1: 128, 2: 64, 3: 32, 4: 16, 5: 8, 6: 4, 7: 2, 8: 1}
    return rev.get(code)


def estimate_irms(globalscaler: int, cs: int, r_sense: float = 0.075, v_fs: float = 0.325) -> float:
    """Estimativa de I_rms conforme modelagem usada no projeto.

    I_fs_rms = (GS/256) * V_fs / (R_sense*sqrt(2))
    I_rms    = I_fs_rms * (CS+1)/32
    """
    gs = 256 if globalscaler == 0 else globalscaler
    cs = clamp(int(cs), 0, 31)
    i_fs_rms = (gs / 256.0) * (v_fs / (r_sense * math.sqrt(2.0)))
    return i_fs_rms * ((cs + 1) / 32.0)


# --------- PersistÃªncia de Ãºltimos valores escritos (para W-only) ---------
import json
from pathlib import Path

STATE_DIR = Path.home() / ".config" / "tmc5160_tuner"
STATE_FILE = STATE_DIR / "state.json"

# Registradores write-only que queremos reapresentar no status
WRITE_ONLY_REGS = (
    REG_GLOBAL_SCALER,      # 0x0B
    REG_IHOLD_IRUN,         # 0x10
    REG_TPOWERDOWN,         # 0x11
    REG_TPWMTHRS,           # 0x13
    0x14,                   # TCOOLTHRS
    0x15,                   # THIGH
    REG_COOLCONF,           # 0x6D
    REG_PWMCONF,            # 0x70
    REG_DRV_CONF,           # 0x0A
    REG_SHORT_CONF,         # 0x09
)

REG_NAMES = {
    REG_GCONF: "GCONF",
    REG_GSTAT: "GSTAT",
    REG_GLOBAL_SCALER: "GLOBALSCALER",
    REG_IHOLD_IRUN: "IHOLD_IRUN",
    REG_TPOWERDOWN: "TPOWERDOWN",
    REG_TPWMTHRS: "TPWMTHRS",
    0x14: "TCOOLTHRS",
    0x15: "THIGH",
    REG_CHOPCONF: "CHOPCONF",
    REG_COOLCONF: "COOLCONF",
    REG_PWMCONF: "PWMCONF",
    REG_DRV_STATUS: "DRV_STATUS",
    REG_DRV_CONF: "DRV_CONF",
    REG_SHORT_CONF: "SHORT_CONF",
}

def _load_state() -> dict:
    try:
        if STATE_FILE.exists():
            return json.loads(STATE_FILE.read_text())
    except Exception:
        pass
    return {}

def _save_state(data: dict) -> None:
    try:
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        STATE_FILE.write_text(json.dumps(data, indent=2))
    except Exception:
        pass

def _record_writes(bus: int, dev: int, writes: List[Tuple[int,int]]) -> None:
    state = _load_state()
    key = f"{bus}.{dev}"
    entry = state.get(key, {})
    for addr, val in writes:
        if addr in WRITE_ONLY_REGS:
            entry[f"0x{addr:02X}"] = int(val) & 0xFFFFFFFF
    if entry:
        state[key] = entry
        _save_state(state)


def parse_int0(v: Optional[str]) -> Optional[int]:
    if v is None:
        return None
    return int(v, 0)


# --------- Argumentos comuns ---------
def _add_common_spi_arguments(p: argparse.ArgumentParser) -> None:
    p.add_argument("--bus", type=int, default=0, help="Barramento SPI (default: 0)")
    p.add_argument("--dev", type=int, default=1, help="Dispositivo SPI (default: 1)")
    p.add_argument("--speed", type=int, default=4_000_000, help="Velocidade SPI em Hz")


def _build_set_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Ajusta mÃºltiplos parÃ¢metros do TMC5160 com validaÃ§Ã£o e seguranÃ§a")
    _add_common_spi_arguments(p)
    # Segurança/calculo
    p.add_argument("--sense-resistor", type=float, default=0.075, help="R_SENSE em ohms (default: 0.075)")
    p.add_argument("--max-irms", type=float, default=2.0, help="Corrente RMS mÃ¡xima permitida (A). Default: 2.0")
    # SaÃ­da estruturada
    p.add_argument("--json", action="store_true", help="Saida JSON estruturada do que foi aplicado")

    # GLOBALSCALER (0x0B)
    p.add_argument("--globalscaler", type=int, help="GLOBALSCALER (0 ou 32..255; 0 equivale a 256)")

    # IHOLD_IRUN (0x10)
    p.add_argument("--ihold", type=int, help="IHOLD (0..31)")
    p.add_argument("--irun", type=int, help="IRUN (0..31)")
    p.add_argument("--ihold-delay", type=int, dest="iholddelay", help="IHOLDDELAY (0..15)")

    # TPOWERDOWN/TPWMTHRS/TCOOLTHRS/THIGH
    p.add_argument("--tpowerdown", type=lambda x: int(x, 0), help="TPOWERDOWN (0..255)")
    p.add_argument("--tpwmthrs", type=lambda x: int(x, 0), help="TPWMTHRS (20 bits)")
    p.add_argument("--tcoolthrs", type=lambda x: int(x, 0), help="TCOOLTHRS (20 bits)")
    p.add_argument("--thigh", type=lambda x: int(x, 0), help="THIGH (20 bits)")

    # CHOPCONF (0x6C)
    p.add_argument("--toff", type=int, help="TOFF (0 desliga; 1..15 vÃ¡lido)")
    p.add_argument("--tbl", type=int, choices=[0, 1, 2, 3], help="TBL index (0..3)")
    p.add_argument("--hstrt", type=int, help="HSTRT (0..7)")
    p.add_argument("--hend", type=int, help="HEND (0..15) [faixa efetiva -3..+12]")
    p.add_argument("--chm", type=int, choices=[0, 1], help="CHM (0 SpreadCycle, 1 FastDecay)")
    p.add_argument("--tpfd", type=int, help="TPFD (0..15)")
    p.add_argument("--vhighchm", type=int, choices=[0, 1], help="VHIGHCHM (0/1)")
    p.add_argument("--vhighfs", type=int, choices=[0, 1], help="VHIGHFS (0/1)")
    p.add_argument("--mres", type=int, help="MRES code (0..8) => microsteps")
    p.add_argument("--microsteps", type=int, choices=[256,128,64,32,16,8,4,2,1], help="define MRES por microsteps")
    p.add_argument("--interpolate", dest="interpolate", action="store_true", help="INTPOL=1")
    p.add_argument("--no-interpolate", dest="interpolate", action="store_false", help="INTPOL=0")
    p.set_defaults(interpolate=None)

    # COOLCONF (0x6D)
    p.add_argument("--semin", type=int, help="SEMIN 0..15")
    p.add_argument("--semax", type=int, help="SEMAX 0..15")
    p.add_argument("--seup", type=int, help="SEUP 0..3")
    p.add_argument("--sedn", type=int, help="SEDN 0..3")
    p.add_argument("--seimin", type=int, choices=[0,1], help="SEIMIN 0/1")
    p.add_argument("--sgt", type=int, help="SGT -64..+63")

    # PWMCONF (0x70)
    p.add_argument("--pwm-autoscale", dest="pwm_autoscale", action="store_true")
    p.add_argument("--no-pwm-autoscale", dest="pwm_autoscale", action="store_false")
    p.set_defaults(pwm_autoscale=None)
    p.add_argument("--pwm-autograd", dest="pwm_autograd", action="store_true")
    p.add_argument("--no-pwm-autograd", dest="pwm_autograd", action="store_false")
    p.set_defaults(pwm_autograd=None)
    p.add_argument("--freewheel", type=int, choices=[0,1,2,3], help="FREEWHEEL 0..3")
    p.add_argument("--pwm-freq", type=int, choices=[0,1,2,3], help="pwm_freq index 0..3")
    p.add_argument("--pwm-ofs", type=int, help="PWM_OFS 0..255")
    p.add_argument("--pwm-grad", type=int, help="PWM_GRAD 0..255")
    p.add_argument("--pwm-lim", type=int, help="PWM_LIM 0..15")
    p.add_argument("--pwm-reg", type=int, help="PWM_REG 1..15")

    # DRV_CONF (0x0A)
    p.add_argument("--filt-isense", type=int, help="FILT_ISENSE 0..3")
    p.add_argument("--bbmtime", type=int, help="BBMTIME 0..31")
    p.add_argument("--bbmclks", type=int, help="BBMCLKS 0..15")
    p.add_argument("--drvstrength", type=int, help="DRVSTRENGTH 0..3")

    # SHORT_CONF (0x09)
    p.add_argument("--s2vs-level", type=int, help="S2VS_LEVEL 4..15")
    p.add_argument("--s2g-level", type=int, help="S2G_LEVEL 2..15")
    p.add_argument("--shortfilter", type=int, help="SHORTFILTER 0..3")
    p.add_argument("--shortdelay", type=int, choices=[0,1], help="SHORTDELAY 0/1")

    return p


def _build_status_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Lê parametros principais legiveis do TMC5160")
    _add_common_spi_arguments(p)
    p.add_argument("--clear-gstat", action="store_true", help="Limpa GSTAT (0x07) antes de ler")
    p.add_argument("--errors", action="store_true", help="Imprime apenas a secao de erros (GSTAT/DRV_STATUS)")
    p.add_argument("--json", action="store_true", help="Saida JSON estruturada (erros e registradores)")
    return p


def _assemble_chopconf(current: int, **fields: int) -> int:
    v = int(current)
    def set_bits(mask: int, shift: int, value: Optional[int]) -> None:
        nonlocal v
        if value is None:
            return
        mv = int(value)
        v = (v & ~mask) | ((mv << shift) & mask)

    # Campos
    if fields.get("toff") is not None:
        v = (v & ~0x0F) | (clamp(int(fields["toff"]), 0, 15) & 0x0F)
    if fields.get("hstrt") is not None:
        v = (v & ~(0x7 << 4)) | ((clamp(int(fields["hstrt"]), 0, 7) & 0x7) << 4)
    if fields.get("hend") is not None:
        v = (v & ~(0xF << 7)) | ((clamp(int(fields["hend"]), 0, 15) & 0xF) << 7)
    if fields.get("tbl") is not None:
        v = (v & ~(0x3 << 15)) | ((int(fields["tbl"]) & 0x3) << 15)
    if fields.get("chm") is not None:
        if int(fields["chm"]) not in (0,1):
            raise ValueError("chm deve ser 0 ou 1")
        if int(fields["chm"]) == 1:
            v |= (1 << 14)
        else:
            v &= ~(1 << 14)
    if fields.get("vhighchm") is not None:
        if int(fields["vhighchm"]) == 1:
            v |= (1 << 19)
        else:
            v &= ~(1 << 19)
    if fields.get("vhighfs") is not None:
        if int(fields["vhighfs"]) == 1:
            v |= (1 << 18)
        else:
            v &= ~(1 << 18)
    if fields.get("tpfd") is not None:
        v = (v & ~(0xF << 20)) | ((clamp(int(fields["tpfd"]), 0, 15) & 0xF) << 20)
    # MRES e INTPOL
    mres = fields.get("mres")
    micro = fields.get("microsteps")
    if micro is not None:
        mres = microsteps_to_mres(int(micro))
    if mres is not None:
        v = (v & ~(0xF << 24)) | ((clamp(int(mres), 0, 8) & 0xF) << 24)
    if fields.get("interpolate") is not None:
        if fields["interpolate"]:
            v |= (1 << 28)
        else:
            v &= ~(1 << 28)
    return v


def _saturate_and_validate_currents(args, gs: Optional[int], ihold: Optional[int], irun: Optional[int]) -> None:
    # Validar faixas bÃ¡sicas
    if gs is not None and not (gs == 0 or 32 <= gs <= 255):
        raise ValueError("globalscaler deve ser 0 ou 32..255 (0 equivale a 256)")
    if ihold is not None and not (0 <= ihold <= 31):
        raise ValueError("ihold deve ser 0..31")
    if irun is not None and not (0 <= irun <= 31):
        raise ValueError("irun deve ser 0..31")

    # Checagem de segurança I_rms para IRUN
    if gs is not None and irun is not None:
        irms = estimate_irms(gs, irun, r_sense=args.sense_resistor)
        if irms > args.max_irms:
            raise ValueError(
                f"SeguranÃ§a: IRMS estimada {irms:.2f} A excede {args.max_irms:.2f} A. "
                "Reduza globalscaler e/ou irun."
            )


def run_set(argv: Sequence[str]) -> int:
    p = _build_set_parser()
    args = p.parse_args(list(argv))

    gs = args.globalscaler
    ihold = args.ihold
    irun = args.irun
    _saturate_and_validate_currents(args, gs, ihold, irun)

    writes: List[Tuple[int, int]] = []

    # Estimativa de IRMS para informar sempre no set()
    # Usa valores novos (se passados) ou os ultimos persistidos (state.json).
    try:
        state = _load_state()
        key = f"{args.bus}.{args.dev}"
        prev = state.get(key, {}) if isinstance(state, dict) else {}
        prev_gs = int(prev.get("0x0B", 256))
        prev_ihold_irun = int(prev.get("0x10", 0))
        prev_irun = (prev_ihold_irun >> 8) & 0x1F
    except Exception:
        prev_gs, prev_irun = 256, 0
    gs_eff = gs if gs is not None else prev_gs
    irun_eff = irun if irun is not None else prev_irun
    irms_est = None
    try:
        irms_est = estimate_irms(gs_eff, irun_eff, r_sense=args.sense_resistor)
        if not getattr(args, "json", False):
            print(
                "IRMS estimada com GS={} e IRUN={} (R_SENSE={} ohm): {:.2f} A".format(
                    gs_eff, irun_eff, args.sense_resistor, irms_est
                )
            )
    except Exception:
        irms_est = None

    # GLOBALSCALER (0x0B) â€” write-only
    if gs is not None:
        if not (gs == 0 or 32 <= gs <= 255):
            raise ValueError("globalscaler deve ser 0 ou 32..255")
        writes.append((REG_GLOBAL_SCALER, gs & 0xFF))

    # IHOLD_IRUN (0x10) â€” write-only
    if (ihold is not None) or (irun is not None) or (args.iholddelay is not None):
        _ihold = clamp(ihold if ihold is not None else 0, 0, 31)
        _irun = clamp(irun if irun is not None else 0, 0, 31)
        _delay = clamp(args.iholddelay if args.iholddelay is not None else 6, 0, 15)
        ihold_irun = (_ihold & 0x1F) | ((_irun & 0x1F) << 8) | ((_delay & 0x0F) << 16)
        writes.append((REG_IHOLD_IRUN, ihold_irun))

    # TPOWERDOWN/TPWMTHRS/TCOOLTHRS/THIGH (write-only)
    if args.tpowerdown is not None:
        tp = args.tpowerdown & 0xFF
        if tp < 2:
            print("Erro: TPOWERDOWN deve ser >= 2 (datasheet TMC5160 recomenda mínimo 2).")
            return 1
        writes.append((REG_TPOWERDOWN, tp))
    for addr, val in ((0x14, args.tcoolthrs), (0x15, args.thigh), (REG_TPWMTHRS, args.tpwmthrs)):
        if val is not None:
            writes.append((addr, val & 0xFFFFF))

    # CHOPCONF â€” RMW
    chop_wanted = any(x is not None for x in (
        args.toff, args.tbl, args.hstrt, args.hend, args.chm, args.tpfd,
        args.vhighchm, args.vhighfs, args.mres, args.microsteps, args.interpolate
    ))
    if chop_wanted:
        with TMC5160Configurator(bus=args.bus, device=args.dev, speed_hz=args.speed,
                                 register_preset=TMC5160RegisterPreset(writes=tuple())) as d:
            old = d.read_register(REG_CHOPCONF).value
        new = _assemble_chopconf(
            old,
            toff=args.toff,
            tbl=args.tbl,
            hstrt=args.hstrt,
            hend=args.hend,
            chm=args.chm,
            tpfd=args.tpfd,
            vhighchm=args.vhighchm,
            vhighfs=args.vhighfs,
            mres=args.mres,
            microsteps=args.microsteps,
            interpolate=args.interpolate,
        )
        writes.append((REG_CHOPCONF, new))

    # COOLCONF (0x6D) â€” compose when any provided
    cool_wanted = any(x is not None for x in (
        args.semin, args.semax, args.seup, args.sedn, args.seimin, args.sgt
    ))
    if cool_wanted:
        semin = clamp(args.semin if args.semin is not None else 0, 0, 15)
        semax = clamp(args.semax if args.semax is not None else 0, 0, 15)
        seup  = clamp(args.seup if args.seup is not None else 0, 0, 3)
        sedn  = clamp(args.sedn if args.sedn is not None else 0, 0, 3)
        seimin = 1 if (args.seimin and int(args.seimin) == 1) else 0
        sgt = args.sgt if args.sgt is not None else 0
        if not (-64 <= int(sgt) <= 63):
            raise ValueError("sgt deve estar em -64..+63")
        # SGT Ã© 7 bits com sinal; aqui apenas repassamos valor signed em 7 bits two's complement
        sgt_val = (int(sgt) & 0x7F)
        cool = (semin & 0x0F) | ((seup & 0x03) << 5) | ((semax & 0x0F) << 8) | ((sedn & 0x03) << 13) \
               | ((seimin & 0x01) << 15) | (sgt_val << 16)
        writes.append((REG_COOLCONF, cool))

    # PWMCONF (0x70)
    pwm_wanted = any(x is not None for x in (
        args.pwm_autoscale, args.pwm_autograd, args.freewheel, args.pwm_freq,
        args.pwm_ofs, args.pwm_grad, args.pwm_lim, args.pwm_reg
    ))
    if pwm_wanted:
        pwm = 0
        if args.pwm_autoscale is not None and args.pwm_autoscale:
            pwm |= (1 << 18)
        if args.pwm_autograd is not None and args.pwm_autograd:
            pwm |= (1 << 19)
        if args.freewheel is not None:
            pwm |= ((int(args.freewheel) & 0x3) << 20)
        if args.pwm_freq is not None:
            pwm |= ((int(args.pwm_freq) & 0x3) << 16)
        if args.pwm_reg is not None:
            pwm |= ((clamp(int(args.pwm_reg), 1, 15) & 0x0F) << 24)
        if args.pwm_lim is not None:
            pwm |= ((clamp(int(args.pwm_lim), 0, 15) & 0x0F) << 28)
        if args.pwm_grad is not None:
            pwm |= ((clamp(int(args.pwm_grad), 0, 255) & 0xFF) << 8)
        if args.pwm_ofs is not None:
            pwm |= (clamp(int(args.pwm_ofs), 0, 255) & 0xFF)
        writes.append((REG_PWMCONF, pwm))

    # DRV_CONF (0x0A)
    drv_wanted = any(x is not None for x in (
        args.filt_isense, args.bbmtime, args.bbmclks, args.drvstrength
    ))
    if drv_wanted:
        val = 0
        if args.filt_isense is not None:
            val |= ((clamp(int(args.filt_isense),0,3) & 0x3) << 20)
        if args.drvstrength is not None:
            val |= ((clamp(int(args.drvstrength),0,3) & 0x3) << 18)
        if args.bbmclks is not None:
            val |= ((clamp(int(args.bbmclks),0,15) & 0xF) << 8)
        if args.bbmtime is not None:
            val |= (clamp(int(args.bbmtime),0,31) & 0x1F)
        writes.append((REG_DRV_CONF, val))

    # SHORT_CONF (0x09)
    short_wanted = any(x is not None for x in (
        args.s2vs_level, args.s2g_level, args.shortfilter, args.shortdelay
    ))
    if short_wanted:
        s2vs = args.s2vs_level
        s2g = args.s2g_level
        if s2vs is not None and not (4 <= int(s2vs) <= 15):
            raise ValueError("s2vs-level deve ser 4..15")
        if s2g is not None and not (2 <= int(s2g) <= 15):
            raise ValueError("s2g-level deve ser 2..15")
        val = 0
        if s2vs is not None:
            val |= ((int(s2vs) & 0xF) << 4)
        if s2g is not None:
            val |= ((int(s2g) & 0xF) << 8)
        if args.shortfilter is not None:
            val |= ((clamp(int(args.shortfilter),0,3) & 0x3) << 12)
        if args.shortdelay is not None:
            if int(args.shortdelay) == 1:
                val |= (1 << 15)
        writes.append((REG_SHORT_CONF, val))

    # Aplicar
    if not writes:
        if getattr(args, "json", False):
            import json as _json
            payload = {
                "bus": args.bus,
                "dev": args.dev,
                "speed": args.speed,
                "irms": {
                    "gs": gs_eff,
                    "irun": irun_eff,
                    "r_sense": args.sense_resistor,
                    "estimate": irms_est,
                },
                "writes": [],
                "ok": False,
                "error": "Nada a escrever. Informe parâmetros com --param valor",
            }
            print(_json.dumps(payload, ensure_ascii=False))
        else:
            print("Nada a escrever. Informe parÃ¢metros com --param valor")
        return 1

    configurator = TMC5160Configurator(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset(writes=tuple()),
    )

    with configurator as driver:  # type: ignore
        _record_writes(args.bus, args.dev, writes)
        if not getattr(args, "json", False):
            print(f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para aplicar parÃ¢metros")
            for addr, val in writes:
                print(f" - 0x{addr:02X} = 0x{val:08X}")
        res = driver.apply_registers(writes)
        if getattr(args, "json", False):
            import json as _json
            payload = {
                "bus": args.bus,
                "dev": args.dev,
                "speed": args.speed,
                "irms": {
                    "gs": gs_eff,
                    "irun": irun_eff,
                    "r_sense": args.sense_resistor,
                    "estimate": irms_est,
                },
                "writes": [
                    {
                        "address": int(addr),
                        "address_hex": f"0x{addr:02X}",
                        "name": REG_NAMES.get(addr, None),
                        "value": int(val),
                        "value_hex": f"0x{val:08X}",
                        "value_bin": _bin32(int(val)),
                    }
                    for addr, val in writes
                ],
                "results": [
                    {
                        "address": (int(r.address) & 0x7F),
                        "address_hex": f"0x{(int(r.address) & 0x7F):02X}",
                        "status": int(r.status.raw),
                        "status_hex": f"0x{int(r.status.raw):02X}",
                        "response": r.raw_hex,
                        "previous_data": int(r.previous_data),
                        "previous_data_hex": f"0x{int(r.previous_data):08X}",
                    }
                    for r in res
                ],
                "ok": True,
            }
            print(_json.dumps(payload, ensure_ascii=False))
        else:
            for r in res:
                print(f"  -> 0x{r.address:02X} <= 0x{r.value:08X}  status=0x{r.status.raw:02X}")
    return 0


def _format_read_result(res: TMC5160ReadResult) -> str:
    name = {REG_GCONF:"GCONF", REG_CHOPCONF:"CHOPCONF", REG_DRV_STATUS:"DRV_STATUS", REG_GSTAT:"GSTAT"}.get(res.address, f"0x{res.address:02X}")
    lines = [
        f"{name} (0x{res.address:02X}) => 0x{res.value:08X}",
        f"  Requisição  : status=0x{res.request.status.raw:02X}, resposta={res.request.raw_hex}",
        f"  Resposta útil: status=0x{res.reply.status.raw:02X}, resposta={res.reply.raw_hex}",
    ]
    dec = decode_register_value(res.address, res.value)
    if dec:
        lines.append("  Tradução:")
        for d in dec:
            lines.append(f"    - {d}")
    return "\n".join(lines)


def run_status(argv: Sequence[str]) -> int:
    p = _build_status_parser()
    args = p.parse_args(list(argv))
    configurator = TMC5160Configurator(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset.default(),
    )
    with configurator as driver:  # type: ignore
        print(f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para consultar o TMC5160")
        if args.clear_gstat:
            driver.write_register(REG_GSTAT, 0x00000007)
        regs = (REG_GSTAT, REG_DRV_STATUS, REG_GCONF, REG_CHOPCONF)
        results = driver.read_registers(regs)
        # Preparar dados
        gstat_val = next((r.value for r in results if r.address == REG_GSTAT), 0)
        drv_val = next((r.value for r in results if r.address == REG_DRV_STATUS), 0)
        errors = _summarize_errors(gstat_val, drv_val)
        # JSON?
        if getattr(args, "json", False):
            import json as _json
            reg_names = {REG_GCONF:"GCONF", REG_CHOPCONF:"CHOPCONF", REG_DRV_STATUS:"DRV_STATUS", REG_GSTAT:"GSTAT"}
            payload = {
                "bus": args.bus,
                "dev": args.dev,
                "speed": args.speed,
                "errors": errors,
            }
            # Inclui sempre campos brutos de GSTAT e DRV_STATUS com binário
            r_g = next((r for r in results if r.address == REG_GSTAT), None)
            r_d = next((r for r in results if r.address == REG_DRV_STATUS), None)
            if r_g is not None:
                payload["gstat"] = {
                    "value": r_g.value,
                    "value_hex": f"0x{r_g.value:08X}",
                    "req": _bin5_bytes(r_g.request.response),
                    "resp": _bin5_bytes(r_g.reply.response),
                }
            if r_d is not None:
                payload["drv_status"] = {
                    "value": r_d.value,
                    "value_hex": f"0x{r_d.value:08X}",
                    "req": _bin5_bytes(r_d.request.response),
                    "resp": _bin5_bytes(r_d.reply.response),
                }
            if not getattr(args, "errors", False):
                regs_out = []
                for r in results:
                    regs_out.append({
                        "name": reg_names.get(r.address, f"0x{r.address:02X}"),
                        "address": r.address,
                        "value": r.value,
                        "value_hex": f"0x{r.value:08X}",
                        "req": _bin5_bytes(r.request.response),
                        "resp": _bin5_bytes(r.reply.response),
                    })
                payload["registers"] = regs_out
            print(_json.dumps(payload, ensure_ascii=False, indent=2))
            return 0
        # Texto normal: imprime apenas erros se --errors
        if getattr(args, "errors", False):
            # Mostra valores/binários dos regs de erro
            r_g = next((r for r in results if r.address == REG_GSTAT), None)
            r_d = next((r for r in results if r.address == REG_DRV_STATUS), None)
            if r_g is not None:
                print(f"- GSTAT (0x01) => 0x{r_g.value:08X}")
                print(f"  binario: {_bin32(r_g.value)}")
                print(f"  req: {_bin5_bytes(r_g.request.response)}")
                print(f"  resp): {_bin5_bytes(r_g.reply.response)}")
            if r_d is not None:
                print(f"- DRV_STATUS (0x6F) => 0x{r_d.value:08X}")
                print(f"  binario: {_bin32(r_d.value)}")
                print(f"  req: {_bin5_bytes(r_d.request.response)}")
                print(f"  resp: {_bin5_bytes(r_d.reply.response)}")
            if errors:
                print("Erros detectados:")
                for e in errors:
                    print(f"  - {e}")
            else:
                print("Erros: nenhum")
            return 0
        # Caso padrão: imprime registradores + binários + resumo de erros
        for r in results:
            print(_format_read_result(r))
            print(f"  Requisicao binaria: {_bin5_bytes(r.request.response)}")
            print(f"  Resposta   binaria: {_bin5_bytes(r.reply.response)}")
        if errors:
            print("Erros detectados:")
            for e in errors:
                print(f"  - {e}")
        else:
            print("Erros: nenhum")
    return 0


def _build_get_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Lê registradores legíveis do TMC5160 em ordem solicitada")
    _add_common_spi_arguments(p)
    p.add_argument("register", nargs="+", help=(
        "Lista de registradores (alias conhecidos: gconf, gstat, ioin, tstep, xactual, vactual, "
        "mscnt, mscuract, chopconf, drv_status). Também aceita endereços numéricos (ex.: 0x6F)."
    ))
    return p


def _bin32(v: int) -> str:
    b = f"{v:032b}"
    # Agrupa em nybbles para leitura
    return " ".join(b[i:i+4] for i in range(0, 32, 4))


def _bin5_bytes(bs: tuple[int, int, int, int, int]) -> str:
    return " ".join(f"{x:08b}" for x in bs)


def _summarize_errors(gstat_val: int, drv_val: int) -> list[str]:
    errs: list[str] = []
    # GSTAT
    if gstat_val & (1 << 2):
        errs.append("GSTAT.UV_CP (subtensao charge pump)")
    if gstat_val & (1 << 1):
        errs.append("GSTAT.DRV_ERR (falha de driver)")
    # DRV_STATUS
    if drv_val & (1 << 26):
        errs.append("DRV_STATUS.OT (sobretemperatura)")
    if drv_val & (1 << 25):
        errs.append("DRV_STATUS.OTPW (pre-aviso sobretemperatura)")
    if drv_val & (1 << 24):
        errs.append("DRV_STATUS.S2GA (curto a terra fase A)")
    if drv_val & (1 << 23):
        errs.append("DRV_STATUS.S2GB (curto a terra fase B)")
    if drv_val & (1 << 22):
        errs.append("DRV_STATUS.S2VSA (curto a Vsup fase A)")
    if drv_val & (1 << 21):
        errs.append("DRV_STATUS.S2VSB (curto a Vsup fase B)")
    if drv_val & (1 << 20):
        errs.append("DRV_STATUS.OLA (open-load fase A)")
    if drv_val & (1 << 19):
        errs.append("DRV_STATUS.OLB (open-load fase B)")
    return errs


def run_get(argv: Sequence[str]) -> int:
    # Registradores legíveis (R/RW) suportados
    readable: dict[str, int] = {
        "gconf": 0x00,
        "gstat": 0x01,
        "ioin": 0x04,
        "tstep": 0x12,
        "xactual": 0x21,
        "vactual": 0x22,
        "mscnt": 0x6A,
        "mscuract": 0x6B,
        "chopconf": 0x6C,
        "drv_status": 0x6F,
        "drvstatus": 0x6F,
        "drv": 0x6F,
    }
    # Registradores write-only para aviso
    write_only = {
        0x0B, 0x10, 0x11, 0x13, 0x14, 0x15, 0x6D, 0x70, 0x0A, 0x09
    }

    p = _build_get_parser()
    args = p.parse_args(list(argv))

    # Resolve lista na ordem pedida
    req_addrs: list[tuple[str,int]] = []
    warnings: list[str] = []
    for item in args.register:
        key = item.strip().lower()
        addr: Optional[int] = None
        if key in readable:
            addr = readable[key]
            req_addrs.append((key, addr))
            continue
        # Tenta numérico
        try:
            val = int(key, 0)
            if not (0 <= val <= 0x7F):
                warnings.append(f"Aviso: endereço fora da faixa 0x00..0x7F: {item}")
                continue
            # Se for write-only: avisa e não lê
            if val in write_only:
                warnings.append(f"Aviso: 0x{val:02X} é write-only — solicite via 'status' para ver últimos valores enviados, ou use 'set'.")
                continue
            # Se não estiver na lista ‘readable’ mas é potencialmente legível, lê mesmo assim
            req_addrs.append((f"0x{val:02X}", val))
        except Exception:
            warnings.append(f"Aviso: registrador/alias desconhecido: {item}")

    if warnings:
        for w in warnings:
            print(w)

    if not req_addrs:
        print("Nada para ler.")
        return 1

    configurator = TMC5160Configurator(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset.default(),
    )
    with configurator as driver:  # type: ignore
        print(f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para leitura")
        for name, addr in req_addrs:
            try:
                r = driver.read_register(addr)
                print(f"{name} (0x{addr:02X}) => 0x{r.value:08X}")
                print(f"  binário: {_bin32(r.value)}")
                dec = decode_register_value(addr, r.value)
                if dec:
                    print("  Tradução:")
                    for d in dec:
                        print(f"    - {d}")
            except Exception as exc:
                print(f"Erro ao ler 0x{addr:02X}: {exc}")
    return 0


def _build_errors_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Consulta erros em GSTAT/DRV_STATUS do TMC5160")
    _add_common_spi_arguments(p)
    p.add_argument("--devs", type=str, help="Lista de dispositivos (ex.: 1,2,3). Se ausente, usa --dev.")
    return p


def run_errors(argv: Sequence[str]) -> int:
    p = _build_errors_parser()
    args = p.parse_args(list(argv))

    if args.devs:
        dev_list = []
        for tok in str(args.devs).split(','):
            tok = tok.strip()
            if not tok:
                continue
            try:
                dev_list.append(int(tok))
            except Exception:
                print(f"Aviso: dev invalido: {tok}")
        if not dev_list:
            print("Nenhum dispositivo valido em --devs")
            return 1
    else:
        dev_list = [args.dev]

    for dev in dev_list:
        configurator = TMC5160Configurator(
            bus=args.bus,
            device=dev,
            speed_hz=args.speed,
            register_preset=TMC5160RegisterPreset.default(),
        )
        with configurator as driver:  # type: ignore
            print(f"Abrindo SPI bus={args.bus} dev={dev} a {args.speed} Hz para verificar erros")
            r_gstat = driver.read_register(REG_GSTAT)
            r_drv = driver.read_register(REG_DRV_STATUS)
            errors = _summarize_errors(r_gstat.value, r_drv.value)
            print(f"- GSTAT (0x01) => 0x{r_gstat.value:08X}")
            print(f"  binario: {_bin32(r_gstat.value)}")
            print(f"  resposta: {_bin5_bytes(r_gstat.reply.response)}")
            print(f"- DRV_STATUS (0x6F) => 0x{r_drv.value:08X}")
            print(f"  binario: {_bin32(r_drv.value)}")
            print(f"  resposta: {_bin5_bytes(r_drv.reply.response)}")
            if errors:
                print("Erros detectados:")
                for e in errors:
                    print(f"  - {e}")
            else:
                print("Erros: nenhum")
    return 0


def _build_security_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="Security operations for TMC5160")
    _add_common_spi_arguments(p)
    p.add_argument(
        "mode",
        choices=["off", "on"],
        help=(
            "off: desabilita corrente (IHOLD/IRUN=0, FREEWHEEL=1, TOFF=0); "
            "on: preset seguro (IHOLD/IRUN=0, FREEWHEEL=0, TOFF=3)."
        ),
    )
    return p


def run_security(argv: Sequence[str]) -> int:
    p = _build_security_parser()
    args = p.parse_args(list(argv))

    # Modo OFF: desabilitar corrente e chopper; Modo ON: preset seguro
    writes: List[Tuple[int, int]] = []

    if args.mode == "off":
        # 1) Zero currents (IHOLD/IRUN = 0)
        writes.append((REG_IHOLD_IRUN, 0x00000000))
        # 2) PWMCONF: FREEWHEEL=1 (coast / freewheeling)
        pwm_val = (1 & 0x3) << 20
        writes.append((REG_PWMCONF, pwm_val))
        # 3) CHOPCONF: TOFF=0 (disable chopper/outputs)
        with TMC5160Configurator(
            bus=args.bus,
            device=args.dev,
            speed_hz=args.speed,
            register_preset=TMC5160RegisterPreset(writes=tuple()),
        ) as d:
            old_chop = d.read_register(REG_CHOPCONF).value
        new_chop = (old_chop & ~0x0F) | 0x00
        writes.append((REG_CHOPCONF, new_chop))
        action_desc = "aplicar security off"
    else:
        # Security ON: preset seguro de baixa corrente e chopper ativo
        # GLOBALSCALER baixo (32), IHOLD/IRUN mínimos, FREEWHEEL=0, TOFF=3, TBL=2, HSTRT/HEND=5/2,
        # MRES=1/16 com interpolação.
        writes.append((REG_GLOBAL_SCALER, 32))  # 0x20
        # iholddelay=6, irun=0, ihold=0 (corrente mínima 1/32)
        writes.append((REG_IHOLD_IRUN, (6 << 16)))
        # PWMCONF default seguro (autoscale/autograd ON, freewheel=0, ofs/grad moderados)
        writes.append((REG_PWMCONF, 0xC10D0024))
        # Monta CHOPCONF seguro
        toff = 3
        hstrt = 5
        hend = 2
        tbl = 2
        mres = 4  # 1/16
        intpol = 1
        chop = 0
        chop |= (toff & 0x0F)
        chop |= (hstrt & 0x07) << 4
        chop |= (hend & 0x0F) << 7
        chop |= (tbl & 0x03) << 15
        chop |= (mres & 0x0F) << 24
        chop |= (intpol & 0x01) << 28
        writes.append((REG_CHOPCONF, chop))
        action_desc = "aplicar security on (preset seguro)"

    configurator = TMC5160Configurator(
        bus=args.bus,
        device=args.dev,
        speed_hz=args.speed,
        register_preset=TMC5160RegisterPreset(writes=tuple()),
    )
    with configurator as driver:  # type: ignore
        print(f"Abrindo SPI bus={args.bus} dev={args.dev} a {args.speed} Hz para {action_desc}")
        _record_writes(args.bus, args.dev, writes)
        for addr, val in writes:
            print(f" - 0x{addr:02X} = 0x{val:08X}")
        res = driver.apply_registers(writes)
        for r in res:
            print(f"  -> 0x{r.address:02X} <= 0x{r.value:08X}  status=0x{r.status.raw:02X}")
    if args.mode == "off":
        print("Security off aplicado: IHOLD/IRUN=0, PWM FREEWHEEL=1 (coast), CHOPCONF.TOFF=0.")
        print("Motores sem corrente. Para reativar, use 'security on' ou 'set'.")
    else:
        print("Security on aplicado: GS=32, IHOLD/IRUN=0, TOFF=3, TBL=2, HSTRT/HEND=5/2, MRES=1/16 + INTPOL.")
        print("Sistema em estado seguro com corrente mínima e chopper ligado.")
    return 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    import sys
    if argv is None:
        argv = sys.argv[1:]
    if not argv:
        print("Uso: set|status [opções] (use --help em cada subcomando)")
        return 1
    cmd, *rest = list(argv)
    if cmd == "set":
        return run_set(rest)
    if cmd == "status":
        return run_status(rest)
    if cmd == "get":
        return run_get(rest)
    if cmd == "security":
        return run_security(rest)
    if cmd == "errors":
        return run_errors(rest)
    print("Comando desconhecido:", cmd)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
