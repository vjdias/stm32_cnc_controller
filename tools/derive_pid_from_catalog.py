#!/usr/bin/env python3
"""
Deriva (calcula) Kp, Ki e Kd para o firmware a partir do catálogo
de perfis (tuning_profiles.py), usando as seguintes regras:

Contínuo → Firmware (inteiros, SHIFT=8):
  - Kp_fw = round(Kp_cont · 2^8)
  - Kd_fw = round(Kd_cont · 2^8)
  - Ki_fw = round(Ki_cont · Ts · 2^8), com Ki_cont = Kp_cont / Ti e Ti = 5·tau

Assume Ts = 1 ms (TIM7 ~ 1 kHz) e SHIFT = 8 (K_SCALE = 256).

Saída: uma linha por perfil contendo contínuo (Kp, Ki_est, Kd, tau, Ti)
e inteiros de firmware (kp_fw, ki_fw, kd_fw).
"""
from __future__ import annotations

from pathlib import Path
import sys

# Garantir que o diretório raiz do repositório esteja no sys.path, mesmo
# quando o script for chamado como arquivo (ex.: "python tools/...py").
_ROOT = Path(__file__).resolve().parents[1]
if str(_ROOT) not in sys.path:
    sys.path.insert(0, str(_ROOT))

from tuning_profiles import AnalysisCatalog, K_SCALE

TS = 0.001  # s


def _axis_letter(axis: int) -> str:
    return {0: "X", 1: "Y", 2: "Z"}.get(axis, str(axis))


def main() -> None:
    cat = AnalysisCatalog()

    # Coleta e ordena perfis por (axis, microstep)
    entries = []
    for key, prof in cat._profiles.items():  # type: ignore[attr-defined]
        entries.append((key.axis, key.microstep, prof))
    entries.sort(key=lambda x: (x[0], x[1]))

    print("PID derivation (Ts=1 ms, SHIFT=8, Ti=5·tau)")
    print("axis@µstep  Kp      Ki_est   Kd      tau(ms)   Ti(ms)   kp_fw   ki_fw   kd_fw")
    for axis, ms, p in entries:
        # Contínuo
        kp_cont = float(p.kp)
        kd_cont = float(p.kd)
        tau_s = float(p.tau_s)

        Ti_s = 5.0 * tau_s if tau_s > 0.0 else 0.0
        ki_cont = (kp_cont / Ti_s) if Ti_s > 0.0 else 0.0

        # Firmware (inteiros)
        kp_fw = int(round(kp_cont * K_SCALE))
        kd_fw = int(round(kd_cont * K_SCALE))
        ki_fw = int(round(ki_cont * TS * K_SCALE))

        print(
            f"{_axis_letter(axis)}@1/{ms:<4}  "
            f"{kp_cont:6.4f}  {ki_cont:7.4f}  {kd_cont:6.4f}  "
            f"{tau_s*1e3:7.3f}  {Ti_s*1e3:7.3f}  "
            f"{kp_fw:6d}  {ki_fw:6d}  {kd_fw:6d}"
        )

    print("\nObservação: use os inteiros (kp_fw, ki_fw, kd_fw) no firmware/JSON.")


if __name__ == "__main__":
    main()
