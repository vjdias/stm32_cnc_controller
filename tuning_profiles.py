#!/usr/bin/env python3
"""
Perfis de ganho derivados das análises SWV.

Esta classe encapsula os valores calculados em analysis_summary.txt e
oferece helpers para convertê-los nos formatos usados pelo firmware
(k_scale = 256) ou diretamente pelo simulador InteractiveSim.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple, Optional

K_SCALE = 256  # MOTION_PI_SHIFT (>> 8)


@dataclass(frozen=True)
class AxisKey:
    axis: int
    microstep: int

    def label(self) -> str:
        letters = {0: "X", 1: "Y", 2: "Z"}
        return f"{letters.get(self.axis, self.axis)}@1/{self.microstep}"


@dataclass(frozen=True)
class TuningProfile:
    axis: int
    microstep: int
    kp: float
    ki: float
    kd: float
    l_s: float
    tau_s: float
    steady_cmd: float
    steady_enc: float

    def firmware_gains(self) -> Tuple[int, int, int]:
        """Converte Kp/Ki/Kd contínuos para o formato inteiro do firmware."""
        return (
            int(round(self.kp * K_SCALE)),
            int(round(self.ki * K_SCALE)),
            int(round(self.kd * K_SCALE)),
        )

    def describe(self) -> str:
        kp_i, ki_i, kd_i = self.firmware_gains()
        return (
            f"{AxisKey(self.axis, self.microstep).label()}: "
            f"Kp={self.kp:.4f}/{kp_i}, Ki={self.ki:.4f}/{ki_i}, "
            f"Kd={self.kd:.4f}/{kd_i}, L={self.l_s*1e3:.1f} ms, τ={self.tau_s*1e3:.1f} ms"
        )


class AnalysisCatalog:
    """Catálogo imutável dos perfis encontrados na análise."""

    def __init__(self):
        self._profiles: Dict[AxisKey, TuningProfile] = {}
        self._load_defaults()

    def _load_defaults(self) -> None:
        data = [
            # axis, microstep, kp, ki, kd, L, tau, steady_cmd, steady_enc
            (0, 16, 3.6942, 0.0, 0.0018, 0.001, 0.019, 6852.10, 42289.97),
            (0, 256, 3.1300, 0.0, 0.0016, 0.001, 0.001, 6895.48, 2643.62),
            (0, 4, 0.1941, 0.0, 0.0004, 0.004, 0.016, 6692.09, 165494.11),
            (1, 16, 29.1803, 0.0, 0.0146, 0.001, 0.019, 6854.29, 5355.59),
            (1, 256, 24.5333, 0.0, 0.0123, 0.001, 0.001, 6897.20, 337.36),
            (1, 4, 0.9213, 0.0, 0.0023, 0.005, 0.012, 6659.10, 20816.11),
            (2, 16, 3.7886, 0.0, 0.0019, 0.001, 0.019, 6857.33, 41268.06),
            (2, 256, 3.1142, 0.0, 0.0016, 0.001, 0.001, 6894.10, 2656.51),
            (2, 4, 0.2612, 0.0, 0.0004, 0.003, 0.017, 6775.07, 176394.14),
        ]
        for axis, ms, kp, ki, kd, L, tau, vcmd, venc in data:
            key = AxisKey(axis, ms)
            self._profiles[key] = TuningProfile(axis, ms, kp, ki, kd, L, tau, vcmd, venc)

    def get(self, axis: int, microstep: int) -> Optional[TuningProfile]:
        return self._profiles.get(AxisKey(axis, microstep))

    def for_simulator(self) -> Dict[str, Tuple[int, int, int]]:
        """Retorna dicionário label -> (kp_i, ki_i, kd_i) para uso direto no simulador."""
        return {key.label(): prof.firmware_gains() for key, prof in self._profiles.items()}

    def dump(self) -> str:
        lines = ["Perfis disponíveis:"]
        for key in sorted(self._profiles.keys(), key=lambda k: (k.axis, k.microstep)):
            lines.append(self._profiles[key].describe())
        return "\n".join(lines)


if __name__ == "__main__":
    catalog = AnalysisCatalog()
    print(catalog.dump())
