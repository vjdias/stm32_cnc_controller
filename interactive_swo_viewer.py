#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualizador interativo dos encoders usando dados CSV do SWO/SWV.

Objetivo:
  - Interface parecida com o interactive_sim (círculos + 3 gráficos),
    mas SEM caixas de texto: apenas botões (Play / Pause / Reset).
  - Os dados não vêm da simulação; vêm de um stream CSV:
        axis,time_ms,enc_pos,step_count
    ou variações compatíveis (3, 4 ou 5 colunas numéricas).

Fontes suportadas:
  - Porta serial (--port COM3, --baud 115200)
  - Arquivo texto/CSV sendo atualizado (--file ...)
  - stdin (ex: pipe do SWO/SWV) (--stdin)
"""

from __future__ import annotations

import argparse
import os
import sys
import threading
import time
from collections import deque
from math import pi
from pathlib import Path
from queue import Queue, Empty, Full
from typing import Deque, Dict, List, Optional, Sequence, Tuple

import numpy as np

import matplotlib
import csv
from datetime import datetime

# Respeita execução headless via env (evita Qt mensagens de QStandardPaths)
_ENV_MPL = os.environ.get("MPLBACKEND", "").lower()
_ENV_HEADLESS = os.environ.get("SIM_HEADLESS", "0") in ("1", "true", "yes")
if _ENV_MPL == "agg" or _ENV_HEADLESS:
    matplotlib.use("Agg")
else:
    try:
        matplotlib.use("QtAgg")  # ou "TkAgg"
    except ImportError:
        try:
            matplotlib.use("TkAgg")
        except ImportError:
            matplotlib.use("Agg")

import matplotlib.gridspec as gridspec
import matplotlib.pyplot as plt
from matplotlib.widgets import Button


def _open_serial(port: str, baud: int):
    """Abre porta serial (opcional, depende de pyserial)."""
    try:
        import serial  # type: ignore
    except Exception:  # pragma: no cover - dependência externa
        print("pyserial é necessário para --port: pip install pyserial", file=sys.stderr)
        raise SystemExit(2)
    return serial.Serial(port=port, baudrate=baud, timeout=0.1)


def _is_debug_line(s: str) -> bool:
    s = s.lstrip()
    if not s:
        return True
    if s.startswith("[") or s.startswith("LOG:") or s.startswith("OG:") or s.startswith("L:svc="):
        return True
    return False


def _parse_csv_line(s: str) -> Optional[Tuple[int, int, int, int, int]]:
    """
    Aceita formatos:
      - axis,time_ms,enc_pos,step_count
      - axis,id,time_ms,enc_pos,step_count
      - time_ms,enc_pos,step_count   (axis = 0)
    Retorna (axis, time_ms, enc_pos, step_count) ou None.
    """
    s = s.strip()
    if not s or _is_debug_line(s):
        return None
    parts = [p.strip() for p in s.split(",")]
    try:
        if len(parts) == 5:
            # axis,id,time_ms,enc_rel,steps
            axis = int(parts[0], 10)
            seq = int(parts[1], 10)
            t_ms = int(parts[2], 10)
            enc = int(parts[3], 10)
            steps = int(parts[4], 10)
        elif len(parts) == 4:
            # axis,time_ms,enc_rel,steps (id ausente; usamos -1 como placeholder)
            axis = int(parts[0], 10)
            seq = -1
            t_ms = int(parts[1], 10)
            enc = int(parts[2], 10)
            steps = int(parts[3], 10)
        else:
            # Formatos antigos (3 colunas) ou linhas que não são CSV são ignorados
            return None
    except ValueError:
        return None
    return axis, seq, t_ms, enc, steps


class LiveEncoderViewer:
    """Interface interativa tipo interactive_sim, alimentada por stream CSV."""

    def __init__(
        self,
        *,
        source_mode: str,
        port: Optional[str],
        baud: int,
        file_path: Optional[Path],
        offsets_xyz: Sequence[int],
        steps_per_rev_base: int,
        microsteps: int,
        initial_window_s: float = 5.0,
        avg_n: int = 1,
        log_dir: Optional[Path] = None,
    ) -> None:
        if _ENV_HEADLESS:
            raise RuntimeError("SIM_HEADLESS=1: este viewer requer backend interativo.")

        if source_mode not in ("serial", "file", "stdin"):
            raise ValueError(f"source_mode inválido: {source_mode}")

        self.source_mode = source_mode
        self.port = port
        self.baud = baud
        self.file_path = file_path
        self.offsets_xyz = list(offsets_xyz)
        if len(self.offsets_xyz) != 3:
            raise ValueError("offsets_xyz deve ter 3 inteiros (X,Y,Z).")

        self.steps_per_rev = int(steps_per_rev_base) * int(microsteps)
        if self.steps_per_rev <= 0:
            self.steps_per_rev = 102400

        # Janela de média (N amostras -> 1 ponto)
        self.avg_n = max(1, int(avg_n))
        self._batch_count: int = 0
        self._batch_sum_t: float = 0.0
        self._batch_sum_pos: np.ndarray = np.zeros(3, dtype=float)
        self._batch_sum_vel: np.ndarray = np.zeros(3, dtype=float)
        self._batch_sum_err: np.ndarray = np.zeros(3, dtype=float)

        # Estado por eixo
        self.base_pos: List[Optional[float]] = [None, None, None]
        self.base_steps: List[Optional[int]] = [None, None, None]
        self.last_t: List[Optional[float]] = [None, None, None]
        self.pos: List[float] = [0.0, 0.0, 0.0]
        self.steps: List[float] = [0.0, 0.0, 0.0]
        self.vel: List[float] = [0.0, 0.0, 0.0]
        self.err: List[float] = [0.0, 0.0, 0.0]
        self.err_accum: np.ndarray = np.zeros(3, dtype=float)

        self.t: float = 0.0

        # Histórico usado nos gráficos (janela deslizante)
        self.max_points = 5000
        self.history: Dict[str, Deque[float]] = {
            "t_s": deque(maxlen=self.max_points),
            "x_pos": deque(maxlen=self.max_points),
            "y_pos": deque(maxlen=self.max_points),
            "z_pos": deque(maxlen=self.max_points),
            "x_v": deque(maxlen=self.max_points),
            "y_v": deque(maxlen=self.max_points),
            "z_v": deque(maxlen=self.max_points),
            "x_err": deque(maxlen=self.max_points),
            "y_err": deque(maxlen=self.max_points),
            "z_err": deque(maxlen=self.max_points),
        }

        # Buffer de mensagens brutas do SWO/SWV (para exibição na lateral)
        self.log_lines: Deque[str] = deque(maxlen=80)
        self._log_lock = threading.Lock()

        # Parâmetros de janela horizontal
        self.graph_time_pad_left = max(0.1, 0.05 * float(initial_window_s))
        self.graph_time_pad_right = max(0.2, 0.1 * float(initial_window_s))

        # Controle de execução
        self.running = False
        self._timer_running = False

        # Fila de amostras vindas do thread leitor:
        # (axis, seq, t_ms, enc_rel, steps)
        self._samples: "Queue[Tuple[int, int, int, int, int]]" = Queue(maxsize=10000)
        self._stop_event = threading.Event()
        self._running_event = threading.Event()

        # Log CSV sanitizado (sempre que houver dados válidos)
        self.log_dir = Path(log_dir) if log_dir is not None else Path("sim_logs")
        try:
            self.log_dir.mkdir(parents=True, exist_ok=True)
        except Exception:
            # Se não for possível criar o diretório, desabilita log em disco.
            self.log_dir = None
        self._csv_fh: Optional[object] = None
        self._csv_writer: Optional[csv.writer] = None

        # Figura e layout (similar ao interactive_sim, mas mais compacto)
        self.fig = plt.figure(figsize=(14, 8), dpi=96)
        try:
            self.fig.canvas.manager.set_window_title("SWO Encoder Viewer")
        except Exception:
            try:
                self.fig.canvas.set_window_title("SWO Encoder Viewer")
            except Exception:
                pass

        gs = gridspec.GridSpec(4, 4, figure=self.fig)
        self.fig.subplots_adjust(
            left=0.06,
            right=0.98,
            top=0.94,
            bottom=0.06,
            hspace=0.4,
            wspace=0.3,
        )

        # Linha 0: círculos (X,Y,Z)
        self.ax_motor_x = self.fig.add_subplot(
            gs[0, 0], aspect="equal", xlim=(-1.2, 1.2), ylim=(-1.2, 1.2)
        )
        self.ax_motor_y = self.fig.add_subplot(
            gs[0, 1], aspect="equal", xlim=(-1.2, 1.2), ylim=(-1.2, 1.2)
        )
        self.ax_motor_z = self.fig.add_subplot(
            gs[0, 2], aspect="equal", xlim=(-1.2, 1.2), ylim=(-1.2, 1.2)
        )

        # Painel de controle (botões) - compacto no topo direito
        self.ax_control_panel = self.fig.add_subplot(gs[0, 3])

        # Gráficos (linhas 1-3, colunas 0-2)
        gs_graphs = gridspec.GridSpecFromSubplotSpec(
            3,
            1,
            subplot_spec=gs[1:4, 0:3],
            hspace=0.25,
        )
        self.ax_graph_pos = self.fig.add_subplot(gs_graphs[0, 0])
        self.ax_graph_vel = self.fig.add_subplot(gs_graphs[1, 0])
        self.ax_graph_err = self.fig.add_subplot(gs_graphs[2, 0])

        # Painel lateral (erros acumulados + log SWO)
        self.ax_friction_panel = self.fig.add_subplot(gs[1:4, 3])

        # Inicializa artistas e widgets
        self._init_artists()
        self._init_widgets()

        # Timer de atualização
        self.animation_interval_ms = 50
        self._timer = self.fig.canvas.new_timer(interval=self.animation_interval_ms)
        self._timer.add_callback(self._on_timer_tick)

        # Thread leitor
        self._reader_thread = threading.Thread(
            target=self._reader_loop, name="SWOReader", daemon=True
        )
        self._reader_thread.start()

        # Encerra thread ao fechar a janela
        self.fig.canvas.mpl_connect("close_event", self._on_close)

        print("SWO Encoder Viewer pronto. Use Play/Pause/Reset.")

    def _log_numeric_sample(
        self,
        axis: int,
        seq: int,
        t_ms: int,
        enc_rel: float,
        steps: float,
        err: float,
        err_accum: float,
    ) -> None:
        """Grava uma linha sanitizada em CSV sempre que houver dados válidos.

        Colunas:
          axis,id,time_ms,enc_rel,steps,err_steps,err_abs,err_accum_steps_s
        """
        if self.log_dir is None:
            return
        if self._csv_fh is None:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = self.log_dir / f"swo_viewer_{ts}.csv"
            try:
                self._csv_fh = path.open("w", newline="", encoding="utf-8")
            except Exception:
                self._csv_fh = None
                return
            self._csv_writer = csv.writer(self._csv_fh)
            self._csv_writer.writerow(
                [
                    "axis",
                    "id",
                    "time_ms",
                    "enc_rel",
                    "steps",
                    "err_steps",
                    "err_abs",
                    "err_accum_steps_s",
                ]
            )
        if self._csv_writer is None:
            return
        try:
            self._csv_writer.writerow(
                [
                    int(axis),
                    int(seq),
                    int(t_ms),
                    float(enc_rel),
                    float(steps),
                    float(err),
                    float(abs(err)),
                    float(err_accum),
                ]
            )
            self._csv_fh.flush()
        except Exception:
            pass

    # ------------------------------------------------------------------ GUI
    def _init_artists(self) -> None:
        # Círculos
        self.motor_lines: List[plt.Line2D] = []
        self.motor_texts: List[plt.Text] = []

        for ax, color, name in zip(
            [self.ax_motor_x, self.ax_motor_y, self.ax_motor_z],
            ["r", "g", "b"],
            ["X", "Y", "Z"],
        ):
            ax.clear()
            ax.set_title(f"Eixo {name}", fontsize=9)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_xlim(-1.2, 1.2)
            ax.set_ylim(-1.2, 1.2)
            circle = plt.Circle((0, 0), 1.0, color="gray", fill=False, linestyle="--")
            ax.add_artist(circle)
            line, = ax.plot(
                [],
                [],
                f"{color}-o",
                lw=1.25,
                markersize=8,
                antialiased=False,
            )
            self.motor_lines.append(line)
            txt = ax.text(0, -1.6, "0.0v", ha="center", fontsize=8)
            self.motor_texts.append(txt)

        # Gráfico de posição
        self.ax_graph_pos.clear()
        self.ax_graph_pos.set_ylabel("Posição (steps relativos)")
        self.ax_graph_pos.grid(True)
        self.ax_graph_pos.set_xlim(0.0, self.graph_time_pad_right)
        self.line_pos_x, = self.ax_graph_pos.plot([], [], "r-", label="X pos", lw=1.25)
        self.line_pos_y, = self.ax_graph_pos.plot([], [], "g-", label="Y pos", lw=1.25)
        self.line_pos_z, = self.ax_graph_pos.plot([], [], "b-", label="Z pos", lw=1.25)
        self.ax_graph_pos.legend(loc="upper left")

        # Gráfico de velocidade
        self.ax_graph_vel.clear()
        self.ax_graph_vel.set_ylabel("Velocidade (steps/s)")
        self.ax_graph_vel.grid(True)
        self.ax_graph_vel.set_xlim(0.0, self.graph_time_pad_right)
        self.line_vel_x, = self.ax_graph_vel.plot([], [], "r-", label="X vel", lw=1.25)
        self.line_vel_y, = self.ax_graph_vel.plot([], [], "g-", label="Y vel", lw=1.25)
        self.line_vel_z, = self.ax_graph_vel.plot([], [], "b-", label="Z vel", lw=1.25)
        self.ax_graph_vel.legend(loc="upper left")

        # Gráfico de erro
        self.ax_graph_err.clear()
        self.ax_graph_err.set_ylabel("Erro (steps)")
        self.ax_graph_err.set_xlabel("Tempo (s)")
        self.ax_graph_err.grid(True)
        self.ax_graph_err.set_xlim(0.0, self.graph_time_pad_right)
        self.line_err_x, = self.ax_graph_err.plot([], [], "r-", label="Erro X", lw=1.25)
        self.line_err_y, = self.ax_graph_err.plot([], [], "g-", label="Erro Y", lw=1.25)
        self.line_err_z, = self.ax_graph_err.plot([], [], "b-", label="Erro Z", lw=1.25)
        self.ax_graph_err.legend(loc="upper left")

        # Painel lateral: textos de erro acumulado + caixa de log
        self.ax_friction_panel.clear()
        # Caixa cinza com borda simples
        self.ax_friction_panel.set_facecolor("0.92")
        self.ax_friction_panel.set_xticks([])
        self.ax_friction_panel.set_yticks([])
        for spine in self.ax_friction_panel.spines.values():
            spine.set_visible(True)
        self.text_timer_x = self.ax_friction_panel.text(
            0.02,
            0.98,
            "Erro acum X: 0.00 steps·s",
            ha="left",
            va="top",
            fontsize=8,
        )
        self.text_timer_y = self.ax_friction_panel.text(
            0.02,
            0.92,
            "Erro acum Y: 0.00 steps·s",
            ha="left",
            va="top",
            fontsize=8,
        )
        self.text_timer_z = self.ax_friction_panel.text(
            0.02,
            0.86,
            "Erro acum Z: 0.00 steps·s",
            ha="left",
            va="top",
            fontsize=8,
        )
        # Título da área de log
        self.ax_friction_panel.text(
            0.02,
            0.78,
            "SWO log:",
            ha="left",
            va="top",
            fontsize=8,
            fontweight="bold",
        )
        # Caixa de texto para últimas linhas do SWO
        self.text_log = self.ax_friction_panel.text(
            0.02,
            0.76,
            "",
            ha="left",
            va="top",
            fontsize=7,
            family="monospace",
        )

    def _init_widgets(self) -> None:
        self.ax_control_panel.axis("off")
        # Três botões em formato de lista vertical, menores
        gs_control = gridspec.GridSpecFromSubplotSpec(
            3,
            1,
            subplot_spec=self.ax_control_panel.get_subplotspec(),
            hspace=0.1,
        )

        # Linha 0: Play
        ax_play = self.fig.add_subplot(gs_control[0, 0])
        self.btn_play = Button(ax_play, "Play", color="lightgreen")
        self.btn_play.on_clicked(self.on_play)

        # Linha 1: Pause
        ax_pause = self.fig.add_subplot(gs_control[1, 0])
        self.btn_pause = Button(ax_pause, "Pause", color="lightyellow")
        self.btn_pause.on_clicked(self.on_pause)

        # Linha 2: Reset
        ax_reset = self.fig.add_subplot(gs_control[2, 0])
        self.btn_reset = Button(ax_reset, "Reset", color="lightcoral")
        self.btn_reset.on_clicked(self.on_reset)

    # ----------------------------------------------------------------- leitura
    def _reader_loop(self) -> None:
        """Thread que lê da fonte (serial/arquivo/stdin) e joga na fila."""
        try:
            if self.source_mode == "serial":
                if not self.port:
                    print("Porta serial não informada.", file=sys.stderr)
                    return
                ser = _open_serial(self.port, self.baud)
                try:
                    buf = bytearray()
                    while not self._stop_event.is_set():
                        if not self._running_event.is_set():
                            time.sleep(0.05)
                            continue
                        b = ser.read(1)
                        if not b:
                            continue
                        if b in (b"\r", b"\n"):
                            if buf:
                                try:
                                    line = buf.decode("utf-8", errors="ignore")
                                finally:
                                    buf.clear()
                                self._enqueue_line(line)
                        else:
                            buf += b
                finally:
                    try:
                        ser.close()
                    except Exception:
                        pass
            elif self.source_mode == "file":
                if not self.file_path:
                    print("Arquivo não informado.", file=sys.stderr)
                    return
                # Tail básico: lê até EOF, espera, continua.
                with self.file_path.open("r", encoding="utf-8", errors="ignore") as fh:
                    while not self._stop_event.is_set():
                        if not self._running_event.is_set():
                            time.sleep(0.05)
                            continue
                        line = fh.readline()
                        if not line:
                            time.sleep(0.05)
                            continue
                        self._enqueue_line(line)
            else:  # stdin
                for line in sys.stdin:
                    if self._stop_event.is_set():
                        break
                    if not self._running_event.is_set():
                        time.sleep(0.05)
                        continue
                    self._enqueue_line(line)
        except Exception as exc:
            print(f"[reader] erro: {exc}", file=sys.stderr)

    def _enqueue_line(self, line: str) -> None:
        # Registra sempre a linha original no buffer de log
        clean = line.rstrip("\r\n")
        if clean:
            with self._log_lock:
                self.log_lines.append(clean)

        # Só envia para a fila de amostras se bater com o formato CSV esperado
        sample = _parse_csv_line(clean)
        if sample is None:
            return
        axis, seq, t_ms, enc, steps = sample

        try:
            self._samples.put_nowait((axis, seq, t_ms, enc, steps))
        except Full:
            # Descarta o mais antigo para não travar
            try:
                _ = self._samples.get_nowait()
            except Empty:
                pass
            try:
                self._samples.put_nowait((axis, seq, t_ms, enc, steps))
            except Full:
                pass

    # -------------------------------------------------------- callbacks GUI
    def on_play(self, _event) -> None:
        if self.running:
            return
        self.running = True
        self._running_event.set()
        if not self._timer_running:
            self._timer.start()
            self._timer_running = True
        print("Play.")

    def on_pause(self, _event) -> None:
        if not self.running:
            return
        self.running = False
        self._running_event.clear()
        print("Pause.")

    def on_reset(self, _event) -> None:
        self.running = False
        self._running_event.clear()
        self._reset_state()
        self._redraw_all()
        print("Reset.")

    def _on_close(self, _event) -> None:
        self._stop_event.set()
        self._running_event.set()

    # ---------------------------------------------------------- núcleo viewer
    def _reset_state(self) -> None:
        self.base_pos = [None, None, None]
        self.base_steps = [None, None, None]
        self.last_t = [None, None, None]
        self.pos = [0.0, 0.0, 0.0]
        self.steps = [0.0, 0.0, 0.0]
        self.vel = [0.0, 0.0, 0.0]
        self.err = [0.0, 0.0, 0.0]
        self.err_accum[:] = 0.0
        self.t = 0.0
        for key in self.history:
            self.history[key].clear()
        # Limpa acumuladores de média
        self._batch_count = 0
        self._batch_sum_t = 0.0
        self._batch_sum_pos[:] = 0.0
        self._batch_sum_vel[:] = 0.0
        self._batch_sum_err[:] = 0.0

    def _on_timer_tick(self) -> None:
        if not self.running:
            return
        updated = False
        while True:
            try:
                axis, seq, t_ms, enc, steps = self._samples.get_nowait()
            except Empty:
                break
            self._consume_sample(axis, seq, t_ms, enc, steps)
            updated = True
        if updated:
            self._update_artists()
            self.fig.canvas.draw_idle()

    def _axis_index(self, axis_id: int) -> int:
        if axis_id < 0:
            return 0
        return int(axis_id) % 3

    def _consume_sample(
        self, axis_id: int, seq: int, t_ms: int, enc_raw: int, steps_raw: int
    ) -> None:
        axis = self._axis_index(axis_id)
        t_s = float(t_ms) / 1000.0
        enc_abs = float(abs(enc_raw))
        offset = float(self.offsets_xyz[axis])
        pos_rel = enc_abs - offset

        if self.base_pos[axis] is None:
            self.base_pos[axis] = pos_rel
            self.base_steps[axis] = int(steps_raw)
            self.last_t[axis] = t_s
            self.pos[axis] = 0.0
            self.steps[axis] = 0.0
            self.vel[axis] = 0.0
            self.err[axis] = 0.0
        else:
            base_pos = (
                self.base_pos[axis] if self.base_pos[axis] is not None else pos_rel
            )
            base_steps = (
                self.base_steps[axis]
                if self.base_steps[axis] is not None
                else steps_raw
            )
            pos = pos_rel - float(base_pos)
            steps_rel = float(steps_raw - int(base_steps))
            last_t = self.last_t[axis] if self.last_t[axis] is not None else t_s
            dt = max(t_s - last_t, 1e-6)
            vel = (steps_rel - self.steps[axis]) / dt
            err = steps_rel - pos
            self.last_t[axis] = t_s
            self.pos[axis] = pos
            self.steps[axis] = steps_rel
            self.vel[axis] = vel
            self.err[axis] = err
            self.err_accum[axis] += abs(err) * dt

        # Atualiza snapshot global para este instante
        self.t = t_s
        pos_vec = np.array(self.pos, dtype=float)
        vel_vec = np.array(self.vel, dtype=float)
        err_vec = np.array(self.err, dtype=float)

        # Envia ponto (com média opcional)
        self._accumulate_and_maybe_append(t_s, pos_vec, vel_vec, err_vec)

        # Loga amostra com erro calculado para este eixo
        try:
            self._log_numeric_sample(
                axis=axis,
                seq=seq,
                t_ms=t_ms,
                enc_rel=pos_rel,
                steps=self.steps[axis],
                err=self.err[axis],
                err_accum=self.err_accum[axis],
            )
        except Exception:
            # Erros de log não devem quebrar a visualização
            pass

    def _accumulate_and_maybe_append(
        self,
        t_s: float,
        pos_vec: np.ndarray,
        vel_vec: np.ndarray,
        err_vec: np.ndarray,
    ) -> None:
        """Aplica média móvel simples sobre N amostras antes de plotar."""
        if self.avg_n <= 1:
            self._append_history(t_s, pos_vec, vel_vec, err_vec)
            return

        self._batch_sum_t += t_s
        self._batch_sum_pos += pos_vec
        self._batch_sum_vel += vel_vec
        self._batch_sum_err += err_vec
        self._batch_count += 1

        if self._batch_count >= self.avg_n:
            n = float(self._batch_count)
            mean_t = self._batch_sum_t / n
            mean_pos = self._batch_sum_pos / n
            mean_vel = self._batch_sum_vel / n
            mean_err = self._batch_sum_err / n
            self._append_history(mean_t, mean_pos, mean_vel, mean_err)

            # Reseta acumuladores
            self._batch_count = 0
            self._batch_sum_t = 0.0
            self._batch_sum_pos[:] = 0.0
            self._batch_sum_vel[:] = 0.0
            self._batch_sum_err[:] = 0.0

    def _append_history(
        self,
        t_s: float,
        pos_rel_steps: np.ndarray,
        vel_sps: np.ndarray,
        err_steps: np.ndarray,
    ) -> None:
        h = self.history
        h["t_s"].append(t_s)
        h["x_pos"].append(float(pos_rel_steps[0]))
        h["y_pos"].append(float(pos_rel_steps[1]))
        h["z_pos"].append(float(pos_rel_steps[2]))
        h["x_v"].append(float(vel_sps[0]))
        h["y_v"].append(float(vel_sps[1]))
        h["z_v"].append(float(vel_sps[2]))
        h["x_err"].append(float(err_steps[0]))
        h["y_err"].append(float(err_steps[1]))
        h["z_err"].append(float(err_steps[2]))

    def _update_artists(self) -> None:
        t_data = self.history["t_s"]
        if not t_data:
            return

        # Atualiza linhas
        self.line_pos_x.set_data(t_data, self.history["x_pos"])
        self.line_pos_y.set_data(t_data, self.history["y_pos"])
        self.line_pos_z.set_data(t_data, self.history["z_pos"])

        self.line_vel_x.set_data(t_data, self.history["x_v"])
        self.line_vel_y.set_data(t_data, self.history["y_v"])
        self.line_vel_z.set_data(t_data, self.history["z_v"])

        self.line_err_x.set_data(t_data, self.history["x_err"])
        self.line_err_y.set_data(t_data, self.history["y_err"])
        self.line_err_z.set_data(t_data, self.history["z_err"])

        # Atualiza círculos (usando steps relativos como "posição de motor")
        for i in range(3):
            steps_rel = self.steps[i]
            angle = (steps_rel % self.steps_per_rev) / float(self.steps_per_rev) * 2.0 * pi
            x_coord = np.cos(angle)
            y_coord = np.sin(angle)
            self.motor_lines[i].set_data([0.0, x_coord], [0.0, y_coord])
            revs = steps_rel / float(self.steps_per_rev)
            self.motor_texts[i].set_text(f"{revs:.2f}v")

        # Atualiza limites de tempo (X)
        t_min = t_data[0]
        t_max = t_data[-1]
        span = max(1e-3, t_max - t_min)
        left = t_min - self.graph_time_pad_left
        right = t_max + max(self.graph_time_pad_right, 0.02 * span)
        for ax in (self.ax_graph_pos, self.ax_graph_vel, self.ax_graph_err):
            ax.set_xlim(left, right)

        # Atualiza limites de Y com margem
        def autoscale_y(values: Sequence[float], ax) -> None:
            if not values:
                return
            vmin = min(values)
            vmax = max(values)
            if vmin == vmax:
                vmin -= 0.5
                vmax += 0.5
            span_v = vmax - vmin
            margin = max(1.0, 0.1 * span_v)
            ax.set_ylim(vmin - margin, vmax + margin)

        pos_all: List[float] = (
            list(self.history["x_pos"])
            + list(self.history["y_pos"])
            + list(self.history["z_pos"])
        )
        vel_all: List[float] = (
            list(self.history["x_v"])
            + list(self.history["y_v"])
            + list(self.history["z_v"])
        )
        err_all: List[float] = (
            list(self.history["x_err"])
            + list(self.history["y_err"])
            + list(self.history["z_err"])
        )
        autoscale_y(pos_all, self.ax_graph_pos)
        autoscale_y(vel_all, self.ax_graph_vel)
        autoscale_y(err_all, self.ax_graph_err)

        # Textos de erro acumulado
        labels = ["X", "Y", "Z"]
        for i, txt in enumerate(
            [self.text_timer_x, self.text_timer_y, self.text_timer_z]
        ):
            txt.set_text(
                f"Erro acum {labels[i]}: {self.err_accum[i]:.2f} steps·s"
            )

        # Atualiza caixa de log com as últimas linhas
        with self._log_lock:
            last_lines = list(self.log_lines)[-12:]
        if last_lines:
            self.text_log.set_text("\n".join(last_lines))
        else:
            self.text_log.set_text("")

    def _redraw_all(self) -> None:
        # Zera dados nos artistas
        self.line_pos_x.set_data([], [])
        self.line_pos_y.set_data([], [])
        self.line_pos_z.set_data([], [])
        self.line_vel_x.set_data([], [])
        self.line_vel_y.set_data([], [])
        self.line_vel_z.set_data([], [])
        self.line_err_x.set_data([], [])
        self.line_err_y.set_data([], [])
        self.line_err_z.set_data([], [])
        for i in range(3):
            self.motor_lines[i].set_data([], [])
            self.motor_texts[i].set_text("0.0v")
        for txt, eixo in zip(
            (self.text_timer_x, self.text_timer_y, self.text_timer_z),
            ("X", "Y", "Z"),
        ):
            txt.set_text(f"Erro acum {eixo}: 0.00 steps·s")
        self.text_log.set_text("")
        self.fig.canvas.draw_idle()

    # -------------------------------------------------------------- interface
    def run(self) -> None:
        plt.show()
        # Ao sair do show(), garantir encerramento
        self._stop_event.set()
        self._running_event.set()
        # Fecha arquivo de log, se aberto
        if self._csv_fh is not None:
            try:
                self._csv_fh.close()
            except Exception:
                pass


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Visualizador interativo (tipo interactive_sim) alimentado por SWO/SWV (CSV)."
    )
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument(
        "--port",
        "-p",
        help="Porta serial (ex.: COM3, /dev/ttyUSB0) com CSV axis,time,enc,steps.",
    )
    src.add_argument(
        "--file",
        "-f",
        type=Path,
        help="Arquivo texto/CSV para 'tail' (SWV export ou encoder_csv).",
    )
    src.add_argument(
        "--stdin",
        action="store_true",
        help="Ler dados de stdin (ex.: pipe do SWO/SWV).",
    )
    p.add_argument(
        "--baud",
        "-b",
        type=int,
        default=115200,
        help="Baudrate da porta serial (default: 115200).",
    )
    p.add_argument(
        "--offsets",
        nargs=3,
        type=int,
        default=[40017, 5005, 40039],
        metavar=("X", "Y", "Z"),
        help="Offsets a subtrair de |enc_pos| para X,Y,Z (default: 40017 5005 40039).",
    )
    p.add_argument(
        "--steps-per-rev-base",
        type=int,
        default=400,
        help="Passos base por volta do motor (ex.: 400).",
    )
    p.add_argument(
        "--microsteps",
        type=int,
        default=256,
        help="Microsteps por passo base (default: 256).",
    )
    p.add_argument(
        "--window",
        type=float,
        default=5.0,
        help="Janela inicial de tempo em segundos (default: 5.0).",
    )
    p.add_argument(
        "--avg",
        type=int,
        default=1,
        help=(
            "Número de amostras para média antes de plotar "
            "(1 = sem média, valor direto)."
        ),
    )
    p.add_argument(
        "--log-dir",
        type=Path,
        default=Path("sim_logs"),
        help="Diretório para salvar o CSV sanitizado capturado do SWO (default: sim_logs).",
    )
    return p.parse_args()


def main() -> None:
    args = parse_args()
    if args.stdin:
        source_mode = "stdin"
    elif args.port:
        source_mode = "serial"
    else:
        source_mode = "file"

    viewer = LiveEncoderViewer(
        source_mode=source_mode,
        port=args.port,
        baud=args.baud,
        file_path=args.file,
        offsets_xyz=args.offsets,
        steps_per_rev_base=args.steps_per_rev_base,
        microsteps=args.microsteps,
        initial_window_s=args.window,
        avg_n=args.avg,
        log_dir=args.log_dir,
    )
    viewer.run()


if __name__ == "__main__":
    main()
