#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simulador Interativo: CASC (Cross-Axis Sync) + PID + DDA @ 50 kHz
------------------------------------------------------------------
Versão 13.5: Adicionada função "Reset + Play"

- CORREÇÃO (BUG): "Lag" ao trocar o foco (clicar) entre os TextBoxes,
  mesmo com a simulação pausada.
- CAUSA: Todos os TextBoxes (mesmo os de cenário) estavam ligados
  a 'on_text_change', disparando redraws desnecessários no 'focus'.
- SOLUÇÃO: Os TextBoxes foram separados em 'cenário' e 'runtime'.
  - 'Cenário' (s*, v*, Kp...): Só usam 'on_submit' (que não faz nada).
    Eles só são lidos no 'Reset'.
  - 'Runtime' (Atrito): Têm seus 'callbacks' (on_text_change)
    dinamicamente LIGADOS no 'on_play' e DESLIGADOS no 'on_pause'.
    Isso impede redraws ao editar em pausa.

- [FIX V13.5] FEATURE: Clicar em "Play" enquanto a simulação já está 
  rodando agora funciona como "Reset + Play" (via helper '_restart_play').
"""
from __future__ import annotations
import argparse
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Dict, List, Tuple

import json
import math
import numpy as np
from collections import deque

import matplotlib
try:
    matplotlib.use("QtAgg") # ou "TkAgg"
except ImportError:
    print("Backend QtAgg não encontrado, usando TkAgg.")
    try:
        matplotlib.use("TkAgg")
    except ImportError:
        print("Nenhum backend interativo encontrado. Widgets podem não funcionar.")
        matplotlib.use("Agg") # Fallback para não-interativo

print("Backend ativo:", matplotlib.get_backend())

import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button, TextBox

from tuning_profiles import AnalysisCatalog, AxisKey

# =========================
# [FIX GUI] Helpers de Otimização
# =========================
from contextlib import contextmanager
import matplotlib as mpl

# Simplifica caminhos de render para gráficos mais rápidos
mpl.rcParams['path.simplify'] = True
mpl.rcParams['path.simplify_threshold'] = 0.5
# [FIX V13.4] Reduz custo de paths longos
mpl.rcParams['agg.path.chunksize'] = 20000

@contextmanager
def drawing_off():
    """Context manager para congelar o desenho durante atualizações em lote."""
    was_ion = plt.isinteractive()
    if was_ion: plt.ioff()
    try:
        yield
    finally:
        if was_ion: plt.ion()
# ============================================================


# =========================
# Configurações (Valores Padrão do Firmware)
# =========================

@dataclass
class PlantConfig:
    """Configurações de hardware/físicas da planta."""
    Ts: float = 0.001        # passo de controle (TIM7) [s] - 1kHz
    tim6_hz: float = 50_000.0  # TIM6 (DDA) [Hz] - 50kHz
    
    step_high_ticks: int = 1   # (MOTION_STEP_HIGH_TICKS)
    step_low_ticks:  int = 1   # (MOTION_STEP_LOW_TICKS)
    
    accel_sps2: float = 200_000.0 # aceleração [steps/s^2] (DEMO_ACCEL_SPS2)

    enc_cpr_xyz: Tuple[int, int, int] = (40_000, 2500, 40_000) # (ENC_COUNTS_PER_REV)
    microstep_factor: int = 256 # (MICROSTEP_FACTOR)
    motor_steps_per_rev_base: int = 400 # (STEPS_PER_REV_BASE)
    
    k_scale: int = (1 << 8) # (MOTION_PI_SHIFT)
    kd_alpha_bits: int = 8          # (alpha de filtro da derivada, 8 = >> 8)
    i_clamp: float = 200_000.0    # (MOTION_PI_I_CLAMP)
    deadband_steps: int = 10      # (MOTION_PI_DEADBAND_STEPS)

    load_C_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    load_B_xyz: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def max_sps(self) -> float:
        low = max(1, self.step_low_ticks)
        return self.tim6_hz / (self.step_high_ticks + low)

@dataclass
class Scenario:
    """Configurações do movimento (move_queue_add_req_t)."""
    s_xyz: Tuple[int, int, int] = (20000, 16000, 12000) 
    v_xyz: Tuple[int, int, int] = (10000, 8000, 6000)
    dir_xyz: Tuple[int, int, int] = (1, 1, 1)
    kp_xyz: Tuple[int, int, int] = (800, 800, 800)
    ki_xyz: Tuple[int, int, int] = (40, 40, 40)
    kd_xyz: Tuple[int, int, int] = (120, 120, 120)
    
    sim_time_s: float = 3.0 # Tempo total da simulação
    use_dda: bool = True


def parse_axis_map(text: str) -> List[Tuple[str, int]]:
    mapping: List[Tuple[str, int]] = []
    for token in text.split(","):
        token = token.strip()
        if not token:
            continue
        if ":" not in token:
            raise ValueError(f"Formato inválido para eixo: '{token}' (use X:256)")
        letter, micro = token.split(":", 1)
        letter = letter.strip().upper()
        if letter not in ("X", "Y", "Z"):
            raise ValueError(f"Eixo desconhecido: {letter}")
        mapping.append((letter, int(micro)))
    if len(mapping) != 3:
        raise ValueError("Informe exatamente três eixos (ex.: X:256,Y:256,Z:256).")
    return mapping


def gains_from_catalog(axis_map: List[Tuple[str, int]]):
    catalog = AnalysisCatalog()
    kp_list: List[int] = []
    ki_list: List[int] = []
    kd_list: List[int] = []
    axis_ids = {"X": 0, "Y": 1, "Z": 2}
    for letter, microstep in axis_map:
        axis_id = axis_ids[letter]
        profile = catalog.get(axis_id, microstep)
        if not profile:
            raise ValueError(f"Sem perfil SWV para {letter}@1/{microstep}")
        kp_i, ki_i, kd_i = profile.firmware_gains()
        kp_list.append(kp_i)
        ki_list.append(ki_i)
        kd_list.append(kd_i)
    return tuple(kp_list), tuple(ki_list), tuple(kd_list)

# =========================
# Utilitários (Conversão)
# =========================

def dda_steps_per_rev(cfg: PlantConfig) -> int:
    return cfg.motor_steps_per_rev_base * cfg.microstep_factor

# Conversão Encoder -> DDA (Espelha 'motion_conv_enc_to_dda')
def conv_enc_to_dda(cfg: PlantConfig, enc_counts: float, axis: int) -> float:
    enc_per_rev = cfg.enc_cpr_xyz[axis]
    if enc_per_rev == 0:
        return 0.0
    
    dda_rev = dda_steps_per_rev(cfg)
    num = enc_counts * dda_rev
    den = enc_per_rev
    return (num + (den // 2)) / den # Arredondamento

# =========================
# Classe DDA (Operário Burro)
# =========================
class DDAStepper:
    __slots__ = ("acc_q16","q16_one","hz")

    def __init__(self, tim6_hz: float):
        self.q16_one = float(1 << 16)
        self.hz = tim6_hz
        self.reset()
        
    def reset(self):
        """Zera o acumulador de fase."""
        self.acc_q16 = 0.0

    def emit_steps(self, v_sps: float, Ts: float) -> int:
        """Ts aqui é o Ts do *Chefe* (ex: 0.001s)"""
        substeps = int(round(self.hz * Ts)) # (ex: 50000 * 0.001 = 50)
        
        inc = (v_sps / self.hz) * self.q16_one
        
        acc = self.acc_q16
        emitted = 0
        
        for _ in range(substeps):
            acc += inc
            if acc >= self.q16_one:
                acc -= self.q16_one
                emitted += 1
        self.acc_q16 = acc
        return emitted

# =========================
# Classe de Simulação Interativa (Lógica CASC/PID REESCRITA)
# =========================

class InteractiveSim:
    def __init__(self, cfg: PlantConfig, scn: Scenario):
        self.cfg = cfg
        self.scn = scn
        
        self.microsteps_per_rev = dda_steps_per_rev(cfg)
        self.N_steps_total = int(round(scn.sim_time_s / cfg.Ts))
        # Mantém, no mínimo, todo o histórico de um ciclo completo de simulação
        self.plot_history_size = max(self.N_steps_total + 1, 500)
        
        # Estado da Simulação
        self.is_paused = True
        
        # [FIX OTIMIZAÇÃO] Cache do background para blitting
        self.background = None
        
        # [FIX REENTRANCY] Flag para evitar 'draw' síncrono sobreposto
        self._in_freeze_draw = False

        # --- Parâmetros do Cenário (Lidos da GUI no Reset) ---
        self.target_s32: np.ndarray = np.zeros(3, dtype=np.int64)
        self.target_mag: np.ndarray = np.zeros(3, dtype=np.int64)
        self.v_target_sps: np.ndarray = np.zeros(3, dtype=float)
        self.dir_xyz: np.ndarray = np.ones(3, dtype=int)
        self.kp_xyz: np.ndarray = np.zeros(3, dtype=int)
        self.ki_xyz: np.ndarray = np.zeros(3, dtype=int)
        self.kd_xyz: np.ndarray = np.zeros(3, dtype=int)

        # --- Estado da Simulação Real ---
        self.ddas_real: List[DDAStepper] = []
        self.pos_real: np.ndarray = np.zeros(3, dtype=float) 
        self.enc_pos_counts: np.ndarray = np.zeros(3, dtype=float) 
        self.enc_origin_counts: np.ndarray = np.zeros(3, dtype=float)
        
        dda_per_rev = dda_steps_per_rev(self.cfg)
        self.steps_to_counts_ratio = np.array([
            cpr / dda_per_rev if dda_per_rev > 0 else 0 
            for cpr in self.cfg.enc_cpr_xyz
        ])
        
        self.v_real: np.ndarray = np.zeros(3, dtype=float)
        
        # --- Estado do CASC/PID (espelha C globals) ---
        self.g_pi_i_accum: np.ndarray = np.zeros(3, dtype=float)
        self.g_pi_prev_err: np.ndarray = np.zeros(3, dtype=float)
        self.g_pi_d_filt: np.ndarray = np.zeros(3, dtype=float)
        self.g_v_accum: np.ndarray = np.zeros(3, dtype=float)
        self.g_casc_err_s32: np.ndarray = np.zeros(3, dtype=float)
        
        # Estados que mudam a cada passo
        self.k: int = 0
        self.t: float = 0.0
        
        # [FIX 2] Lógica de Carga (lida no Reset, usada no runtime)
        self.C_load_values: np.ndarray = np.zeros(3) 
        self.load_start_times: np.ndarray = np.zeros(3)
        self.load_end_times: np.ndarray = np.full(3, np.inf)
        self.active_C_load: np.ndarray = np.zeros(3)
        self.load_timer_xyz: np.ndarray = np.zeros(3)
        self.B_load: np.ndarray = np.array(cfg.load_B_xyz, dtype=float)
        
        # Estado de Parada por Eixo (lido em tempo real)
        self.is_stopped: np.ndarray = np.array([False, False, False], dtype=bool)
        self.btn_color_off = '0.85' # Cinza claro

        # Buffers de histórico para plotagem
        self.history = {}

        # --- Configuração da Interface Gráfica (GUI) ---
        self.fig = plt.figure(figsize=(17, 10), dpi=96)
        gs = gridspec.GridSpec(4, 4, figure=self.fig)
        self.fig.subplots_adjust(left=0.07, right=0.98, top=0.95, bottom=0.05, hspace=0.7, wspace=0.4)

        # Eixos dos Círculos (Linha 0)
        self.ax_motor_x = self.fig.add_subplot(gs[0, 0], aspect='equal', xlim=(-1.2, 1.2), ylim=(-1.2, 1.2))
        self.ax_motor_y = self.fig.add_subplot(gs[0, 1], aspect='equal', xlim=(-1.2, 1.2), ylim=(-1.2, 1.2))
        self.ax_motor_z = self.fig.add_subplot(gs[0, 2], aspect='equal', xlim=(-1.2, 1.2), ylim=(-1.2, 1.2))
        
        # Painéis de Input (Linha 1)
        self.ax_input_x = self.fig.add_subplot(gs[1, 0])
        self.ax_input_y = self.fig.add_subplot(gs[1, 1])
        self.ax_input_z = self.fig.add_subplot(gs[1, 2])
        
        # Painel de Controle (Linhas 0-1, Coluna 3)
        self.ax_control_panel = self.fig.add_subplot(gs[0:2, 3])
        
        # Gráficos (Linhas 2-3, Colunas 0,1,2)
        gs_graphs = gridspec.GridSpecFromSubplotSpec(3, 1, subplot_spec=gs[2:4, 0:3], hspace=0.3)
        self.ax_graph_pos = self.fig.add_subplot(gs_graphs[0, 0])
        self.ax_graph_vel = self.fig.add_subplot(gs_graphs[1, 0])
        self.ax_graph_err = self.fig.add_subplot(gs_graphs[2, 0])
        
        # Painel de Atrito (Linhas 2-3, Coluna 3)
        self.ax_friction_panel = self.fig.add_subplot(gs[2:4, 3])
        
        # Inicializa plots e widgets
        self.artists = []
        self._init_artists()
        self._init_widgets()
        
        # [FIX V13.4] Agrupa TextBoxes e conecta callbacks estáticos
        self._tbs_scenario = [
            self.txt_s_x, self.txt_v_x, self.txt_dir_x, self.txt_kp_x, self.txt_ki_x, self.txt_kd_x,
            self.txt_s_y, self.txt_v_y, self.txt_dir_y, self.txt_kp_y, self.txt_ki_y, self.txt_kd_y,
            self.txt_s_z, self.txt_v_z, self.txt_dir_z, self.txt_kp_z, self.txt_ki_z, self.txt_kd_z,
        ]
        self._tbs_runtime = [
            self.txt_c_x, self.txt_t_start_x, self.txt_t_end_x,
            self.txt_c_y, self.txt_t_start_y, self.txt_t_end_y,
            self.txt_c_z, self.txt_t_start_z, self.txt_t_end_z
        ]
        # Para cenário: só aplica ao dar ENTER (não redesenha a cada tecla/foco)
        for tb in self._tbs_scenario:
            tb.on_submit(lambda _=None: None)  # sem redraw; reset() que lê
        
        # Guardar cids (connection ids) para (des)ligar live-update dos runtime
        self._live_cids = []
        
        # Carrega os valores iniciais do Scn nos TextBoxes e no estado
        self.reset(None) 
        
        print("Interface Pronta. Pressione 'Play' para iniciar.")
        
        self.animation_interval_ms = 20 # 50Hz refresh rate
        self.sim_steps_per_frame = int(self.animation_interval_ms / (self.cfg.Ts * 1000.0))
        if self.sim_steps_per_frame == 0: self.sim_steps_per_frame = 1
        
        # Timer manual para animação (corrige lag)
        self.timer = self.fig.canvas.new_timer(interval=self.animation_interval_ms)
        self.timer.add_callback(self._on_timer_tick)
        # O timer é iniciado pelo on_play()

    # --- [FIX BLIT-FADE] Helpers para "congelar" o frame ao pausar ---
    
    def _enable_blitting(self):
        """Ativa o blit: marca artistas como animados e cacheia o background."""
        print("Ativando blitting e cacheando background...")
        try:
            # 1. Marca artistas como animados
            for artist in self.artists:
                artist.set_animated(True)
            
            # 2. Força um 'draw' para ter um background limpo
            self.fig.canvas.draw()
            
            # 3. Cacheia o background
            self.background = self.fig.canvas.copy_from_bbox(self.fig.bbox)
        except Exception as e:
            print(f"Falha ao ativar blit/cachear: {e}")
            self.background = None

    def _disable_blitting_and_redraw(self):
        """Desativa o blit: marca artistas como 'não animados' e força um redesenho completo."""
        
        # [FIX REENTRANCY] Evita chamadas de 'draw' reentrantes.
        if self._in_freeze_draw:
            return
        self._in_freeze_draw = True
        
        try:
            print("Desativando blitting e forçando redesenho final.")
            
            # 1. Marca artistas como 'normais' (não-animados)
            for artist in self.artists:
                artist.set_animated(False)
            
            # 2. Invalida o cache
            self.background = None
            
            # [FIX DEBOUNCE/DRAW] Troca 'draw_idle' por 'draw' síncrono.
            # Isso "congela" a tela imediatamente no estado final.
            if hasattr(self.fig.canvas, 'draw'):
                self.fig.canvas.draw()
            elif hasattr(self.fig.canvas, 'draw_idle'):
                # Fallback caso 'draw' não esteja disponível
                self.fig.canvas.draw_idle()
        finally:
            # Garante que a flag seja liberada
            self._in_freeze_draw = False
    # -----------------------------------------------------------------

    # --- [FIX V13.4] Helpers para ligar/desligar callbacks ---
    def _attach_runtime_live(self):
        """Liga live-update (on_text_change) só para os inputs de runtime."""
        self._live_cids.clear()
        for tb in self._tbs_runtime:
            if hasattr(tb, "on_text_change"):
                cid = tb.on_text_change(self._schedule_redraw)
                self._live_cids.append((tb, cid))
            else: # Fallback
                cid = tb.on_submit(self._schedule_redraw)
                self._live_cids.append((tb, cid))

    def _detach_runtime_live(self):
        """Desliga os callbacks de live-update (para não travar em pausa)."""
        for tb, cid in self._live_cids:
            try:
                tb.disconnect(cid)
            except Exception:
                pass
        self._live_cids.clear()
        
    # [FIX V13.5] Adicionado helper de Restart
    def _restart_play(self):
        """Helper para 'Reset + Play' (usado pelo botão Play)."""
        # Recomeça com os valores atuais dos TextBoxes
        self.reset(None)           # pausa, redesenha “congelado”
        self.is_paused = False
        self._enable_blitting()
        self._attach_runtime_live()
        self.timer.start()
        print("Restart: cenário recarregado e execução reiniciada.")
    # ----------------------------------------------------------

    def _latch_all_inputs_from_gui(self):
        """[FIX 2] Lê TODOS os 24 TextBoxes de *configuração* (Cenário + Atrito)."""
        try:
            # Eixo X
            s_x = int(self.txt_s_x.text)
            v_x = float(self.txt_v_x.text)
            dir_x = int(self.txt_dir_x.text)
            kp_x = int(self.txt_kp_x.text)
            ki_x = int(self.txt_ki_x.text)
            kd_x = int(self.txt_kd_x.text)
            
            # Eixo Y
            s_y = int(self.txt_s_y.text)
            v_y = float(self.txt_v_y.text)
            dir_y = int(self.txt_dir_y.text)
            kp_y = int(self.txt_kp_y.text)
            ki_y = int(self.txt_ki_y.text)
            kd_y = int(self.txt_kd_y.text)

            # Eixo Z
            s_z = int(self.txt_s_z.text)
            v_z = float(self.txt_v_z.text)
            dir_z = int(self.txt_dir_z.text)
            kp_z = int(self.txt_kp_z.text)
            ki_z = int(self.txt_ki_z.text)
            kd_z = int(self.txt_kd_z.text)

            # Atualiza o estado do Cenário
            self.target_mag = np.array([s_x, s_y, s_z], dtype=np.int64)
            self.v_target_sps = np.array([v_x, v_y, v_z], dtype=float)
            self.dir_xyz = np.array([dir_x, dir_y, dir_z], dtype=int)
            self.kp_xyz = np.array([kp_x, kp_y, kp_z], dtype=int)
            self.ki_xyz = np.array([ki_x, ki_y, ki_z], dtype=int)
            self.kd_xyz = np.array([kd_x, kd_y, kd_z], dtype=int)
            
            self.dir_xyz[self.dir_xyz >= 0] = 1
            self.dir_xyz[self.dir_xyz < 0] = -1
            self.target_s32 = self.target_mag * self.dir_xyz
            
            # [FIX 2] Atualiza o estado do Atrito
            self.C_load_values[0] = float(self.txt_c_x.text)
            self.load_start_times[0] = float(self.txt_t_start_x.text)
            self.load_end_times[0] = float(self.txt_t_end_x.text)
            
            self.C_load_values[1] = float(self.txt_c_y.text)
            self.load_start_times[1] = float(self.txt_t_start_y.text)
            self.load_end_times[1] = float(self.txt_t_end_y.text)
            
            self.C_load_values[2] = float(self.txt_c_z.text)
            self.load_start_times[2] = float(self.txt_t_start_z.text)
            self.load_end_times[2] = float(self.txt_t_end_z.text)

        except ValueError as e:
            # Lida com "inf"
            if 'inf' in str(e):
                if 'txt_t_end_x' in str(e): self.load_end_times[0] = np.inf
                if 'txt_t_end_y' in str(e): self.load_end_times[1] = np.inf
                if 'txt_t_end_z' in str(e): self.load_end_times[2] = np.inf
            else:
                print(f"Erro ao ler inputs do cenário: {e}")

    def reset(self, event):
        """Reseta a simulação para o estado inicial."""
        print("Carregando cenário e resetando simulação...")
        
        self.is_paused = True
        if hasattr(self, 'timer'): # Só para se o timer já foi criado
            self.timer.stop() # Para o loop de simulação
        
        # [FIX V13.4] Garante que callbacks de runtime estão mortos
        self._detach_runtime_live()
        
        # [FIX GUI] Congela o desenho enquanto atualiza a UI
        with drawing_off():
            # 1. Carrega os valores dos TextBoxes para o estado
            if hasattr(self, 'txt_s_x'):
                self._latch_all_inputs_from_gui() # [FIX 2]
            else:
                # Primeira chamada (do __init__), usa o Scn
                self.target_mag = np.array(self.scn.s_xyz, dtype=np.int64)
                self.v_target_sps = np.array(self.scn.v_xyz, dtype=float)
                self.dir_xyz = np.array(self.scn.dir_xyz, dtype=int)
                self.kp_xyz = np.array(self.scn.kp_xyz, dtype=int)
                self.ki_xyz = np.array(self.scn.ki_xyz, dtype=int)
                self.kd_xyz = np.array(self.scn.kd_xyz, dtype=int)
                self.dir_xyz[self.dir_xyz >= 0] = 1
                self.dir_xyz[self.dir_xyz < 0] = -1
                self.target_s32 = self.target_mag * self.dir_xyz
            
            # [FIX OTIMIZAÇÃO] Trava os limites Y dos gráficos
            # Pega os valores máximos do cenário
            s_max = np.max(self.target_mag)
            v_max = np.max(self.v_target_sps)
            
            # Adiciona uma margem (ex: 20%)
            s_margin = max(10, s_max * 0.2)
            v_margin = max(10, v_max * 0.2)
            
            # Define os limites Y (o erro deixamos mais solto)
            self.ax_graph_pos.set_ylim(-s_margin, s_max + s_margin)
            self.ax_graph_vel.set_ylim(-v_margin, v_max + v_margin)
            # Chute para o erro, ajuste se necessário
            self.ax_graph_err.set_ylim(-500, 500) 
            
            # 2. Reseta os estados da simulação
            self.ddas_real = [DDAStepper(self.cfg.tim6_hz) for _ in range(3)]
            self.pos_real = np.zeros(3, dtype=float)      # Passos emitidos (para gráfico)
            self.enc_pos_counts = np.zeros(3, dtype=float)  # Posição encoder (para feedback)
            self.enc_origin_counts = np.zeros(3, dtype=float) # "Zero" do CASC
            self.v_real = np.zeros(3, dtype=float)          # v_actual_sps
            
            self.g_pi_i_accum = np.zeros(3, dtype=float)
            self.g_pi_prev_err = np.zeros(3, dtype=float)
            self.g_pi_d_filt = np.zeros(3, dtype=float)
            self.g_v_accum = np.zeros(3, dtype=float)
            self.g_casc_err_s32 = np.zeros(3, dtype=float)
            
            self.k = 0
            self.t = 0.0
            
            # [FIX 2] O estado de carga já foi lido. Zera apenas os timers.
            self.active_C_load = np.zeros(3)
            self.load_timer_xyz = np.zeros(3)
            
            self.is_stopped = np.array([False, False, False], dtype=bool)

            # 3. Limpa o histórico
            self.history = {
                't_s': deque(maxlen=self.plot_history_size),
                'x_pos': deque(maxlen=self.plot_history_size),
                'y_pos': deque(maxlen=self.plot_history_size),
                'z_pos': deque(maxlen=self.plot_history_size),
                'x_v': deque(maxlen=self.plot_history_size),
                'y_v': deque(maxlen=self.plot_history_size),
                'z_v': deque(maxlen=self.plot_history_size),
                'x_err': deque(maxlen=self.plot_history_size), # Erro CASC
                'y_err': deque(maxlen=self.plot_history_size), # Erro CASC
                'z_err': deque(maxlen=self.plot_history_size), # Erro CASC
                'sync_span': deque(maxlen=self.plot_history_size),
            }
            self._append_history(0.0, np.zeros(3), self.v_real, np.zeros(3), 0.0)
            
            # 4. Reseta a GUI (botões)
            if hasattr(self, 'btn_stop_x'):
                self.btn_stop_x.color = self.btn_color_off
                self.btn_stop_y.color = self.btn_color_off
                self.btn_stop_z.color = self.btn_color_off
            
            # Envia dados vazios para os artistas
            self._update_artists_data() 
        
        # [FIX BLIT-FADE] O 'reset' deve terminar em um estado estático (blit desligado).
        self._disable_blitting_and_redraw()
            
        print("Reset concluído. Pressione 'Play' para iniciar.")

    def _init_artists(self) -> List:
        """Configura e limpa todos os artistas (linhas, textos) da GUI."""
        self.artists = []
        
        # --- Círculos (Mostram a Posição REAL) ---
        self.motor_lines = []
        self.motor_texts = []
        
        for i, (ax, color, name) in enumerate(zip([self.ax_motor_x, self.ax_motor_y, self.ax_motor_z],
                                                  ['r', 'g', 'b'], ['X', 'Y', 'Z'])):
            ax.clear()
            ax.set_title(f"Eixo {name}", fontsize=9)
            ax.set_xticks([])
            ax.set_yticks([])
            ax.set_xlim(-1.2, 1.2)
            ax.set_ylim(-1.2, 1.2)
            circle = plt.Circle((0, 0), 1.0, color='gray', fill=False, linestyle='--')
            ax.add_artist(circle)
            line, = ax.plot([], [], f'{color}-o', lw=1.25, markersize=8, antialiased=False)
            self.motor_lines.append(line)
            self.artists.append(line)
            
            txt = ax.text(0, -1.6, '0.0v', ha='center', fontsize=8) # Fonte menor
            self.motor_texts.append(txt)
            self.artists.append(txt)
        
        self.time_text = self.fig.suptitle('Tempo: 0.00 s', fontsize=14)
        self.artists.append(self.time_text)
            
        # --- Gráficos (Mostram Posição/Velocidade REAL) ---
        self.ax_graph_pos.clear()
        self.ax_graph_pos.set_ylabel("Posição (steps)")
        self.ax_graph_pos.grid(True)
        self.line_pos_x, = self.ax_graph_pos.plot([], [], 'r-', label='X pos', antialiased=False, lw=1.25)
        self.line_pos_y, = self.ax_graph_pos.plot([], [], 'g-', label='Y pos', antialiased=False, lw=1.25)
        self.line_pos_z, = self.ax_graph_pos.plot([], [], 'b-', label='Z pos', antialiased=False, lw=1.25)
        self.ax_graph_pos.legend(loc='upper left')
        self.artists.extend([self.line_pos_x, self.line_pos_y, self.line_pos_z])

        self.ax_graph_vel.clear()
        self.ax_graph_vel.set_ylabel("Velocidade (sps)")
        self.ax_graph_vel.grid(True)
        self.line_vel_x, = self.ax_graph_vel.plot([], [], 'r-', label='X vel', antialiased=False, lw=1.25)
        self.line_vel_y, = self.ax_graph_vel.plot([], [], 'g-', label='Y vel', antialiased=False, lw=1.25)
        self.line_vel_z, = self.ax_graph_vel.plot([], [], 'b-', label='Z vel', antialiased=False, lw=1.25)
        self.ax_graph_vel.legend(loc='upper left')
        self.artists.extend([self.line_vel_x, self.line_vel_y, self.line_vel_z])
        
        # --- Gráfico de Erro (Mostra Erro CASC) ---
        self.ax_graph_err.clear()
        self.ax_graph_err.set_ylabel("Erro CASC (steps)")
        self.ax_graph_err.set_xlabel("Tempo (s)")
        self.ax_graph_err.grid(True)
        self.line_err_x, = self.ax_graph_err.plot([], [], 'r-', label='Erro CASC X', antialiased=False, lw=1.25)
        self.line_err_y, = self.ax_graph_err.plot([], [], 'g-', label='Erro CASC Y', antialiased=False, lw=1.25)
        self.line_err_z, = self.ax_graph_err.plot([], [], 'b-', label='Erro CASC Z', antialiased=False, lw=1.25)
        self.ax_graph_err.legend(loc='upper left')
        self.artists.extend([self.line_err_x, self.line_err_y, self.line_err_z])
        
        # [FIX 1] Textos de Carga (Timers) - Removidos daqui
        self.ax_friction_panel.clear(); self.ax_friction_panel.axis('off')
        
        return self.artists
    
    # --- [FIX GUI] Funções de Debounce ---
    def _setup_debounced_redraw(self):
        """Cria o timer de debounce para a GUI."""
        self._debounce_timer = self.fig.canvas.new_timer(interval=120)
        
        # [FIX DEBOUNCE/DRAW] ESSENCIAL: Dispara apenas uma vez.
        self._debounce_timer.single_shot = True
        
        def _fire():
            # [FIX REENTRANCY] Se já estiver pausado, NÃO faça nada.
            # Isso evita a colisão de redraw com o TextBox.
            if self.is_paused:
                return
            
            # Se estava rodando, pausar é a ação correta.
            # O on_pause() já chama _disable_blitting_and_redraw(),
            # que faz o 'draw()' síncrono.
            self.on_pause(None)
            
            # [FIX REENTRANCY] O 'draw_idle()' que estava aqui foi removido
            # por ser redundante (on_pause já redesenha).
                
        self._debounce_timer.add_callback(_fire)

    def _schedule_redraw(self, *_):
        """Agenda um redesenho (chamado pelos TextBoxes)."""
        
        # [FIX REENTRANCY] Não agenda redraws se já estiver pausado.
        # Isso é a chave para o TextBox não travar.
        if self.is_paused:
            return
            
        try:
            self._debounce_timer.stop()
        except Exception:
            pass
        # (Re)inicia o timer single-shot
        self._debounce_timer.start()
    # -----------------------------------
        
    def _init_widgets(self):
        """Cria os botões e caixas de texto no layout correto."""
        
        # --- Painel de Controle (Botões) ---
        self.ax_control_panel.axis('off')
        gs_control = gridspec.GridSpecFromSubplotSpec(
            2, 1,
            subplot_spec=self.ax_control_panel.get_subplotspec(),
            hspace=0.2,
        )
        
        # Linha 0: Play / Pause
        gs_play_pause = gridspec.GridSpecFromSubplotSpec(1, 2, subplot_spec=gs_control[0, 0], wspace=0.1)
        ax_play = self.fig.add_subplot(gs_play_pause[0, 0])
        self.btn_play = Button(ax_play, 'Play', color='lightgreen')
        self.btn_play.on_clicked(self.on_play)

        ax_pause = self.fig.add_subplot(gs_play_pause[0, 1])
        self.btn_pause = Button(ax_pause, 'Pause', color='lightyellow')
        self.btn_pause.on_clicked(self.on_pause)

        # Linha 1: Reset
        ax_reset = self.fig.add_subplot(gs_control[1, 0])
        self.btn_reset = Button(ax_reset, 'Carregar Cenário e Resetar', color='lightcoral')
        self.btn_reset.on_clicked(self.reset)
        
        # --- Painéis de Input (Cenário) ---
        
        # Inputs Eixo X
        self.ax_input_x.clear(); self.ax_input_x.axis('off'); self.ax_input_x.set_title("Eixo X (s*, v*, dir, Kp, Ki, Kd)", fontsize=10)
        gs_input_x = gridspec.GridSpecFromSubplotSpec(
            6, 1,
            subplot_spec=self.ax_input_x.get_subplotspec(),
            hspace=0.1,
        )
        self.txt_s_x   = TextBox(self.fig.add_subplot(gs_input_x[0, 0]), "s (passos): ", initial=str(self.scn.s_xyz[0]))
        self.txt_v_x   = TextBox(self.fig.add_subplot(gs_input_x[1, 0]), "v (sps): ", initial=str(self.scn.v_xyz[0]))
        self.txt_dir_x = TextBox(self.fig.add_subplot(gs_input_x[2, 0]), "dir (1/-1): ", initial=str(self.scn.dir_xyz[0]))
        self.txt_kp_x  = TextBox(self.fig.add_subplot(gs_input_x[3, 0]), "Kp: ", initial=str(self.scn.kp_xyz[0]))
        self.txt_ki_x  = TextBox(self.fig.add_subplot(gs_input_x[4, 0]), "Ki: ", initial=str(self.scn.ki_xyz[0]))
        self.txt_kd_x  = TextBox(self.fig.add_subplot(gs_input_x[5, 0]), "Kd: ", initial=str(self.scn.kd_xyz[0]))
        
        # Inputs Eixo Y
        self.ax_input_y.clear(); self.ax_input_y.axis('off'); self.ax_input_y.set_title("Eixo Y (s*, v*, dir, Kp, Ki, Kd)", fontsize=10)
        gs_input_y = gridspec.GridSpecFromSubplotSpec(
            6, 1,
            subplot_spec=self.ax_input_y.get_subplotspec(),
            hspace=0.1,
        )
        self.txt_s_y   = TextBox(self.fig.add_subplot(gs_input_y[0, 0]), "s (passos): ", initial=str(self.scn.s_xyz[1]))
        self.txt_v_y   = TextBox(self.fig.add_subplot(gs_input_y[1, 0]), "v (sps): ", initial=str(self.scn.v_xyz[1]))
        self.txt_dir_y = TextBox(self.fig.add_subplot(gs_input_y[2, 0]), "dir (1/-1): ", initial=str(self.scn.dir_xyz[1]))
        self.txt_kp_y  = TextBox(self.fig.add_subplot(gs_input_y[3, 0]), "Kp: ", initial=str(self.scn.kp_xyz[1]))
        self.txt_ki_y  = TextBox(self.fig.add_subplot(gs_input_y[4, 0]), "Ki: ", initial=str(self.scn.ki_xyz[1]))
        self.txt_kd_y  = TextBox(self.fig.add_subplot(gs_input_y[5, 0]), "Kd: ", initial=str(self.scn.kd_xyz[1]))

        # Inputs Eixo Z
        self.ax_input_z.clear(); self.ax_input_z.axis('off'); self.ax_input_z.set_title("Eixo Z (s*, v*, dir, Kp, Ki, Kd)", fontsize=10)
        gs_input_z = gridspec.GridSpecFromSubplotSpec(
            6, 1,
            subplot_spec=self.ax_input_z.get_subplotspec(),
            hspace=0.1,
        )
        self.txt_s_z   = TextBox(self.fig.add_subplot(gs_input_z[0, 0]), "s (passos): ", initial=str(self.scn.s_xyz[2]))
        self.txt_v_z   = TextBox(self.fig.add_subplot(gs_input_z[1, 0]), "v (sps): ", initial=str(self.scn.v_xyz[2]))
        self.txt_dir_z = TextBox(self.fig.add_subplot(gs_input_z[2, 0]), "dir (1/-1): ", initial=str(self.scn.dir_xyz[2]))
        self.txt_kp_z  = TextBox(self.fig.add_subplot(gs_input_z[3, 0]), "Kp: ", initial=str(self.scn.kp_xyz[2]))
        self.txt_ki_z  = TextBox(self.fig.add_subplot(gs_input_z[4, 0]), "Ki: ", initial=str(self.scn.ki_xyz[2]))
        self.txt_kd_z  = TextBox(self.fig.add_subplot(gs_input_z[5, 0]), "Kd: ", initial=str(self.scn.kd_xyz[2]))
        
        # --- Painel de Atrito (Runtime) ---
        # [FIX 1] Alterado para 4 linhas
        gs_friction = gridspec.GridSpecFromSubplotSpec(
            4, 1,
            subplot_spec=self.ax_friction_panel.get_subplotspec(),
            hspace=0.1,
        )
        
        # Atrito X
        # [FIX 1] Aumentado wspace
        gs_fric_x = gridspec.GridSpecFromSubplotSpec(3, 2, subplot_spec=gs_friction[0, 0], wspace=0.4)
        self.txt_c_x = TextBox(self.fig.add_subplot(gs_fric_x[0, :]), "Atrito X (C): ", initial="0.0")
        self.txt_t_start_x = TextBox(self.fig.add_subplot(gs_fric_x[1, 0]), "Início(s):", initial="0.0")
        self.txt_t_end_x = TextBox(self.fig.add_subplot(gs_fric_x[1, 1]), "Fim(s):", initial="inf")
        self.btn_stop_x = Button(self.fig.add_subplot(gs_fric_x[2, :]), 'Stop X', color=self.btn_color_off)
        self.btn_stop_x.on_clicked(self.on_stop_x)
        
        # Atrito Y
        # [FIX 1] Aumentado wspace
        gs_fric_y = gridspec.GridSpecFromSubplotSpec(3, 2, subplot_spec=gs_friction[1, 0], wspace=0.4)
        self.txt_c_y = TextBox(self.fig.add_subplot(gs_fric_y[0, :]), "Atrito Y (C): ", initial="0.0")
        self.txt_t_start_y = TextBox(self.fig.add_subplot(gs_fric_y[1, 0]), "Início(s):", initial="0.0")
        self.txt_t_end_y = TextBox(self.fig.add_subplot(gs_fric_y[1, 1]), "Fim(s):", initial="inf")
        self.btn_stop_y = Button(self.fig.add_subplot(gs_fric_y[2, :]), 'Stop Y', color=self.btn_color_off)
        self.btn_stop_y.on_clicked(self.on_stop_y)
        
        # Atrito Z
        # [FIX 1] Aumentado wspace
        gs_fric_z = gridspec.GridSpecFromSubplotSpec(3, 2, subplot_spec=gs_friction[2, 0], wspace=0.4)
        self.txt_c_z = TextBox(self.fig.add_subplot(gs_fric_z[0, :]), "Atrito Z (C): ", initial="0.0")
        self.txt_t_start_z = TextBox(self.fig.add_subplot(gs_fric_z[1, 0]), "Início(s):", initial="0.0")
        self.txt_t_end_z = TextBox(self.fig.add_subplot(gs_fric_z[1, 1]), "Fim(s):", initial="inf")
        self.btn_stop_z = Button(self.fig.add_subplot(gs_fric_z[2, :]), 'Stop Z', color=self.btn_color_off)
        self.btn_stop_z.on_clicked(self.on_stop_z)

        # [FIX 1] Painel de Timers de Carga (na linha 4)
        ax_timers = self.fig.add_subplot(gs_friction[3, 0])
        ax_timers.axis('off')
        self.text_timer_x = ax_timers.text(0.5, 0.66, 'Carga X: 0.00s', ha='center', va='center', fontsize=9)
        self.text_timer_y = ax_timers.text(0.5, 0.33, 'Carga Y: 0.00s', ha='center', va='center', fontsize=9)
        self.text_timer_z = ax_timers.text(0.5, 0.0, 'Carga Z: 0.00s', ha='center', va='center', fontsize=9)
        
        # [FIX OTIMIZAÇÃO] Adiciona os textos de timer aos artistas para o blitting
        self.artists.extend([self.text_timer_x, self.text_timer_y, self.text_timer_z])
        
        # --- [FIX GUI] Ligar o Debounce ---
        self._setup_debounced_redraw()
        
        # [FIX V13.4] O loop de conexão de 'all_textboxes' foi REMOVIDO daqui
        # e substituído pela lógica em __init__ e nos helpers
        # _attach_runtime_live / _detach_runtime_live.
        # ---------------------------------

    # [FIX V13.5] Modificado para incluir o 'else'
    def on_play(self, event):
        if self.is_paused:
            self.is_paused = False
            
            # [FIX BLIT-FADE] Ativa o modo blit ANTES de iniciar o timer
            self._enable_blitting()
            
            # [FIX V13.4] Liga live-update do Atrito
            self._attach_runtime_live()
            
            self.timer.start() # Inicia o loop de _on_timer_tick
            print("Simulação iniciada (com blit).")
        else:
            # [FIX V13.5] Clique de novo em Play = Reset + Play
            self._restart_play()
        
    def on_pause(self, event):
        if not self.is_paused:
            self.is_paused = True
            self.timer.stop() # Para o loop de _on_timer_tick
            
            # [FIX V13.4] Desliga live-update do Atrito
            self._detach_runtime_live()
            
            # [FIX BLIT-FADE] Desativa o blit e redesenha o frame final
            self._disable_blitting_and_redraw()
            
            print("Simulação pausada (blit desativado).")
            
    def on_stop_x(self, event):
        self.is_stopped[0] = not self.is_stopped[0] # Toggle
        self.btn_stop_x.color = 'red' if self.is_stopped[0] else self.btn_color_off
        if self.is_stopped[0]:
             self.g_pi_i_accum[0] = 0.0 # Reseta o integrador
            
    def on_stop_y(self, event):
        self.is_stopped[1] = not self.is_stopped[1] # Toggle
        self.btn_stop_y.color = 'red' if self.is_stopped[1] else self.btn_color_off
        if self.is_stopped[1]:
             self.g_pi_i_accum[1] = 0.0 # Reseta o integrador
            
    def on_stop_z(self, event):
        self.is_stopped[2] = not self.is_stopped[2] # Toggle
        self.btn_stop_z.color = 'red' if self.is_stopped[2] else self.btn_color_off
        if self.is_stopped[2]:
             self.g_pi_i_accum[2] = 0.0 # Reseta o integrador
            
    def _read_inputs(self):
        """[FIX 2] Esta função foi DELETADA (movida para 'reset')"""
        pass
            
    def _append_history(self, t, pos_rel_dda_from_enc, v_real, casc_err, span_real):
        """Adiciona o estado atual aos buffers de histórico."""
        self.history['t_s'].append(t)
        self.history['x_pos'].append(pos_rel_dda_from_enc[0])
        self.history['y_pos'].append(pos_rel_dda_from_enc[1])
        self.history['z_pos'].append(pos_rel_dda_from_enc[2])
        self.history['x_v'].append(v_real[0])
        self.history['y_v'].append(v_real[1])
        self.history['z_v'].append(v_real[2])
        self.history['x_err'].append(casc_err[0])
        self.history['y_err'].append(casc_err[1])
        self.history['z_err'].append(casc_err[2])
        self.history['sync_span'].append(span_real)

    # ===================================================================
    # "CHEFE" (TIM7 @ 1kHz) - CASC + PID + RAMPA
    # ===================================================================
    def _step(self):
        """Executa um único passo de 1ms da simulação (CASC/PID/RAMPA)."""
        if self.k >= self.N_steps_total:
            if not self.is_paused:
                self.is_paused = True
                self.timer.stop()
                print("Tempo de simulação concluído.")
                
                # [FIX V13.4] Desliga callbacks antes de redesenhar
                self._detach_runtime_live()
                # [FIX BLIT-FADE] Desativa o blit e redesenha o frame final
                self._disable_blitting_and_redraw()
            return

        t = self.k * self.cfg.Ts
        
        # --- 1. Lógica de Carga (para a simulação Real) ---
        current_C_load = np.zeros(3)
        for i in range(3):
            # [FIX 2] Usa os valores 'latched' (self.load_start_times)
            if self.load_start_times[i] <= t < self.load_end_times[i]:
                current_C_load[i] = self.C_load_values[i]
                self.load_timer_xyz[i] += self.cfg.Ts
            else:
                self.load_timer_xyz[i] = 0.0
        self.active_C_load = current_C_load
        
        
        # --- 2. LÓGICA CASC (Achar Mestre "Percentual") ---
        
        master_axis = -1
        mestre_prog_num = 0
        mestre_prog_den = 1
        
        pos_enc_rel = self.enc_pos_counts - self.enc_origin_counts
        pos_rel_dda_from_enc = np.array([
            conv_enc_to_dda(self.cfg, pos_enc_rel[0], 0),
            conv_enc_to_dda(self.cfg, pos_enc_rel[1], 1),
            conv_enc_to_dda(self.cfg, pos_enc_rel[2], 2)
        ])
        
        alvo_final_mag = self.target_mag

        for i in range(3):
            total_mag = alvo_final_mag[i]
            if total_mag == 0: continue # Eixo não está em movimento
            
            prog_num = np.abs(pos_rel_dda_from_enc[i]) # Progresso (magnitude)
            prog_den = total_mag
            
            if prog_num < 0: prog_num = 0
            if prog_num > prog_den: prog_num = prog_den # Clamp (chegou)

            if master_axis == -1:
                master_axis = i
                mestre_prog_num = prog_num
                mestre_prog_den = prog_den if prog_den > 0 else 1
            else:
                # É (prog_i / prog_den_i) < (mestre_prog_num / mestre_prog_den) ?
                prog_i_64 = prog_num * mestre_prog_den
                prog_mestre_64 = mestre_prog_num * prog_den

                if prog_i_64 < prog_mestre_64:
                    master_axis = i
                    mestre_prog_num = prog_num
                    mestre_prog_den = prog_den if prog_den > 0 else 1
        
        # Condição de Término Correta
        if master_axis == -1 or (mestre_prog_den > 0 and mestre_prog_num >= mestre_prog_den):
            if not self.is_paused:
                print(f"Fim do movimento CASC (Mestre: {master_axis}, Prog: {mestre_prog_num}/{mestre_prog_den}).")
                self.is_paused = True # Para o movimento
                self.timer.stop()
                
                # [FIX V13.4] Desliga callbacks antes de redesenhar
                self._detach_runtime_live()
                # [FIX BLIT-FADE] Desativa o blit e redesenha o frame final
                self._disable_blitting_and_redraw()
                
            self.v_real = np.zeros(3) # Zera a velocidade
            for i in range(3): self.ddas_real[i].reset()
            return # Fim

        # --- 3. LOOP DE CONTROLE (PID + CASC + Rampa) ---
        
        v_final_sps = np.zeros(3) # Velocidade final pós-PID/CASC
        v_ramped = np.zeros(3)    # Velocidade final pós-Rampa
        
        rem_steps_mestre = mestre_prog_den - mestre_prog_num
        
        for axis in range(3):
            total_s32 = self.target_s32[axis]
            v_cmd_sps_ideal = self.v_target_sps[axis]
            
            if total_s32 == 0:
                 v_final_sps[axis] = 0.0
                 continue # Vai para o próximo eixo
            
            # 3a. CÁLCULO DO "ALVO SINCRONIZADO" (CASC)
            num_sync_64 = mestre_prog_num * total_s32
            desired_dda_steps = (num_sync_64 + (mestre_prog_den // 2)) / mestre_prog_den
            
            # 3b. CÁLCULO DO PID (CASC + PID)
            actual_dda_steps = pos_rel_dda_from_enc[axis] 
            
            err = desired_dda_steps - actual_dda_steps
            self.g_casc_err_s32[axis] = err
            
            if abs(err) < self.cfg.deadband_steps:
                err = 0.0
                
            iacc = self.g_pi_i_accum[axis] + err
            iacc = np.clip(iacc, -self.cfg.i_clamp, self.cfg.i_clamp)
            
            draw = err - self.g_pi_prev_err[axis]
            self.g_pi_prev_err[axis] = err
            
            # Filtro D Corrigido
            d_filt = self.g_pi_d_filt[axis]
            decay = 1.0 / (1 << self.cfg.kd_alpha_bits) # 1/256
            d_filt = d_filt + (draw - d_filt) * decay
            self.g_pi_d_filt[axis] = d_filt
            
            pterm = (self.kp_xyz[axis] * err) / self.cfg.k_scale
            iterm = (self.ki_xyz[axis] * iacc) / self.cfg.k_scale
            dterm = (self.kd_xyz[axis] * d_filt) / self.cfg.k_scale
            
            corr = pterm + iterm + dterm
            corr = np.clip(corr, -self.cfg.max_sps, self.cfg.max_sps)
            
            v_adj = v_cmd_sps_ideal + corr
            v_adj = np.clip(v_adj, 0, self.cfg.max_sps)
            
            if not (v_adj == 0 or v_adj == self.cfg.max_sps):
                self.g_pi_i_accum[axis] = iacc
                
            v_final_sps[axis] = v_adj

        # --- 4. CÁLCULO DE RAMPA ---
        a_sps2 = self.cfg.accel_sps2
        v_now = self.v_real # v_actual_sps (estado salvo do último loop)
        
        s_brake = np.zeros(3)
        denom = 2 * a_sps2
        if denom > 0:
            s_brake = (v_now * v_now) / denom

        steps_avail = a_sps2 * self.cfg.Ts
        
        for axis in range(3):
            if rem_steps_mestre <= s_brake[axis]:
                v_ramped[axis] = np.clip(v_now[axis] - steps_avail, 0, v_now[axis])
            elif v_now[axis] < v_final_sps[axis]:
                v_ramped[axis] = np.clip(v_now[axis] + steps_avail, v_now[axis], v_final_sps[axis])
            elif v_now[axis] > v_final_sps[axis]:
                v_ramped[axis] = np.clip(v_now[axis] - steps_avail, v_final_sps[axis], v_now[axis])
            else:
                v_ramped[axis] = v_now[axis] # Cruzeiro
                
        self.v_real = np.clip(v_ramped, 0, self.cfg.max_sps) # Salva v_actual_sps
        
        # --- 5. FÍSICA (Atrito) ---
        v_eff_real = self.v_real - np.sign(self.v_real) * (self.active_C_load + self.B_load * np.abs(self.v_real))
        flip_mask = (np.sign(self.v_real) != np.sign(v_eff_real)) & (np.abs(self.v_real) > 1e-9)
        v_eff_real[flip_mask] = 0.0
        
        # --- 6. PARADA DE SEGURANÇA (Botões / Stall) ---
        is_stalled = flip_mask
        is_manually_stopped = self.is_stopped
        individual_stop_state = is_stalled | is_manually_stopped
        global_stop_triggered = np.any(individual_stop_state)
        
        global_op_factor = 1.0 - float(global_stop_triggered) 
        
        v_final_DDA = v_eff_real * global_op_factor
        
        self.v_real = self.v_real * global_op_factor # Zera a rampa também
        
        # Reset do Acumulador DDA
        for i in range(3):
            if abs(v_final_DDA[i]) < 1e-12:
                self.ddas_real[i].reset()
        
        # --- 7. "OPERÁRIO" DDA (TIM6 @ 50kHz) ---
        steps_emit_real_mag = np.array([self.ddas_real[i].emit_steps(v_final_DDA[i], self.cfg.Ts) for i in range(3)], dtype=float)
        
        steps_emit_real = steps_emit_real_mag * self.dir_xyz
        
        self.pos_real += steps_emit_real
        
        emitted_enc_counts = (steps_emit_real_mag * self.dir_xyz) * self.steps_to_counts_ratio
        self.enc_pos_counts += emitted_enc_counts
        
        v_emit_real = steps_emit_real_mag / self.cfg.Ts 

        span_real = float(np.max(self.pos_real) - np.min(self.pos_real))
        
        # --- 8. Salvar no Histórico ---
        self._append_history(t, pos_rel_dda_from_enc, v_emit_real, self.g_casc_err_s32, span_real)
        self.t = t
        self.k += 1

    def _on_timer_tick(self):
        """Função chamada pelo self.timer a cada 20ms."""
        
        # Roda a simulação (ex: 20 steps de 1ms)
        for _ in range(self.sim_steps_per_frame): 
            if not self.is_paused:
                self._step()
            else:
                break # Para o loop for se o _step() pausar
        
        # [FIX BLIT-FADE] Se o _step() pausou a simulação, o blit
        # foi desativado (self.background = None), e não devemos desenhar.
        if self.is_paused or self.background is None:
            return
            
        # --- Início do Bloco de Desenho ---
        
        for i in range(3):
            pos_mod = self.pos_real[i] % self.microsteps_per_rev
            angle = (pos_mod / self.microsteps_per_rev) * 2 * np.pi
            
            x_coord = np.cos(angle)
            y_coord = np.sin(angle)
            self.motor_lines[i].set_data([0, x_coord], [0, y_coord])
            
            revs = self.pos_real[i] / self.microsteps_per_rev
            self.motor_texts[i].set_text(f"{revs:.2f}v")

        self.time_text.set_text(f'Tempo: {self.t:.2f} s / {self.scn.sim_time_s:.2f} s')

        # Gráficos (mostram dados do Histórico, que vem do Encoder)
        self._update_artists_data()
        
        # [FIX OTIMIZAÇÃO] Usa Blitting ao invés de draw_idle()
        try:
            # 1. Restaura o background limpo (muito rápido)
            self.fig.canvas.restore_region(self.background)
            
            # 2. Redesenha *apenas* os artistas que mudaram (muito rápido)
            for artist in self.artists:
                self.fig.draw_artist(artist)
                
            # 3. "Cola" os artistas atualizados na tela (muito rápido)
            self.fig.canvas.blit(self.fig.bbox)
            
        except Exception as e:
            # Blit falhou (provavelmente redimensionamento da janela).
            print(f"Erro de Blit ({e}). Pausando e forçando redesenho estático.")
            self.on_pause(None) # Força um pause para "congelar" o frame
 

    def _update_artists_data(self):
        """Helper para atualizar todos os dados dos gráficos."""
        t_data = self.history['t_s']
        self.line_pos_x.set_data(t_data, self.history['x_pos'])
        self.line_pos_y.set_data(t_data, self.history['y_pos'])
        self.line_pos_z.set_data(t_data, self.history['z_pos'])
        
        self.line_vel_x.set_data(t_data, self.history['x_v'])
        self.line_vel_y.set_data(t_data, self.history['y_v'])
        self.line_vel_z.set_data(t_data, self.history['z_v'])
        
        self.line_err_x.set_data(t_data, self.history['x_err'])
        self.line_err_y.set_data(t_data, self.history['y_err'])
        self.line_err_z.set_data(t_data, self.history['z_err'])
        
        if (not self.is_paused) and len(t_data) > 1:
            # [FIX OTIMIZAÇÃO] scaley=False pois os limites Y são fixos
            for ax in [self.ax_graph_pos, self.ax_graph_vel, self.ax_graph_err]:
                ax.relim()
                # O eixo X (tempo) ainda escala, o Y não.
                ax.autoscale_view(scalex=True, scaley=False)
                
        # Atualiza os textos do cronômetro
        for i, timer_text in enumerate([self.text_timer_x, self.text_timer_y, self.text_timer_z]):
            timer_text.set_text(f'Carga {["X","Y","Z"][i]}: {self.load_timer_xyz[i]:.2f}s')
            if self.load_timer_xyz[i] > 0:
                timer_text.set_color('red')
            else:
                timer_text.set_color('black')

# =========================
# Ponto de Entrada Principal
# =========================

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Simulador CASC/PID com ganhos derivados dos dados SWV.")
    parser.add_argument(
        "--axes",
        default="X:256,Y:256,Z:256",
        help="Mapa de eixos no formato X:microstep,Y:microstep (default: X/Y/Z em 1/256).",
    )
    args = parser.parse_args()

    axis_map = parse_axis_map(args.axes)
    kp_xyz, ki_xyz, kd_xyz = gains_from_catalog(axis_map)

    cfg = PlantConfig(
        microstep_factor=axis_map[0][1],  # assume todos os eixos usam o mesmo microstep
        enc_cpr_xyz=(40000, 2500, 40000), # X, Y, Z
        kd_alpha_bits=8,
        step_high_ticks=1,
        step_low_ticks=1,
    )
    
    scn = Scenario(
        s_xyz=(20000, 16000, 12000),
        v_xyz=(10000, 8000,  6000),
        dir_xyz=(1, 1, 1),
        kp_xyz=kp_xyz,
        ki_xyz=ki_xyz,
        kd_xyz=kd_xyz,
        sim_time_s=3.0,
        use_dda=True
    )

    sim_app = InteractiveSim(cfg, scn)
    plt.show()
    print("Simulação interativa encerrada.")
