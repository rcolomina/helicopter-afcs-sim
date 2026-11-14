"""Mark-on-Target (MOT) mode logic: Mark -> Approach 1 -> Approach 2 -> Hover.

The mode is a state machine wrapped around feedback loops:

* MARK        the target position is stored; radar-altitude hold for a few seconds.
* APPROACH_1  the pattern: turn downwind on an offset track, decelerate to the gate
              speed and descend to the gate height, then a course reversal that
              rolls out on the final course, into the wind, through the target.
* APPROACH_2  the final: decelerate at a constant rate and descend so that the
              helicopter reaches zero groundspeed and hover height together over
              the hover point, a short distance downwind of the target.
* HOVER       position hold over the hover point, heading into the wind.

The geometry is computed in a wind-aligned frame (+y = into the wind) with the
marked target at the origin.
"""
from __future__ import annotations

from enum import Enum

import numpy as np

from .model import Command, State, wrap
from .params import G, KT, MOTParams


class Phase(str, Enum):
    MARK = "MARK"
    APPROACH_1 = "APPROACH_1"
    APPROACH_2 = "APPROACH_2"
    HOVER = "HOVER"


class Lateral(str, Enum):
    HOLD = "HOLD"
    DOWNWIND = "DOWNWIND"
    REVERSAL = "REVERSAL"
    FINAL = "FINAL"


def track_line(pos: np.ndarray, p0: np.ndarray, direction: np.ndarray, l1: float) -> float:
    """Heading command to capture and follow a straight line (L1-style law)."""
    d = direction / np.linalg.norm(direction)
    rel = pos - p0
    cross = d[0] * rel[1] - d[1] * rel[0]  # > 0: left of the line
    return wrap(np.arctan2(d[1], d[0]) - np.arctan(cross / l1))


class MarkOnTarget:
    def __init__(self, p: MOTParams, s0: State):
        self.p = p
        self.phase = Phase.MARK
        self.lateral = Lateral.HOLD
        self.t_phase = 0.0
        self.v_ref = s0.speed
        self.h_ref = s0.h
        self.psi_hold = s0.psi
        self.decel = p.decel_kt_s * KT
        self.vs = p.descent_fpm / 60.0
        vg = p.gate_speed_kt * KT
        self.r_rev = vg**2 / (G * np.tan(np.radians(p.reversal_bank_deg)))
        self.hover_point = np.array([0.0, -p.hover_offset_m])
        final_len = vg**2 / (2 * self.decel) + vg * p.final_margin_s
        self.y_reversal = self.hover_point[1] - final_len
        self.side = 1.0
        self.r_turn = self.r_rev

    # ------------------------------------------------------------------ logic
    def _transitions(self, s: State) -> None:
        p = self.p
        if self.phase is Phase.MARK and self.t_phase >= p.mark_hold_s:
            self.side = 1.0 if s.x >= -1.0 else -1.0
            self._enter(Phase.APPROACH_1)
            self.lateral = Lateral.DOWNWIND
        if self.phase is Phase.APPROACH_1:
            if self.lateral is Lateral.DOWNWIND:
                on_track = abs(s.x - 2 * self.side * self.r_rev) < 40 and abs(wrap(s.psi + np.pi / 2)) < np.radians(15)
                if on_track and s.y <= self.y_reversal:
                    self.lateral = Lateral.REVERSAL
                    self.r_turn = max(abs(s.x) / 2.0, 0.5 * self.r_rev)
            elif self.lateral is Lateral.REVERSAL:
                if abs(wrap(s.psi - np.pi / 2)) < np.radians(10):
                    self.lateral = Lateral.FINAL
            elif self.lateral is Lateral.FINAL:
                if abs(s.x) < 15 and abs(wrap(s.psi - np.pi / 2)) < np.radians(5):
                    self._enter(Phase.APPROACH_2)
        if self.phase is Phase.APPROACH_2:
            d_go = float(np.linalg.norm(self.hover_point - s.pos))
            if d_go < 5.0 and s.speed < 0.5 and abs(s.h - p.hover_height_ft) < 3.0:
                self._enter(Phase.HOVER)

    def _enter(self, phase: Phase) -> None:
        self.phase = phase
        self.t_phase = 0.0

    # --------------------------------------------------------------- commands
    def update(self, s: State, dt: float) -> Command:
        self.t_phase += dt
        self._transitions(s)
        p = self.p
        k_h = 0.5  # 1/s height loop gain
        vz_ff = 0.0

        if self.phase is Phase.MARK:
            psi_c = self.psi_hold
            vel = self.v_ref * np.array([np.cos(s.psi), np.sin(s.psi)])
            return Command(vel, psi_c, k_h * (self.h_ref - s.h))

        if self.phase is Phase.APPROACH_1:
            vg = p.gate_speed_kt * KT
            self.v_ref = max(self.v_ref - self.decel * dt, vg)
            if self.h_ref > p.gate_height_ft:
                self.h_ref = max(self.h_ref - self.vs * dt, p.gate_height_ft)
                vz_ff = -self.vs
            limit = None
            if self.lateral is Lateral.DOWNWIND:
                p0 = np.array([2 * self.side * self.r_rev, 0.0])
                psi_c = track_line(s.pos, p0, np.array([0.0, -1.0]), p.l1_m)
            elif self.lateral is Lateral.REVERSAL:
                turn = -self.side  # east of the final course -> right-hand reversal
                psi_c = wrap(s.psi + turn * 0.6)
                limit = s.speed / self.r_turn
            else:  # FINAL capture
                psi_c = track_line(s.pos, np.zeros(2), np.array([0.0, 1.0]), p.l1_m)
            vel = self.v_ref * np.array([np.cos(s.psi), np.sin(s.psi)])
            vz = max(vz_ff + k_h * (self.h_ref - s.h), -self.vs)
            return Command(vel, psi_c, vz, limit)

        into_wind = np.pi / 2
        if self.phase is Phase.APPROACH_2:
            d_go = max(self.hover_point[1] - s.y, 0.0)
            # constant-deceleration speed profile that stops exactly over the hover point
            # (lead of one response time constant so the lagged aircraft does not overshoot)
            self.v_ref = min(self.v_ref, np.sqrt(2 * self.decel * max(d_go - s.speed * 1.0, 0.0)))
            if d_go < 3.0:
                return self._position_hold(s, k_h)
            # descend so that the hover height is reached when the speed reaches zero
            v = max(s.speed, 1.0)
            t_go = max(d_go - v**2 / (2 * self.decel), 0.0) / v + v / self.decel
            if self.h_ref > p.hover_height_ft and (self.h_ref - p.hover_height_ft) >= self.vs * t_go - 1.0:
                self.h_ref = max(self.h_ref - self.vs * dt, p.hover_height_ft)
                vz_ff = -self.vs
            lateral = float(np.clip(-0.05 * s.x, -3.0, 3.0))
            vel = np.array([lateral, self.v_ref])
            psi_c = track_line(s.pos, np.zeros(2), np.array([0.0, 1.0]), p.l1_m) if s.speed > 15 * KT else into_wind
            vz = max(vz_ff + k_h * (self.h_ref - s.h), -self.vs)  # never exceed the scheduled descent rate
            return Command(vel, psi_c, vz)

        # HOVER
        return self._position_hold(s, k_h)

    def _position_hold(self, s: State, k_h: float) -> Command:
        p = self.p
        self.h_ref = p.hover_height_ft
        vel = p.hover_kp * (self.hover_point - s.pos)
        n = np.linalg.norm(vel)
        if n > p.hover_max_speed:
            vel *= p.hover_max_speed / n
        vz = k_h * (self.h_ref - s.h)
        vz = float(np.clip(vz, -self.vs, self.vs))
        return Command(vel, np.pi / 2, vz)
