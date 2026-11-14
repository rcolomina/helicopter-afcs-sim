"""Point-mass helicopter model with first-order response lags.

The model is deliberately simple: the AFCS inner loops (SAS, attitude) are
represented by lags and limits on velocity, heading and vertical speed. That is
enough to study outer-loop modes such as Mark on Target. It is not a rotor or
6-DOF flight model.

Frame: local wind-aligned frame, x = right of the final course, y = into the
wind (final course), h = height above the surface in feet. psi is the heading
as a math angle from +x, counter-clockwise, in radians.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .params import FT, G, KT, HelicopterParams


def wrap(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


@dataclass
class State:
    x: float
    y: float
    h: float  # ft
    psi: float  # rad
    vx: float  # m/s, ground velocity
    vy: float
    vz: float  # ft/s

    @property
    def pos(self) -> np.ndarray:
        return np.array([self.x, self.y])

    @property
    def speed(self) -> float:
        return float(np.hypot(self.vx, self.vy))


@dataclass
class Command:
    """Outer-loop command produced by a mode (guidance)."""

    vel: np.ndarray  # desired ground velocity (m/s), 2D
    psi: float  # desired heading (rad)
    vz: float  # desired vertical speed (ft/s)
    turn_rate_limit: float | None = None  # rad/s, overrides the bank limit


class PointMassHelicopter:
    def __init__(self, params: HelicopterParams, state: State):
        self.p = params
        self.s = state

    def turn_rate_limit(self) -> float:
        v = self.s.speed
        if v < self.p.forward_flight_speed_kt * KT:
            return np.radians(self.p.hover_yaw_rate_dps)
        return G * np.tan(np.radians(self.p.max_bank_deg)) / v

    def step(self, cmd: Command, gust: np.ndarray, dt: float) -> State:
        s, p = self.s, self.p
        # heading
        r_max = self.turn_rate_limit() if cmd.turn_rate_limit is None else cmd.turn_rate_limit
        r = float(np.clip(p.k_heading * wrap(cmd.psi - s.psi), -r_max, r_max))
        s.psi = wrap(s.psi + r * dt)
        # horizontal velocity (inner loop as a lag + acceleration limit)
        v = np.array([s.vx, s.vy])
        acc = (cmd.vel - v) / p.tau_vel
        n = np.linalg.norm(acc)
        if n > p.max_accel:
            acc *= p.max_accel / n
        # gusts act as disturbance accelerations (gust = [along wind, across wind, vertical], m/s);
        # an along-wind gust pushes the aircraft downwind (-y)
        acc = acc + np.array([gust[1], -gust[0]]) / p.tau_gust
        v = v + acc * dt
        s.vx, s.vy = v
        # vertical
        vz_max = p.max_vz_fpm / 60.0
        vz_c = float(np.clip(cmd.vz, -vz_max, vz_max))
        s.vz += ((vz_c - s.vz) / p.tau_h + gust[2] / FT / p.tau_gust) * dt
        # kinematics
        s.x += s.vx * dt
        s.y += s.vy * dt
        s.h = max(s.h + s.vz * dt, 0.0)
        return s
