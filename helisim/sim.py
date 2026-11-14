"""Run a Mark-on-Target manoeuvre and return the trajectory."""
from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .guidance import MarkOnTarget, Phase
from .model import PointMassHelicopter, State
from .params import KT, SimConfig
from .wind import Turbulence


@dataclass
class Trajectory:
    """Time histories. Positions are East/North (m) with the target at the origin."""

    t: np.ndarray
    east: np.ndarray
    north: np.ndarray
    height_ft: np.ndarray
    groundspeed_kt: np.ndarray
    heading_deg: np.ndarray  # true heading
    vz_fpm: np.ndarray
    phase: np.ndarray  # Phase values as strings
    hover_point_en: np.ndarray
    wind_from_deg: float
    wind_speed_kt: float

    def phase_mask(self, phase: Phase | str) -> np.ndarray:
        return self.phase == Phase(phase).value

    def to_csv(self, path: str | Path) -> None:
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "east_m", "north_m", "height_ft", "groundspeed_kt", "heading_deg", "vz_fpm", "phase"])
            for row in zip(self.t, self.east, self.north, self.height_ft, self.groundspeed_kt,
                           self.heading_deg, self.vz_fpm, self.phase):
                w.writerow([f"{row[0]:.2f}", f"{row[1]:.2f}", f"{row[2]:.2f}", f"{row[3]:.2f}",
                            f"{row[4]:.2f}", f"{row[5]:.1f}", f"{row[6]:.1f}", row[7]])


def _local_to_en(x: np.ndarray, y: np.ndarray, wind_from_deg: float) -> tuple[np.ndarray, np.ndarray]:
    """Wind-aligned frame (+y into the wind) -> East/North."""
    b = np.radians(wind_from_deg)  # bearing of the final course
    east = y * np.sin(b) + x * np.cos(b)
    north = y * np.cos(b) - x * np.sin(b)
    return east, north


def run_mark_on_target(cfg: SimConfig | None = None) -> Trajectory:
    cfg = cfg or SimConfig()
    # true heading -> local math angle (0 = +x, CCW), with +y = wind_from bearing
    rel = np.radians(cfg.entry_heading_deg - cfg.wind_from_deg)
    psi0 = np.pi / 2 - rel
    v0 = cfg.initial_speed_kt * KT
    s = State(0.0, 0.0, cfg.initial_height_ft, float(np.arctan2(np.sin(psi0), np.cos(psi0))),
              v0 * np.cos(psi0), v0 * np.sin(psi0), 0.0)
    heli = PointMassHelicopter(cfg.heli, s)
    mode = MarkOnTarget(cfg.mot, s)
    turb = Turbulence(cfg.wind_speed_kt, enabled=cfg.turbulence, seed=cfg.seed)

    rows = []
    t, t_hover = 0.0, 0.0
    while t < cfg.t_max:
        cmd = mode.update(s, cfg.dt)
        gust = turb.step(cfg.dt, s.speed + cfg.wind_speed_kt * KT, s.h)
        heli.step(cmd, gust, cfg.dt)
        t += cfg.dt
        hdg = (cfg.wind_from_deg + 90.0 - np.degrees(s.psi)) % 360.0
        rows.append((t, s.x, s.y, s.h, s.speed / KT, hdg, s.vz * 60.0, mode.phase.value))
        if mode.phase is Phase.HOVER:
            t_hover += cfg.dt
            if t_hover >= cfg.hover_time_s:
                break

    a = list(zip(*rows))
    east, north = _local_to_en(np.array(a[1]), np.array(a[2]), cfg.wind_from_deg)
    he, hn = _local_to_en(np.array([mode.hover_point[0]]), np.array([mode.hover_point[1]]), cfg.wind_from_deg)
    return Trajectory(
        t=np.array(a[0]), east=east, north=north, height_ft=np.array(a[3]),
        groundspeed_kt=np.array(a[4]), heading_deg=np.array(a[5]), vz_fpm=np.array(a[6]),
        phase=np.array(a[7]), hover_point_en=np.array([he[0], hn[0]]),
        wind_from_deg=cfg.wind_from_deg, wind_speed_kt=cfg.wind_speed_kt,
    )
