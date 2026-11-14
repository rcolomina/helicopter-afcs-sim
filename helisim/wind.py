"""Atmospheric turbulence driven by a Wiener process.

Each gust component g follows a first-order Gauss-Markov process, the low-order
form of the Dryden model in MIL-F-8785C (low altitude):

    dg = -(V / L) g dt + sigma * sqrt(2 V / L) dW

where W(t) is a standard Wiener process. It is integrated with the exact
discretisation of the Ornstein-Uhlenbeck process, so it is stable for any dt.
"""
from __future__ import annotations

import numpy as np

from .params import FT, KT


def dryden_low_altitude(height_ft: float, wind20_kt: float) -> tuple[float, float, float, float]:
    """Return (sigma_u, sigma_w, L_u, L_w) in metres and m/s, MIL-F-8785C low altitude."""
    h = float(np.clip(height_ft, 10.0, 1000.0))
    w20 = wind20_kt * KT / FT  # ft/s
    sigma_w = 0.1 * w20
    sigma_u = sigma_w / (0.177 + 0.000823 * h) ** 0.4
    l_w = h
    l_u = h / (0.177 + 0.000823 * h) ** 1.2
    return sigma_u * FT, sigma_w * FT, l_u * FT, l_w * FT


class Turbulence:
    """Gusts along wind (u), across wind (v) and vertical (w), in m/s."""

    def __init__(self, wind_speed_kt: float, enabled: bool = True, seed: int | None = 0):
        self.wind_speed_kt = wind_speed_kt
        self.enabled = enabled and wind_speed_kt > 0
        self.rng = np.random.default_rng(seed)
        self.g = np.zeros(3)

    def step(self, dt: float, airspeed: float, height_ft: float) -> np.ndarray:
        if not self.enabled:
            return self.g
        su, sw, lu, lw = dryden_low_altitude(height_ft, self.wind_speed_kt)
        v = max(airspeed, 5.0)
        sig = np.array([su, su, sw])
        length = np.array([lu, lu, lw])
        a = np.exp(-v * dt / length)
        # exact OU update: g_{k+1} = a g_k + sigma sqrt(1 - a^2) N(0, 1)
        self.g = a * self.g + sig * np.sqrt(1.0 - a**2) * self.rng.standard_normal(3)
        return self.g
