"""Parameters for the helicopter model, the Mark-on-Target mode and the simulation.

All values are illustrative and come from public material or simple engineering
judgement. They are NOT data from any real aircraft or AFCS.
"""
from __future__ import annotations

from dataclasses import dataclass, field

KT = 0.514444  # m/s per knot
FT = 0.3048  # m per foot
G = 9.80665  # m/s^2


@dataclass
class HelicopterParams:
    """Point-mass helicopter with first-order response lags."""

    tau_vel: float = 1.0  # s, horizontal velocity response lag
    max_accel: float = 3.0  # m/s^2, horizontal acceleration limit
    tau_gust: float = 3.0  # s, how strongly gusts disturb the velocity
    tau_h: float = 1.0  # s, vertical response lag
    max_vz_fpm: float = 1000.0  # ft/min, vertical speed limit
    max_bank_deg: float = 25.0  # deg, bank limit in forward flight
    hover_yaw_rate_dps: float = 15.0  # deg/s, pedal turn rate limit in hover
    k_heading: float = 1.5  # 1/s, heading loop gain
    forward_flight_speed_kt: float = 15.0  # below this, yaw like a hover


@dataclass
class MOTParams:
    """Mark-on-Target automatic approach to hover (illustrative values).

    Rates follow public Sikorsky SAR material: 1.2 kt/s deceleration and
    200 ft/min descent in Approach 1 and Approach 2, hover at 50 ft, 0 kt.
    """

    mark_hold_s: float = 6.0  # radar-altitude hold after the mark
    decel_kt_s: float = 1.2
    descent_fpm: float = 200.0
    gate_speed_kt: float = 60.0  # end of Approach 1
    gate_height_ft: float = 200.0  # end of Approach 1
    hover_height_ft: float = 50.0
    hover_offset_m: float = 100.0  # hover this far downwind of the target
    reversal_bank_deg: float = 20.0  # bank used for the course reversal
    final_margin_s: float = 15.0  # extra straight final before decelerating
    l1_m: float = 400.0  # cross-track guidance length
    hover_kp: float = 0.35  # 1/s, position hold gain
    hover_max_speed: float = 3.0  # m/s, position hold speed limit


@dataclass
class SimConfig:
    dt: float = 0.05
    t_max: float = 900.0
    hover_time_s: float = 20.0
    # initial condition when the pilot marks the target (aircraft over the target)
    initial_speed_kt: float = 90.0
    initial_height_ft: float = 500.0
    entry_heading_deg: float = 90.0  # true heading at the mark
    # wind (meteorological convention: direction the wind comes FROM)
    wind_from_deg: float = 0.0
    wind_speed_kt: float = 15.0
    turbulence: bool = False
    seed: int | None = 0
    heli: HelicopterParams = field(default_factory=HelicopterParams)
    mot: MOTParams = field(default_factory=MOTParams)
