"""helisim: a small, open helicopter simulator for AFCS outer-loop modes.

Reproduces the Mark-on-Target automatic approach to hover
(Mark -> Approach 1 -> Approach 2 -> Hover) with a point-mass model,
wind and Dryden turbulence driven by a Wiener process.
"""
from .guidance import MarkOnTarget, Phase
from .params import HelicopterParams, MOTParams, SimConfig
from .sim import Trajectory, run_mark_on_target

__all__ = [
    "HelicopterParams",
    "MOTParams",
    "MarkOnTarget",
    "Phase",
    "SimConfig",
    "Trajectory",
    "run_mark_on_target",
]
__version__ = "0.1.0"
