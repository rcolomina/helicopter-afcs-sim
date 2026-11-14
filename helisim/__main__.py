"""Command line: python -m helisim --wind-from 0 --wind-speed 15 --entry-heading 90 --turbulence"""
from __future__ import annotations

import argparse
from pathlib import Path

from .guidance import Phase
from .params import SimConfig
from .plotting import plot_3d, plot_ground_track, plot_profiles
from .sim import run_mark_on_target


def main(argv: list[str] | None = None) -> None:
    ap = argparse.ArgumentParser(description="Simulate a Mark-on-Target automatic approach to hover.")
    ap.add_argument("--wind-from", type=float, default=0.0, help="wind direction, deg true (FROM)")
    ap.add_argument("--wind-speed", type=float, default=15.0, help="wind speed, kt")
    ap.add_argument("--entry-heading", type=float, default=90.0, help="true heading when the target is marked")
    ap.add_argument("--speed", type=float, default=90.0, help="groundspeed at the mark, kt")
    ap.add_argument("--height", type=float, default=500.0, help="height at the mark, ft")
    ap.add_argument("--turbulence", action="store_true", help="add Dryden turbulence")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", type=Path, default=Path("out"))
    a = ap.parse_args(argv)

    cfg = SimConfig(wind_from_deg=a.wind_from, wind_speed_kt=a.wind_speed, entry_heading_deg=a.entry_heading,
                    initial_speed_kt=a.speed, initial_height_ft=a.height, turbulence=a.turbulence, seed=a.seed)
    tr = run_mark_on_target(cfg)
    a.out.mkdir(parents=True, exist_ok=True)
    tr.to_csv(a.out / "trajectory.csv")
    plot_3d(tr, a.out / "trajectory_3d.png")
    plot_profiles(tr, a.out / "profiles.png")
    plot_ground_track(tr, a.out / "ground_track.png")
    for ph in Phase:
        m = tr.phase_mask(ph)
        if m.any():
            print(f"{ph.value:<11} t={tr.t[m][0]:6.1f}-{tr.t[m][-1]:6.1f} s  "
                  f"h={tr.height_ft[m][0]:5.0f}->{tr.height_ft[m][-1]:4.0f} ft  "
                  f"V={tr.groundspeed_kt[m][0]:4.0f}->{tr.groundspeed_kt[m][-1]:4.0f} kt")
    print(f"Outputs written to {a.out.resolve()}")


if __name__ == "__main__":
    main()
