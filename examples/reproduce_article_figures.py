"""Reproduce the figures used in the LinkedIn article and in this README."""
from pathlib import Path

from helisim import SimConfig, run_mark_on_target
from helisim.plotting import plot_3d, plot_ground_track, plot_profiles

out = Path(__file__).resolve().parents[1] / "docs" / "img"

# Calm-ish day, wind from the north, target marked while flying east
tr = run_mark_on_target(SimConfig(wind_from_deg=0, wind_speed_kt=15, entry_heading_deg=90))
plot_3d(tr, out / "mot_3d.png", title="Mark on Target: automatic approach to hover")
plot_3d(tr, out / "mot_3d_final.png", title="Approach 1 → Approach 2 → hover, into the wind", elev=24, azim=-72)
plot_profiles(tr, out / "mot_profiles.png")
plot_ground_track(tr, out / "mot_ground_track.png")

# Same manoeuvre in 25 kt of wind from the south-west with Dryden turbulence
tr = run_mark_on_target(SimConfig(wind_from_deg=225, wind_speed_kt=25, entry_heading_deg=0, turbulence=True, seed=4))
plot_3d(tr, out / "mot_3d_turbulence.png", title="Mark on Target with 25 kt wind and turbulence")
plot_ground_track(tr, out / "mot_ground_track_turbulence.png")
print(f"Figures written to {out}")
