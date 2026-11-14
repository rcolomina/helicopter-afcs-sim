import numpy as np
import pytest

from helisim import Phase, SimConfig, run_mark_on_target
from helisim.params import KT
from helisim.wind import Turbulence


def _hover_errors(tr):
    m = tr.phase_mask(Phase.HOVER)
    assert m.any(), "the manoeuvre never reached the hover"
    dist = np.hypot(tr.east[m] - tr.hover_point_en[0], tr.north[m] - tr.hover_point_en[1])
    hdg_err = np.abs((tr.heading_deg[m] - tr.wind_from_deg + 180) % 360 - 180)
    return dist, hdg_err, tr.height_ft[m]


@pytest.mark.parametrize("wind_from", [0, 90, 240])
@pytest.mark.parametrize("entry", [0, 90, 180, 270])
def test_reaches_hover_into_wind(wind_from, entry):
    tr = run_mark_on_target(SimConfig(wind_from_deg=wind_from, entry_heading_deg=entry))
    dist, hdg_err, h = _hover_errors(tr)
    assert dist.max() < 3.0
    assert hdg_err.max() < 2.0
    assert np.all(np.abs(h - 50.0) < 5.0)


def test_phases_in_order():
    tr = run_mark_on_target(SimConfig())
    order = []
    for p in tr.phase:
        if not order or order[-1] != p:
            order.append(p)
    assert order == [p.value for p in Phase]


def test_hover_point_is_downwind_of_target():
    tr = run_mark_on_target(SimConfig(wind_from_deg=0.0))  # wind from the north
    assert tr.hover_point_en[1] == pytest.approx(-100.0, abs=1e-6)  # 100 m south
    assert tr.hover_point_en[0] == pytest.approx(0.0, abs=1e-6)


def test_approach_2_respects_rates():
    tr = run_mark_on_target(SimConfig())
    m = tr.phase_mask(Phase.APPROACH_2)
    dt = np.diff(tr.t[m])
    decel = -np.diff(tr.groundspeed_kt[m]) / dt  # kt/s
    fast = tr.groundspeed_kt[m][1:] > 5.0  # ignore the last metres of position capture
    assert decel[fast].max() < 1.2 * 1.25  # 1.2 kt/s schedule, small lag overshoot
    assert tr.vz_fpm[m].min() > -200 * 1.1  # 200 ft/min schedule


def test_turbulence_hover_stays_close():
    tr = run_mark_on_target(SimConfig(turbulence=True, wind_speed_kt=25, seed=7))
    dist, hdg_err, h = _hover_errors(tr)
    assert dist.max() < 10.0
    assert np.all(np.abs(h - 50.0) < 15.0)


def test_turbulence_statistics_match_dryden_sigma():
    """The exact Ornstein-Uhlenbeck update keeps the stationary variance at sigma^2."""
    turb = Turbulence(wind_speed_kt=20, enabled=True, seed=1)
    samples = np.array([turb.step(0.05, 30 * KT, 200.0).copy() for _ in range(200_000)])
    from helisim.wind import dryden_low_altitude

    su, sw, _, _ = dryden_low_altitude(200.0, 20)
    std = samples[20_000:].std(axis=0)
    assert std[0] == pytest.approx(su, rel=0.15)
    assert std[2] == pytest.approx(sw, rel=0.15)


def test_csv_export(tmp_path):
    tr = run_mark_on_target(SimConfig())
    path = tmp_path / "traj.csv"
    tr.to_csv(path)
    lines = path.read_text().splitlines()
    assert lines[0].startswith("t_s,east_m,north_m")
    assert len(lines) == len(tr.t) + 1
