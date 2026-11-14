"""Generate the computed figures of the maths and physics guide.

Run from the repository root:  python docs/guide/make_figures.py
The figures are written to docs/guide/figures/ and used by helisim_math_guide.tex.
"""
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from helisim import Phase, SimConfig, run_mark_on_target
from helisim.params import FT, KT, HelicopterParams, MOTParams
from helisim.wind import Turbulence, dryden_low_altitude

OUT = Path(__file__).resolve().parent / "figures"
OUT.mkdir(exist_ok=True)
plt.rcParams.update({"font.size": 9, "axes.grid": True, "grid.alpha": 0.3, "figure.dpi": 150,
                     "axes.spines.top": False, "axes.spines.right": False})
C = ["#1f4e79", "#c0504d", "#4f8f3a", "#7f6000"]


def save(fig, name):
    fig.tight_layout()
    fig.savefig(OUT / name)
    plt.close(fig)


def dryden_params():
    h = np.linspace(10, 1000, 300)
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 2.8))
    for w, ls in ((15, "-"), (25, "--")):
        v = np.array([dryden_low_altitude(x, w) for x in h])
        ax[0].plot(h, v[:, 0], ls, color=C[0], label=rf"$\sigma_u=\sigma_v$, {w} kt")
        ax[0].plot(h, v[:, 1], ls, color=C[1], label=rf"$\sigma_w$, {w} kt")
    ax[0].set(xlabel="height (ft)", ylabel="gust standard deviation (m/s)")
    ax[0].legend(fontsize=7)
    ax[1].plot(h, v[:, 2], color=C[0], label=r"$L_u=L_v$")
    ax[1].plot(h, v[:, 3], color=C[1], label=r"$L_w$")
    ax[1].set(xlabel="height (ft)", ylabel="scale length (m)")
    ax[1].legend(fontsize=7)
    save(fig, "dryden_params.pdf")


def ou_sample():
    dt, n, h, w, v = 0.05, 200_000, 200.0, 25.0, 40.0
    turb = Turbulence(w, seed=1)
    g = np.array([turb.step(dt, v, h).copy() for _ in range(n)])
    su, sw, lu, lw = dryden_low_altitude(h, w)
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 2.8))
    t = np.arange(2400) * dt
    ax[0].plot(t, g[:2400, 0], color=C[0], lw=0.8, label="$g_u$")
    ax[0].plot(t, g[:2400, 2], color=C[1], lw=0.8, label="$g_w$")
    ax[0].axhline(su, color=C[0], ls=":", lw=0.8)
    ax[0].axhline(-su, color=C[0], ls=":", lw=0.8)
    ax[0].set(xlabel="time (s)", ylabel="gust (m/s)")
    ax[0].legend(fontsize=7, ncol=2)
    lags = np.arange(0, 400, 8)
    for k, (s, length, lab) in enumerate(((su, lu, "$g_u$"), (sw, lw, "$g_w$"))):
        x = g[:, 0 if k == 0 else 2]
        r = [np.mean(x[: n - m] * x[m:]) for m in lags]
        ax[1].plot(lags * dt, r, "o", ms=2.5, color=C[k], label=f"{lab} simulated")
        ax[1].plot(lags * dt, s**2 * np.exp(-v * lags * dt / length), color=C[k], lw=1,
                   label=rf"{lab}: $\sigma^2 e^{{-V\tau/L}}$")
    ax[1].set(xlabel=r"lag $\tau$ (s)", ylabel=r"autocovariance (m$^2$/s$^2$)")
    ax[1].legend(fontsize=7)
    save(fig, "ou_sample.pdf")


def step_responses():
    hp, mp = HelicopterParams(), MOTParams()
    dt, t = 0.01, np.arange(0, 12, 0.01)

    def second_order(k, tau):
        x = v = 0.0
        out = []
        for _ in t:
            out.append(x)
            v += (k * (1.0 - x) - v) / tau * dt
            x += v * dt
        return np.array(out)

    fig, ax = plt.subplots(figsize=(4.6, 2.8))
    ax.plot(t, 1 - np.exp(-hp.k_heading * t), color=C[0], label=r"heading, $k_\psi=1.5$ (unsaturated)")
    ax.plot(t, second_order(0.5, hp.tau_h), color=C[1], label=r"height, $k_h=0.5$, $\tau_h=1$")
    ax.plot(t, second_order(mp.hover_kp, hp.tau_vel), color=C[2], label=r"hover position, $k_p=0.35$, $\tau_v=1$")
    ax.plot(t, second_order(0.05, hp.tau_vel), color=C[3], label=r"final lateral, $k_y=0.05$, $\tau_v=1$")
    ax.axhline(1, color="k", lw=0.5)
    ax.set(xlabel="time (s)", ylabel="unit step response")
    ax.legend(fontsize=7)
    save(fig, "step_responses.pdf")


def l1_capture():
    l1, v, dt = MOTParams().l1_m, 60 * KT, 0.05
    t = np.arange(0, 80, dt)
    fig, ax = plt.subplots(figsize=(4.6, 2.8))
    for k, e0 in enumerate((100, 400, 1000)):
        e, out = float(e0), []
        for _ in t:
            out.append(e)
            e += -v * e / np.hypot(l1, e) * dt
        ax.plot(t, out, color=C[k], label=f"$e_0$ = {e0} m")
        ax.plot(t, e0 * np.exp(-v * t / l1), ":", color=C[k], lw=0.8)
    ax.set(xlabel="time (s)", ylabel="cross-track error $e$ (m)")
    ax.legend(fontsize=7, title="solid: exact, dotted: linearised", title_fontsize=7)
    save(fig, "l1_capture.pdf")


def approach2_profile():
    cfg = SimConfig()
    tr = run_mark_on_target(cfg)
    m = tr.phase_mask(Phase.APPROACH_2)
    a = cfg.mot.decel_kt_s * KT
    d = np.hypot(tr.east[m] - tr.hover_point_en[0], tr.north[m] - tr.hover_point_en[1])
    fig, ax = plt.subplots(1, 2, figsize=(7.2, 2.8))
    dd = np.linspace(0, d.max(), 300)
    ax[0].plot(dd, np.minimum(np.sqrt(2 * a * dd) / KT, cfg.mot.gate_speed_kt), "--", color=C[1],
               label=r"ideal $\min(V_g,\sqrt{2ad})$")
    ax[0].plot(d, tr.groundspeed_kt[m], color=C[0], label="simulated")
    ax[0].invert_xaxis()
    ax[0].set(xlabel="distance to the hover point (m)", ylabel="groundspeed (kt)")
    ax[0].legend(fontsize=7)
    t = tr.t[m] - tr.t[m][0]
    ax[1].plot(t, tr.height_ft[m], color=C[0], label="height (ft)")
    ax[1].set(xlabel="time in Approach 2 (s)", ylabel="height (ft)")
    ax2 = ax[1].twinx()
    ax2.plot(t, tr.groundspeed_kt[m], color=C[1], label="groundspeed (kt)")
    ax2.set_ylabel("groundspeed (kt)")
    ax2.grid(False)
    ax2.spines["right"].set_visible(True)
    h1, l1 = ax[1].get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax[1].legend(h1 + h2, l1 + l2, fontsize=7, loc="upper right")
    save(fig, "approach2_profile.pdf")


if __name__ == "__main__":
    dryden_params()
    ou_sample()
    step_responses()
    l1_capture()
    approach2_profile()
    print(f"Figures written to {OUT}")
