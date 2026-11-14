"""Plots of a Mark-on-Target trajectory."""
from __future__ import annotations

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from .guidance import Phase  # noqa: E402
from .sim import Trajectory  # noqa: E402

COLORS = {
    Phase.MARK: "#d62728",
    Phase.APPROACH_1: "#1f77b4",
    Phase.APPROACH_2: "#2ca02c",
    Phase.HOVER: "#9467bd",
}
LABELS = {
    Phase.MARK: "Mark + radar altitude hold",
    Phase.APPROACH_1: "Approach 1 (pattern + course reversal)",
    Phase.APPROACH_2: "Approach 2 (final, into the wind)",
    Phase.HOVER: "Hover",
}


def _segments(tr: Trajectory):
    for ph in Phase:
        idx = np.where(tr.phase_mask(ph))[0]
        if idx.size == 0:
            continue
        idx = np.r_[max(idx[0] - 1, 0), idx]
        yield ph, idx


def _wind_vector(tr: Trajectory) -> np.ndarray:
    b = np.radians(tr.wind_from_deg)
    return -np.array([np.sin(b), np.cos(b)])  # direction the wind blows to (E, N)


def plot_3d(tr: Trajectory, path: str | Path | None = None, title: str | None = None,
            elev: float = 28, azim: float = -125):
    fig = plt.figure(figsize=(12, 8), dpi=130)
    ax = fig.add_subplot(111, projection="3d")
    for ph, idx in _segments(tr):
        ax.plot(tr.east[idx], tr.north[idx], tr.height_ft[idx], color=COLORS[ph], lw=2.6, label=LABELS[ph])
        ax.plot(tr.east[idx], tr.north[idx], 0 * idx, color=COLORS[ph], lw=0.8, alpha=0.35, ls="--")
    step = max(len(tr.t) // 25, 1)
    for i in range(0, len(tr.t), step):
        ax.plot([tr.east[i]] * 2, [tr.north[i]] * 2, [0, tr.height_ft[i]], color="grey", lw=0.5, alpha=0.35)
    ax.scatter([0], [0], [0], marker="X", s=140, color="black", label="Marked target", zorder=5)
    w = _wind_vector(tr) * 400
    x0 = tr.east.max() + 100
    y0 = tr.north.max()
    ax.quiver(x0, y0, 0, w[0], w[1], 0, color="#555", arrow_length_ratio=0.25, lw=2)
    ax.text(x0, y0, 30, f"Wind {tr.wind_speed_kt:.0f} kt", color="#555")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_zlabel("Height (ft)")
    ax.set_zlim(0, max(550, tr.height_ft.max() * 1.05))
    ax.set_title(title or "Mark on Target: automatic approach to hover", fontsize=13, pad=12)
    ax.view_init(elev=elev, azim=azim)
    ax.legend(loc="upper left", fontsize=9, frameon=False)
    fig.tight_layout()
    if path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, bbox_inches="tight")
    return fig


def plot_profiles(tr: Trajectory, path: str | Path | None = None):
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), dpi=130, sharex=True)
    hdg = np.degrees(np.unwrap(np.radians(tr.heading_deg)))  # continuous heading, no 360 -> 0 jumps
    for ph, idx in _segments(tr):
        axes[0].plot(tr.t[idx], tr.groundspeed_kt[idx], color=COLORS[ph], lw=2.2, label=LABELS[ph])
        axes[1].plot(tr.t[idx], tr.height_ft[idx], color=COLORS[ph], lw=2.2)
        axes[2].plot(tr.t[idx], hdg[idx], color=COLORS[ph], lw=2.2)
    axes[0].set_ylabel("Groundspeed (kt)")
    axes[1].set_ylabel("Height (ft)")
    axes[2].set_ylabel("Heading (deg, unwrapped)")
    axes[2].set_xlabel("Time since mark (s)")
    axes[2].axhline(hdg[-1], color="#555", ls=":", lw=1)
    axes[2].text(tr.t[0], hdg[-1], "  into the wind", va="bottom", fontsize=8, color="#555")
    for a in axes:
        a.grid(alpha=0.3)
        a.spines[["top", "right"]].set_visible(False)
    axes[0].legend(fontsize=8, frameon=False, loc="upper right")
    axes[0].set_title("Mark on Target: speed, height and heading", fontsize=12)
    fig.tight_layout()
    if path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, bbox_inches="tight")
    return fig


def plot_ground_track(tr: Trajectory, path: str | Path | None = None):
    fig, ax = plt.subplots(figsize=(8, 8), dpi=130)
    for ph, idx in _segments(tr):
        ax.plot(tr.east[idx], tr.north[idx], color=COLORS[ph], lw=2.2, label=LABELS[ph])
    ax.scatter([0], [0], marker="X", s=120, color="black", label="Marked target", zorder=5)
    ax.scatter(*tr.hover_point_en, marker="o", s=50, facecolor="none", edgecolor="#9467bd", label="Hover point")
    w = _wind_vector(tr) * 250
    ax.annotate("", xy=(w[0], w[1]), xytext=(0, 0), arrowprops=dict(arrowstyle="->", color="#555", lw=2))
    ax.set_aspect("equal")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8, frameon=False)
    ax.set_title("Ground track (arrow: wind direction)")
    fig.tight_layout()
    if path:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(path, bbox_inches="tight")
    return fig
