"""Original schematic of an MH-60 style AFCS/SAS control panel (not to scale), numbered 1-12."""
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Circle, FancyBboxPatch  # noqa: E402

BG, PANEL, BTN, TXT, MUTE, NUM = "#0e1117", "#2b2f36", "#3a3f48", "#f0efec", "#8b8a85", "#e0703f"
fig, ax = plt.subplots(figsize=(12, 6.6), dpi=160)
fig.patch.set_facecolor(BG)
ax.set_facecolor(BG)
ax.set_xlim(0, 12)
ax.set_ylim(0, 6.6)
ax.axis("off")

ax.add_patch(FancyBboxPatch((0.4, 0.5), 11.2, 5.4, boxstyle="round,pad=0.02,rounding_size=0.25",
                            fc=PANEL, ec="#4a4f58", lw=2))
for x, y in [(0.7, 0.8), (11.3, 0.8), (0.7, 5.6), (11.3, 5.6)]:
    ax.add_patch(Circle((x, y), 0.12, fc="#1d2026", ec="#4a4f58", lw=1.5))
ax.text(6, 5.55, "AFCS / SAS CONTROL PANEL", ha="center", va="center", color=TXT, fontsize=14,
        fontweight="bold", family="DejaVu Sans")


def button(n, x, y, label, w=1.55, h=0.95, inop=False):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle="round,pad=0.02,rounding_size=0.08",
                                fc="#24272d" if inop else BTN, ec="#5a606b", lw=1.5))
    ax.text(x, y, label, ha="center", va="center", color=MUTE if inop else TXT, fontsize=9.5,
            fontweight="bold", linespacing=1.15)
    if n:
        ax.add_patch(Circle((x - w / 2 + 0.05, y + h / 2 - 0.02), 0.19, fc=NUM, ec="none", zorder=5))
        ax.text(x - w / 2 + 0.05, y + h / 2 - 0.02, str(n), ha="center", va="center", color="white",
                fontsize=9.5, fontweight="bold", zorder=6)


def knob(n, x, y, label, unit):
    ax.add_patch(Circle((x, y), 0.5, fc="#1d2026", ec="#5a606b", lw=2))
    ax.add_patch(Circle((x, y), 0.32, fc=BTN, ec="#6b7280", lw=1.5))
    ax.plot([x, x + 0.22], [y, y + 0.22], color=TXT, lw=2.5, solid_capstyle="round")
    ax.text(x, y - 0.8, label, ha="center", va="center", color=TXT, fontsize=9.5, fontweight="bold")
    ax.text(x, y - 1.08, unit, ha="center", va="center", color=MUTE, fontsize=8.5)
    ax.add_patch(Circle((x - 0.5, y + 0.45), 0.19, fc=NUM, ec="none", zorder=5))
    ax.text(x - 0.5, y + 0.45, str(n), ha="center", va="center", color="white", fontsize=9.5,
            fontweight="bold", zorder=6)


# row 1: stability and autopilot
for n, x, lab in [(1, 1.6, "SAS 1"), (2, 3.4, "SAS 2"), (3, 5.2, "TRIM"), (4, 7.0, "AUTO\nPILOT")]:
    button(n, x, 4.55, lab)
button(None, 9.1, 4.55, "SAS HYD\nBOOST", inop=True)
button(None, 10.75, 4.55, "CMPTR PWR\n/RESET", w=1.45, inop=True)

# row 2: coupled modes
for n, x, lab in [(5, 1.6, "CREW\nHOVER"), (6, 3.4, "APPR /\nHVR"), (7, 5.2, "RDR ALT\nHOLD"),
                  (8, 7.0, "BAR ALT\nHOLD"), (12, 8.8, "DEPART")]:
    button(n, x, 3.15, lab)
button(None, 10.6, 3.15, "FAIL ADVSY\nRESET 1-3\nACKNL", w=1.75, h=1.05, inop=True)

# row 3: approach / hover references
knob(9, 2.4, 1.95, "HVR ALT", "ft above ground")
knob(10, 5.0, 1.95, "LONG VEL", "knots, fwd / aft")
knob(11, 7.6, 1.95, "LAT VEL", "knots, left / right")
ax.text(10.6, 1.35, "Schematic redrawn for\nillustration, not to scale.\nGrey: power, boost\nand advisory functions.", ha="center",
        va="center", color=MUTE, fontsize=8.5, style="italic")

fig.savefig("/home/claude/helicopter-afcs-sim/docs/img/afcs_panel.png", bbox_inches="tight", facecolor=BG)
print("ok")
