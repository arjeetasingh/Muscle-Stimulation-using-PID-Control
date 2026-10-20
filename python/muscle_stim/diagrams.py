"""Block diagrams of the control loop and of the stimulator stage (matplotlib, monochrome, serif).

    python -m muscle_stim.diagrams          # writes docs/figures/fig_block_diagram.png, fig_stimulator.png
"""

from __future__ import annotations

import argparse
import os

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import FancyBboxPatch, Circle  # noqa: E402

from .plotting import use_style  # noqa: E402


def _box(ax, x, y, w, h, text, fs=10, lw=1.2, ls="-", bold=False):
    ax.add_patch(FancyBboxPatch((x, y), w, h, boxstyle="round,pad=0.0,rounding_size=0.05",
                                fc="white", ec="black", lw=lw, ls=ls))
    ax.text(x + w / 2, y + h / 2, text, ha="center", va="center", fontsize=fs,
            fontweight="bold" if bold else "normal", linespacing=1.25)


def _arrow(ax, p0, p1, text=None, dy=0.12, fs=9, ha="center", lw=1.2, ls="-", dx=0.0):
    ax.annotate("", xy=p1, xytext=p0, arrowprops=dict(arrowstyle="-|>", lw=lw, color="black", ls=ls,
                                                      shrinkA=0, shrinkB=0))
    if text:
        ax.text((p0[0] + p1[0]) / 2 + dx, (p0[1] + p1[1]) / 2 + dy, text, ha=ha, va="bottom", fontsize=fs)


def _line(ax, xs, ys, lw=1.2, ls="-"):
    ax.plot(xs, ys, color="black", lw=lw, ls=ls, solid_capstyle="butt")


def _sum(ax, x, y, r=0.17, signs=("+", "-")):
    ax.add_patch(Circle((x, y), r, fc="white", ec="black", lw=1.2))
    ax.text(x - r - 0.08, y + 0.02, signs[0], fontsize=9, ha="right", va="bottom")
    ax.text(x - 0.02, y - r - 0.08, signs[1], fontsize=9, ha="right", va="top")


def block_diagram(path):
    use_style()
    fig, ax = plt.subplots(figsize=(13, 5.6))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 5.6)
    ax.axis("off")
    ymid = 3.6
    # reference sources
    _box(ax, 0.1, 4.55, 2.5, 0.85, "Reference $F_{ref}$ [N]\nprofile (trapezoid, hold)\nor mirror mode", fs=9)
    _box(ax, 0.1, 3.4, 2.5, 0.85, "EMG, healthy limb\nenvelope, % MVC\n(blanked during pulses)", fs=9, ls="--")
    _line(ax, [1.35, 1.35], [4.25, 4.55], ls="--")
    # the loop
    _sum(ax, 3.2, ymid)
    _line(ax, [2.6, 2.6], [5.0, 5.0])
    _line(ax, [2.6, 3.2], [4.97, 4.97])
    _arrow(ax, (3.2, 4.97), (3.2, ymid + 0.17))
    _box(ax, 3.85, ymid - 0.4, 1.7, 0.8, "PID\n$K_p,\\ K_i,\\ K_d$", fs=10)
    _arrow(ax, (3.37, ymid), (3.85, ymid), "$e$ [N]", fs=9)
    _sum(ax, 6.15, ymid, signs=("+", "+"))
    _arrow(ax, (5.55, ymid), (5.98, ymid))
    # feedforward
    _line(ax, [3.2, 3.2], [4.97, 4.97])
    _line(ax, [3.2, 6.15], [4.97, 4.97])
    _arrow(ax, (6.15, 4.97), (6.15, ymid + 0.17), "feed-forward", dy=0.05, fs=8.5)
    _box(ax, 6.8, ymid - 0.4, 1.8, 0.8, "Inverse\nrecruitment\n(look-up table)", fs=9.5)
    _arrow(ax, (6.32, ymid), (6.8, ymid), "$v$ [N]", fs=9)
    _box(ax, 9.15, ymid - 0.4, 1.7, 0.8, "Stimulator\nbiphasic, constant\ncurrent", fs=9.5)
    _arrow(ax, (8.6, ymid), (9.15, ymid), "$u$ [0..1]", fs=9)
    _box(ax, 11.3, ymid - 0.4, 1.6, 0.8, "Muscle\n$r(u)$, delay,\n2 lags, fatigue", fs=9.5)
    _arrow(ax, (10.85, ymid), (11.3, ymid), "$I$ [mA]", fs=9)
    # feedback path
    ax.text(12.25, 3.0, "force $F$ [N]", ha="left", fontsize=9)
    _arrow(ax, (12.1, ymid - 0.4), (12.1, 2.6))
    _box(ax, 10.2, 1.8, 2.0, 0.8, "Force sensor + ADC\ncalibrated, ADC to N", fs=9)
    _arrow(ax, (10.2, 2.2), (3.2, 2.2), "$F_{meas}$ [N]", dy=0.05, fs=9)
    _line(ax, [3.2, 3.2], [2.2, ymid - 0.17])
    _arrow(ax, (3.2, 2.2), (3.2, ymid - 0.17))
    ax.text(12.1, 3.05, "", fontsize=1)
    # disturbance
    ax.text(12.1, 4.55, "disturbances:\nfatigue, electrode shift,\nmuscle variation", ha="center", fontsize=8.5, va="bottom", style="italic")
    _arrow(ax, (12.1, 4.5), (12.1, ymid + 0.4), lw=1.0, ls="--")
    # safety
    _box(ax, 3.2, 0.1, 7.0, 1.1,
         "Safety supervisor (independent of the controller, latching)\n"
         "pulse current, rail voltage, skin temperature, heart rate, force limit,\n"
         "session time, contraction budget, contact check, loop watchdog",
         fs=9, ls="--")
    _line(ax, [11.2, 11.2], [1.8, 0.65], ls="--")
    _arrow(ax, (11.2, 0.65), (10.2, 0.65), "measurements", dy=0.05, fs=8.5, ls="--")
    ax.annotate("", xy=(10.0, 3.2), xytext=(10.0, 1.2), arrowprops=dict(arrowstyle="-|>", lw=1.2, ls="--", color="black"))
    ax.text(9.9, 1.75, "enable /\nstop", ha="right", fontsize=8.5, va="center")
    ax.text(0.1, 0.6, "Single input (u), single output (F):\nSISO force control.\nHR, temperature, current and voltage\nare safety checks, not loop inputs.",
            fontsize=9, va="center")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def stimulator_diagram(path):
    use_style()
    fig, ax = plt.subplots(figsize=(13, 4.6))
    ax.set_xlim(0, 13)
    ax.set_ylim(0, 4.6)
    ax.axis("off")
    y = 2.7
    _box(ax, 0.1, y - 0.5, 1.6, 1.0, "ESP32\npulse task\n(35 Hz timing)", fs=9.5)
    _box(ax, 2.2, y - 0.5, 1.6, 1.0, "12-bit DAC\n(MCP4725)\nsets current", fs=9.5)
    _arrow(ax, (1.7, y + 0.2), (2.2, y + 0.2), "I$^2$C", dy=0.05, fs=8.5)
    _box(ax, 4.3, y - 0.5, 2.0, 1.0, "Voltage-controlled\ncurrent source\n$I = V_{DAC}/R_{sense}$", fs=9.5)
    _arrow(ax, (3.8, y), (4.3, y), "$V_{DAC}$", dy=0.05, fs=8.5)
    _box(ax, 6.8, y - 0.5, 1.6, 1.0, "H-bridge\n(phase A / B)", fs=9.5)
    _arrow(ax, (6.3, y), (6.8, y), "$I$", dy=0.05, fs=8.5)
    _box(ax, 8.9, y - 0.5, 1.5, 1.0, "DC-blocking\ncapacitor", fs=9.5)
    _arrow(ax, (8.4, y), (8.9, y))
    _box(ax, 10.9, y - 0.5, 1.9, 1.0, "Electrode pair\nskin and muscle\n(1 to 5 k$\\Omega$)", fs=9.5)
    _arrow(ax, (10.4, y), (10.9, y))
    # phase drive lines
    _line(ax, [1.1, 1.1, 7.6, 7.6], [y - 0.5, 0.95, 0.95, y - 0.5], ls="--")
    ax.text(4.3, 1.0, "PHASE_A, PHASE_B (never both on; 50 $\\mu$s dead time)", ha="center", va="bottom", fontsize=8.5)
    # supply
    _box(ax, 4.3, 3.9, 2.0, 0.6, "Boost converter\n9 V to 50-60 V", fs=9)
    _arrow(ax, (5.3, 3.9), (5.3, y + 0.5))
    _box(ax, 1.9, 3.9, 1.9, 0.6, "Two 9 V batteries\n(battery only)", fs=9)
    _arrow(ax, (3.8, 4.2), (4.3, 4.2))
    # protection
    _box(ax, 6.8, 3.9, 3.6, 0.6, "Hardware current limit (independent of software)\nfuse, kill switch, dead-man button", fs=8.5, ls="--")
    _arrow(ax, (8.6, 3.9), (8.6, y + 0.05), lw=1.0, ls="--")
    _line(ax, [8.6, 8.6], [y + 0.05, y + 0.05])
    # sensing
    _box(ax, 8.9, 0.1, 3.9, 0.95, "Sensing: shunt + amplifier (pulse current), rail divider,\nNTC (skin temperature)  to ESP32 ADC1", fs=8.5)
    _line(ax, [11.85, 11.85], [1.05, y - 0.5], ls="--")
    ax.text(0.1, 0.45, "Constant current: the stimulation does not change with skin impedance.\nTwo equal phases of opposite sign: zero net charge, no DC through the skin.\nUSB isolator between the ESP32 and a PC while electrodes are on a person.",
            fontsize=8.5, va="center")
    fig.savefig(path, dpi=200, bbox_inches="tight")
    plt.close(fig)


def main(argv=None):
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--figs", default=os.path.join(root, "docs", "figures"))
    a = ap.parse_args(argv)
    os.makedirs(a.figs, exist_ok=True)
    block_diagram(os.path.join(a.figs, "fig_block_diagram.png"))
    stimulator_diagram(os.path.join(a.figs, "fig_stimulator.png"))
    print("written to", a.figs)


if __name__ == "__main__":
    main()
