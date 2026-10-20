"""Plot style: white background, serif font, black/grey only (prints well, no colour needed)."""

from __future__ import annotations

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# name -> matplotlib line style kwargs (distinguishable in greyscale)
STYLES = {
    "Reference":                 dict(color="black", lw=1.1, ls=(0, (5, 3))),
    "Open loop":                 dict(color="0.55", lw=1.7, ls=":"),
    "P":                         dict(color="0.35", lw=1.3, ls="-."),
    "PI":                        dict(color="0.30", lw=1.6, ls="-"),
    "PID":                       dict(color="black", lw=1.9, ls="-"),
    "PI, no inverse recruitment": dict(color="0.55", lw=1.5, ls="--"),
}


def use_style() -> None:
    plt.rcParams.update({
        "font.family": "serif",
        "font.serif": ["Liberation Serif", "Times New Roman", "DejaVu Serif"],
        "mathtext.fontset": "stix",
        "font.size": 11,
        "axes.titlesize": 12,
        "axes.labelsize": 11,
        "axes.edgecolor": "black",
        "axes.linewidth": 0.8,
        "axes.spines.top": False,
        "axes.spines.right": False,
        "axes.grid": True,
        "grid.color": "0.88",
        "grid.linewidth": 0.6,
        "legend.frameon": False,
        "legend.fontsize": 10,
        "figure.facecolor": "white",
        "axes.facecolor": "white",
        "savefig.facecolor": "white",
        "savefig.dpi": 220,
        "figure.dpi": 100,
        "lines.solid_capstyle": "round",
    })
