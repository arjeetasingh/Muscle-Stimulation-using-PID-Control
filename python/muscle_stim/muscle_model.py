"""Hammerstein model of an electrically stimulated muscle (isometric force).

Structure (input -> output):

    u  --> [ recruitment r(u) ] --> [ delay ] --> [ two first-order lags ] --> x2
                                                                                |
                                   force = F_max * gain_scale * fatigue * x2 <--+

* u          stimulation intensity, dimensionless, 0 (off) to 1 (maximum allowed
             pulse amplitude of the stimulator)
* r(u)       static recruitment curve: S-shaped, because more motor units are
             recruited as the pulse amplitude grows, until all are recruited
* delay      electromechanical delay
* lags       excitation-contraction (activation) dynamics
* fatigue    slowly varying factor in [f_min, 1] that falls while the muscle is
             active and recovers while it rests
* gain_scale models changes of electrode placement / skin impedance between
             sessions (1.0 = nominal)

All parameter values are *typical assumed values*, NOT measurements. They are
meant to be replaced by values identified from your own step tests (see
identify.py).  Units are given in every comment.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, replace

import numpy as np


@dataclass(frozen=True)
class MuscleParams:
    f_max: float = 15.0        # N, force at u = 1, fresh muscle, nominal electrodes
    k_rec: float = 9.0         # -, steepness of the recruitment sigmoid
    u50: float = 0.45          # -, intensity at the middle of the recruitment curve
    tau1: float = 0.060        # s, first activation time constant
    tau2: float = 0.100        # s, second activation time constant
    delay: float = 0.040       # s, electromechanical delay
    gain_scale: float = 1.0    # -, 1.0 = nominal electrode placement
    fatigue_enabled: bool = True
    t_fatigue: float = 40.0    # s, fatigue time constant at full activation
    t_recover: float = 150.0   # s, recovery time constant at rest
    f_min: float = 0.30        # -, lowest value of the fatigue factor
    noise_std: float = 0.08    # N, force-sensor noise (after averaging), 1 sigma

    def with_(self, **kw) -> "MuscleParams":
        return replace(self, **kw)


def _sigmoid(x: np.ndarray | float) -> np.ndarray | float:
    return 1.0 / (1.0 + np.exp(-x))


def recruitment(u, p: MuscleParams):
    """Normalised recruitment curve, r(0) = 0 and r(1) = 1."""
    u = np.clip(u, 0.0, 1.0)
    s0 = _sigmoid(-p.k_rec * p.u50)
    s1 = _sigmoid(p.k_rec * (1.0 - p.u50))
    return (_sigmoid(p.k_rec * (u - p.u50)) - s0) / (s1 - s0)


def static_force(u, p: MuscleParams):
    """Steady-state isometric force (N) of a fresh muscle at intensity u."""
    return p.f_max * p.gain_scale * recruitment(u, p)


class MusclePlant:
    """Time-domain simulation of the muscle, advanced with a fixed step dt (s)."""

    def __init__(self, p: MuscleParams, dt: float = 0.001, seed: int | None = 0):
        self.p = p
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        n_delay = max(1, int(round(p.delay / dt)))
        self._buf = deque([0.0] * n_delay, maxlen=n_delay)
        self.x1 = 0.0
        self.x2 = 0.0
        self.fatigue = 1.0
        # exact discretisation of the first-order lags for a fixed step
        self._a1 = 1.0 - np.exp(-dt / p.tau1)
        self._a2 = 1.0 - np.exp(-dt / p.tau2)

    def step(self, u: float) -> float:
        """Advance one step with stimulation intensity u; return TRUE force (N)."""
        p = self.p
        r_in = float(recruitment(u, p))
        r_delayed = self._buf[0]
        self._buf.append(r_in)          # deque with maxlen drops the oldest value
        self.x1 += self._a1 * (r_delayed - self.x1)
        self.x2 += self._a2 * (self.x1 - self.x2)
        if p.fatigue_enabled:
            drain = self.x2 * (self.fatigue - p.f_min) / p.t_fatigue
            recover = (1.0 - self.x2) * (1.0 - self.fatigue) / p.t_recover
            self.fatigue += self.dt * (recover - drain)
            self.fatigue = min(1.0, max(p.f_min, self.fatigue))
        return p.f_max * p.gain_scale * self.fatigue * self.x2

    def measure(self, force_true: float) -> float:
        """Force-sensor reading (N): true force plus noise, never negative."""
        return max(0.0, force_true + self.rng.normal(0.0, self.p.noise_std))
