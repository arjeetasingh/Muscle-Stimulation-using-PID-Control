"""Reference signals and closed-loop trial runner."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

import numpy as np

from .controllers import ForceController
from .muscle_model import MusclePlant, MuscleParams


def trapezoid_ref(t: float, amp: float = 6.0, rest0: float = 1.0, ramp: float = 1.5,
                  hold: float = 5.0, rest: float = 3.0, reps: int = 3) -> float:
    """Repeated contractions: rest, ramp up, hold, ramp down, rest.  Output in N."""
    period = ramp + hold + ramp + rest
    tt = t - rest0
    if tt < 0 or tt >= reps * period:
        return 0.0
    tt = tt % period
    if tt < ramp:
        return amp * tt / ramp
    if tt < ramp + hold:
        return amp
    if tt < 2 * ramp + hold:
        return amp * (1.0 - (tt - ramp - hold) / ramp)
    return 0.0


def trapezoid_hold_windows(rest0=1.0, ramp=1.5, hold=5.0, rest=3.0, reps=3):
    """(start, end) of every hold plateau, in seconds."""
    period = ramp + hold + ramp + rest
    return [(rest0 + k * period + ramp, rest0 + k * period + ramp + hold) for k in range(reps)]


def long_hold_ref(t: float, amp: float = 6.0, rest0: float = 1.0, ramp: float = 1.5,
                  hold: float = 60.0) -> float:
    """One long sustained contraction used to expose fatigue."""
    tt = t - rest0
    if tt < 0:
        return 0.0
    if tt < ramp:
        return amp * tt / ramp
    if tt < ramp + hold:
        return amp
    return 0.0


@dataclass
class Trial:
    t: np.ndarray
    ref: np.ndarray
    force_true: np.ndarray
    force_meas: np.ndarray
    u: np.ndarray
    v: np.ndarray
    fatigue: np.ndarray


def run_trial(controller: ForceController, ref_fn: Callable[[float], float], duration: float,
              params: MuscleParams, dt: float = 0.001, seed: int = 0) -> Trial:
    """Closed-loop simulation. Plant steps every dt, controller every controller.ts."""
    plant = MusclePlant(params, dt=dt, seed=seed)
    n = int(round(duration / dt))
    n_ctrl = int(round(controller.ts / dt))
    controller.reset()
    t = np.arange(n) * dt
    ref = np.zeros(n)
    ft = np.zeros(n)
    fm = np.zeros(n)
    u_log = np.zeros(n)
    v_log = np.zeros(n)
    fat = np.zeros(n)
    u = 0.0
    meas = 0.0
    for k in range(n):
        ref[k] = ref_fn(t[k])
        if k % n_ctrl == 0:
            meas = plant.measure(ft[k - 1] if k > 0 else 0.0)
            u = controller.update(ref[k], meas)
        ft[k] = plant.step(u)
        fm[k] = meas
        u_log[k] = u
        v_log[k] = controller.v
        fat[k] = plant.fatigue
    return Trial(t, ref, ft, fm, u_log, v_log, fat)
