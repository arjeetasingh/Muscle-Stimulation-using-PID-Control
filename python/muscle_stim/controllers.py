"""Force controller: PID in the force-demand domain + inverse recruitment.

Signal chain used in simulation AND in the ESP32 firmware (firmware/muscle_stim_pid/pid_controller.h
is a line-by-line C++ twin of this file; firmware/test checks that both give the same numbers):

    F_ref (N) ---+--> (+) --> v_unsat --> [clamp] --> v (N) --> [inverse recruitment] --> u --> [rate limit] --> u_cmd
                 |     ^
                 |     +-- P + I + D terms from the error e = F_ref - F_meas
    F_meas (N) --+

* v is a *force demand* in newtons.  The inverse recruitment curve turns v into the
  stimulation intensity u (0..1) that a fresh, nominal muscle needs to give that force.
  This cancels the S-shaped nonlinearity, so the PID sees an almost linear plant
  with gain ~1 N/N, and the same gains work at low and high force.
* The reference is fed forward (v = F_ref + corrections), so the PID only has to
  remove what the model gets wrong: fatigue, electrode shift, model error.
* Anti-windup by back-calculation: when v hits its limits the integrator is pulled back.
* The derivative acts on the measurement (no kick when the reference jumps) and is low-pass filtered.

Units:  Kp [N/N] (dimensionless), Ki [1/s], Kd [s], Ts [s], tf [s].
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class InverseRecruitment:
    """Monotone lookup table force(u) -> inverse u(force), linear interpolation.

    force_n[i] is the steady-state force (N) of a fresh muscle at intensity u_grid[i].
    """

    def __init__(self, u_grid, force_n):
        u = np.asarray(u_grid, dtype=float)
        f = np.asarray(force_n, dtype=float)
        if u.shape != f.shape or u.size < 3:
            raise ValueError("u_grid and force_n must have the same length (>= 3)")
        if np.any(np.diff(u) <= 0):
            raise ValueError("u_grid must be strictly increasing")
        # Make force strictly increasing (0.1 mN ramp) so the inverse is well defined.
        f = np.maximum.accumulate(f)
        for i in range(1, f.size):
            if f[i] <= f[i - 1]:
                f[i] = f[i - 1] + 1e-4
        self.u = u
        self.f = f

    def force_of_u(self, u: float) -> float:
        return float(np.interp(u, self.u, self.f))

    def u_of_force(self, v: float) -> float:
        return float(np.interp(v, self.f, self.u))

    @classmethod
    def linear(cls, f_full: float, n: int = 11) -> "InverseRecruitment":
        """No-nonlinearity-compensation case: u = v / f_full."""
        u = np.linspace(0.0, 1.0, n)
        return cls(u, f_full * u)


@dataclass
class PidGains:
    kp: float = 0.0   # N/N
    ki: float = 0.0   # 1/s
    kd: float = 0.0   # s
    tf: float = 0.05  # s, derivative filter time constant


class ForceController:
    """Discrete controller, call update() once every ts seconds."""

    def __init__(self, gains: PidGains, inv: InverseRecruitment, ts: float = 0.01,
                 u_max: float = 1.0, u_rate_max: float = 2.0, kaw: float | None = None,
                 ref_off: float = 0.05):
        self.ref_off = ref_off            # N, below this reference the stimulator is switched off
        self.g = gains
        self.inv = inv
        self.ts = ts
        self.u_max = min(u_max, float(inv.u[-1]))   # never beyond the identified range
        u_max = self.u_max
        self.u_rate_max = u_rate_max      # 1/s, limit on how fast u may change
        self.v_min = 0.0
        self.v_max = inv.force_of_u(u_max)
        # back-calculation rate (1/s); default makes the tracking time constant equal to Ti
        if kaw is None:
            kaw = gains.ki / gains.kp if (gains.kp > 0 and gains.ki > 0) else 1.0
        self.kaw = kaw
        self.reset()

    def reset(self, meas: float = 0.0) -> None:
        self.i_state = 0.0
        self.d_state = 0.0
        self.meas_prev = meas
        self.u_prev = 0.0
        self.v = 0.0

    def update(self, ref: float, meas: float) -> float:
        g, ts = self.g, self.ts
        if ref < self.ref_off:
            # No contraction requested: stimulation off, controller states cleared, so sensor
            # noise at rest can never switch pulses on and the integrator cannot wind up.
            self.i_state = 0.0
            self.d_state = 0.0
            self.meas_prev = meas
            self.v = 0.0
            step = self.u_rate_max * ts
            self.u_prev = max(0.0, self.u_prev - step)
            return self.u_prev
        e = ref - meas
        # derivative on measurement, backward-Euler first-order filter
        if g.kd > 0.0:
            self.d_state = (g.tf * self.d_state - g.kd * (meas - self.meas_prev)) / (g.tf + ts)
        else:
            self.d_state = 0.0
        self.meas_prev = meas

        v_unsat = ref + g.kp * e + self.i_state + self.d_state
        v = min(self.v_max, max(self.v_min, v_unsat))
        # integrator with back-calculation anti-windup
        self.i_state += ts * (g.ki * e + self.kaw * (v - v_unsat))
        self.v = v

        u = self.inv.u_of_force(v)
        step = self.u_rate_max * ts
        u = min(self.u_prev + step, max(self.u_prev - step, u))
        u = min(self.u_max, max(0.0, u))
        self.u_prev = u
        return u
