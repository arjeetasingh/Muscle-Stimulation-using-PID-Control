import numpy as np
import pytest

from muscle_stim.controllers import ForceController, InverseRecruitment, PidGains
from muscle_stim.muscle_model import MusclePlant, MuscleParams, recruitment, static_force


def test_recruitment_curve_is_normalised_and_monotone():
    p = MuscleParams()
    u = np.linspace(0, 1, 101)
    r = recruitment(u, p)
    assert r[0] == pytest.approx(0.0, abs=1e-12)
    assert r[-1] == pytest.approx(1.0, abs=1e-12)
    assert np.all(np.diff(r) > 0)


def test_plant_reaches_static_force_without_fatigue():
    p = MuscleParams(fatigue_enabled=False, noise_std=0.0)
    plant = MusclePlant(p, dt=0.001)
    for _ in range(3000):
        f = plant.step(0.6)
    assert f == pytest.approx(float(static_force(0.6, p)), rel=1e-3)


def test_dead_time_is_respected():
    p = MuscleParams(fatigue_enabled=False, noise_std=0.0, delay=0.040)
    plant = MusclePlant(p, dt=0.001)
    first = None
    for k in range(200):
        f = plant.step(1.0)
        if first is None and f > 1e-3:
            first = k
    assert first is not None and first >= 40        # nothing before the 40 ms delay


def test_fatigue_lowers_force_and_rest_recovers():
    p = MuscleParams(noise_std=0.0)
    plant = MusclePlant(p, dt=0.001)
    for _ in range(30000):
        plant.step(0.8)
    tired = plant.fatigue
    assert tired < 0.9
    for _ in range(60000):
        plant.step(0.0)
    assert plant.fatigue > tired


def make_inv():
    p = MuscleParams()
    u = np.linspace(0, 1, 11)
    return InverseRecruitment(u, static_force(u, p))


def test_inverse_recruitment_round_trip():
    inv = make_inv()
    for v in (0.5, 3.0, 7.7, 12.0):
        assert inv.force_of_u(inv.u_of_force(v)) == pytest.approx(v, abs=1e-6)


def test_inverse_recruitment_rejects_bad_tables():
    with pytest.raises(ValueError):
        InverseRecruitment([0, 0.5, 0.4], [0, 1, 2])


def test_controller_switches_off_at_rest_and_clears_integrator():
    c = ForceController(PidGains(kp=0.5, ki=5.0), make_inv())
    for _ in range(100):
        c.update(5.0, 0.0)
    assert c.i_state > 0
    for _ in range(100):
        u = c.update(0.0, 0.3)
    assert u == 0.0 and c.i_state == 0.0


def test_anti_windup_limits_integrator_and_recovers():
    c = ForceController(PidGains(kp=0.5, ki=5.0), make_inv())
    # reference far above what u_max can deliver, sensor sees nothing for 20 s
    for _ in range(2000):
        c.update(20.0, 0.0)
    assert abs(c.i_state) < 30.0           # without anti-windup it would reach about -2000
    # now a reachable reference; the force follows the demand through a simple lag
    meas = 0.0
    for _ in range(600):
        c.update(3.0, meas)
        meas += 0.2 * (c.v - meas)
    assert meas == pytest.approx(3.0, abs=0.3)


def test_rate_limit():
    c = ForceController(PidGains(), make_inv(), ts=0.01, u_rate_max=2.0)
    u = c.update(10.0, 0.0)
    assert u <= 2.0 * 0.01 + 1e-12


def test_closed_loop_beats_open_loop_under_mismatch_and_fatigue():
    from muscle_stim.scenarios import long_hold_ref, run_trial
    from muscle_stim import metrics
    p = MuscleParams(gain_scale=0.7)
    inv = make_inv()
    ol = run_trial(ForceController(PidGains(), inv), long_hold_ref, 30.0, p, seed=0)
    pi = run_trial(ForceController(PidGains(kp=0.54, ki=4.8), inv), long_hold_ref, 30.0, p, seed=0)
    sel = (ol.t > 6) & (ol.t < 28)
    assert metrics.rmse(pi.ref[sel], pi.force_true[sel]) < 0.3 * metrics.rmse(ol.ref[sel], ol.force_true[sel])
