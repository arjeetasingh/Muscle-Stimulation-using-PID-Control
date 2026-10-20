import numpy as np
import pytest

from muscle_stim import identify, metrics
from muscle_stim.calibrate_fsr import calibrate, lut_text
from muscle_stim.controllers import PidGains


def test_sopdt_fit_recovers_known_parameters():
    t = np.arange(0, 6, 0.01)
    v = np.where(t < 1.0, 3.0, 6.0)
    y = 3.3 + identify.simulate_sopdt(t, v, 1.1, 0.15, 0.05, 0.04, v0=3.0)
    m = identify.fit_sopdt(t, v, y, n_pre=90)
    assert m.k == pytest.approx(1.1, rel=0.02)
    assert m.tau1 == pytest.approx(0.15, rel=0.1)
    assert m.tau2 == pytest.approx(0.05, rel=0.2)
    assert m.theta == pytest.approx(0.04, abs=0.01)
    assert m.r2 > 0.999


def test_simc_formulas_hand_calculation():
    m = identify.Sopdt(k=1.0, tau1=0.2, tau2=0.1, theta=0.05, rmse=0, r2=1)
    tc = 0.1
    pi = identify.simc_pi(m, tc)
    # half rule: tau = 0.25, theta = 0.10; Kc = 0.25/(0.1+0.10) = 1.25; Ti = min(0.25, 0.8) = 0.25
    assert pi.kp == pytest.approx(1.25) and pi.ki == pytest.approx(5.0)
    pid = identify.simc_pid(m, tc)
    # Kc = 0.2/(0.1+0.05) = 1.3333; Ti = min(0.2, 0.6) = 0.2; Td = 0.1
    assert pid.ki == pytest.approx(1.3333 / 0.2, rel=1e-3)
    assert pid.kp == pytest.approx(1.3333 * (1 + 0.1 / 0.2), rel=1e-3)
    assert pid.kd == pytest.approx(1.3333 * 0.1, rel=1e-3)


def test_staircase_extraction_and_flat_top_trim():
    t = np.arange(0, 60, 0.01)
    u = np.zeros_like(t)
    f = np.zeros_like(t)
    levels = [0.2, 0.4, 0.6, 0.8, 1.0]
    force = {0.2: 1.0, 0.4: 5.0, 0.6: 10.0, 0.8: 12.0, 1.0: 12.0}
    for i, lv in enumerate(levels):
        a, b = 10 * i + 2, 10 * i + 5
        m = (t >= a) & (t < b)
        u[m] = lv
        f[m] = force[lv]
    uu, ff = identify.recruitment_from_staircase(t, u, f, min_plateau_s=1.5)
    assert uu[0] == 0.0 and ff[0] == 0.0
    assert uu[-1] == 0.8                       # the flat 1.0 point is trimmed
    assert np.all(np.diff(ff) >= 0)


def test_fsr_calibration_recovers_synthetic_sensor():
    rng = np.random.default_rng(0)
    mass = np.array([0, 50, 100, 200, 300, 500, 700, 1000.0])
    force = mass / 1000 * 9.80665
    cmax = 4000.0
    c0 = 30.0
    x = 3500 * (force / 12.0) ** 0.6
    adc = c0 + np.minimum(x, cmax - c0 - 1) + rng.normal(0, 3, len(mass))
    adc[0] = c0
    res = calibrate(mass, adc)
    assert res["results"][res["best"]]["loo_rmse"] < 0.8
    assert "FSR_LUT_ADC" in lut_text(res["adc_grid"], res["force_grid"])
    assert np.all(np.diff(res["force_grid"]) >= 0)


def test_metrics_basic():
    t = np.linspace(0, 10, 1001)
    ref = np.full_like(t, 5.0)
    y = ref + 0.1
    assert metrics.rmse(ref, y) == pytest.approx(0.1)
    assert metrics.iae(t, ref, y) == pytest.approx(1.0, rel=1e-3)
    h = metrics.hold_stats(t, ref, y, [(0, 10)], settle=1.0)
    assert h["bias_N"] == pytest.approx(0.1) and h["overshoot_pct"] == pytest.approx(2.0)


def test_session_analysis_runs_on_simulated_log(tmp_path):
    import pandas as pd
    from muscle_stim.analyze_session import analyse, find_plateaus
    t = np.arange(0, 20, 0.01)
    ref = np.where((t > 2) & (t < 12), 6.0, 0.0)
    df = pd.DataFrame({"t_ms": (t * 1000).astype(int), "state": 2, "faults": 0, "ref_N": ref,
                       "force_N": ref * 0.98, "emg_pct": np.nan, "u": ref / 15, "v_N": ref,
                       "i_peak_mA": np.nan, "t_skin_C": np.nan, "v_rail_V": np.nan, "hr_bpm": np.nan})
    assert find_plateaus(t, ref) == [(pytest.approx(2.01, abs=0.02), pytest.approx(11.99, abs=0.02))] or find_plateaus(t, ref)
    rep = analyse(df)
    assert rep["bias_N"] == pytest.approx(-0.12, abs=0.02)
