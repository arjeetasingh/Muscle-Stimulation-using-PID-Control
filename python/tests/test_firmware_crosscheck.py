"""The C++ controller in firmware/ must give the same output as the Python controller used in the simulations."""
import os
import re
import shutil
import subprocess

import numpy as np
import pytest

from muscle_stim.controllers import ForceController, InverseRecruitment, PidGains

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
FW = os.path.join(ROOT, "firmware")
CFG = os.path.join(FW, "muscle_stim_pid", "config.h")


def _array(text, name):
    m = re.search(name + r"\[\w+\]\s*=\s*\{([^}]*)\}", text)
    assert m, f"{name} not found in config.h"
    return [float(x.strip().rstrip("f")) for x in m.group(1).split(",") if x.strip()]


def _const(text, name):
    m = re.search(name + r"\s*=\s*([0-9.eE+-]+)f?\s*;", text)
    assert m, f"{name} not found"
    return float(m.group(1))


@pytest.fixture(scope="module")
def trace_exe(tmp_path_factory):
    if shutil.which("g++") is None:
        pytest.skip("g++ not available")
    out = str(tmp_path_factory.mktemp("fw") / "controller_trace")
    subprocess.run(["g++", "-std=c++17", "-O1", "-I", os.path.join(FW, "muscle_stim_pid"),
                    os.path.join(FW, "test", "controller_trace.cpp"), "-o", out], check=True)
    return out


def _inputs(n=3000, seed=3):
    rng = np.random.default_rng(seed)
    t = np.arange(n) * 0.01
    ref = np.where((t > 1) & (t < 12), 4.0, 0.0) + np.where((t > 14) & (t < 22), 9.0, 0.0)  # 9 N saturates
    ref = ref + np.where((t > 24) & (t < 28), 2.0 * (t - 24) / 4, 0.0)
    meas = np.clip(0.8 * ref + rng.normal(0, 0.1, n), 0, None)
    return ref, meas


@pytest.mark.parametrize("gains", [(0.53, 4.8, 0.0, 0.02), (0.9, 6.0, 0.05, 0.02)])
def test_cpp_matches_python(trace_exe, gains):
    cfg = open(CFG).read()
    inv = InverseRecruitment(_array(cfg, "RECRUIT_U"), _array(cfg, "RECRUIT_F_N"))
    ts = _const(cfg, "CONTROL_TS_S")
    rate = _const(cfg, "U_RATE_MAX_PER_S")
    off = _const(cfg, "REF_OFF_N")
    kp, ki, kd, tf = gains
    py = ForceController(PidGains(kp, ki, kd, tf), inv, ts=ts, u_max=1.0, u_rate_max=rate, ref_off=off)
    ref, meas = _inputs()
    u_py = np.array([py.update(r, m) for r, m in zip(ref, meas)])
    stdin = "\n".join(f"{r:.9f} {m:.9f}" for r, m in zip(ref, meas)) + "\n"
    res = subprocess.run([trace_exe, *map(str, gains)], input=stdin, text=True, capture_output=True, check=True)
    u_cpp = np.array([float(x) for x in res.stdout.split()])
    assert u_cpp.size == u_py.size
    assert np.max(np.abs(u_cpp - u_py)) < 1e-3
    assert u_py.max() > 0.3   # the test really exercises the controller
