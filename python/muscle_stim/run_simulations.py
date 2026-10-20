"""Run the complete simulation study and write figures, tables and example session logs.

    python -m muscle_stim.run_simulations --out ../data/sim --figs ../docs/figures

Everything here is SIMULATION with assumed muscle parameters (see muscle_model.py).
It shows how the controller is designed and what to expect; it is not a measurement.
Results are deterministic: all random numbers use fixed seeds.
"""

from __future__ import annotations

import argparse
import csv
import os
import re
from functools import partial

import matplotlib.pyplot as plt
import numpy as np

from . import identify, metrics
from .controllers import ForceController, InverseRecruitment, PidGains
from .muscle_model import MusclePlant, MuscleParams, static_force
from .plotting import STYLES, use_style
from .scenarios import (long_hold_ref, run_trial, trapezoid_hold_windows, trapezoid_ref)

DT = 0.001          # s, plant step
TS = 0.01           # s, controller sample time (100 Hz, same as the firmware)
TAU_C_FACTOR = 1.0  # SIMC closed-loop time constant = factor * effective dead time
N_SEEDS = 5
SESSION_COLUMNS = ["t_ms", "state", "faults", "ref_N", "force_N", "emg_pct", "u", "v_N",
                   "i_peak_mA", "t_skin_C", "v_rail_V", "hr_bpm"]


# ----------------------------------------------------------------------------- #
# identification tests on the simulated muscle
# ----------------------------------------------------------------------------- #
def staircase_test(p: MuscleParams, seed: int = 1):
    """Randomised staircase, 2 s holds, 6 s rests (short holds limit fatigue during the test)."""
    rng = np.random.default_rng(seed)
    levels = np.linspace(0.1, 1.0, 10)
    rng.shuffle(levels)
    seq = [(0.0, 6.0)]
    for lv in levels:
        seq += [(float(lv), 2.0), (0.0, 6.0)]
    plant = MusclePlant(p, DT, seed=seed)
    n_ctrl = int(round(TS / DT))
    t, u, f = [], [], []
    tt = 0.0
    k = 0
    meas = 0.0
    for lv, dur in seq:
        for _ in range(int(round(dur / DT))):
            ft = plant.step(lv)
            if k % n_ctrl == 0:
                meas = plant.measure(ft)
                t.append(tt)
                u.append(lv)
                f.append(meas)
            tt += DT
            k += 1
    return np.array(t), np.array(u), np.array(f)


def step_test(p: MuscleParams, inv: InverseRecruitment, v_lo=3.0, v_hi=6.0, t_step=2.0,
              total=8.0, seed: int = 2):
    """Open-loop force-demand step v_lo -> v_hi through the inverse recruitment table."""
    plant = MusclePlant(p, DT, seed=seed)
    n_ctrl = int(round(TS / DT))
    t, v, f, u_log = [], [], [], []
    meas = 0.0
    u = 0.0
    for k in range(int(round(total / DT))):
        tt = k * DT
        vv = v_lo if tt < t_step else v_hi
        if k % n_ctrl == 0:
            u = inv.u_of_force(vv)
        ft = plant.step(u)
        if k % n_ctrl == 0:
            meas = plant.measure(ft)
            t.append(tt)
            v.append(vv)
            f.append(meas)
            u_log.append(u)
    return np.array(t), np.array(v), np.array(f), np.array(u_log)


def write_session_csv(path, t_s, u=None, v=None, force=None, ref=None, state=2):
    n = len(t_s)
    nan = float("nan")
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(SESSION_COLUMNS)
        for i in range(n):
            w.writerow([int(round(t_s[i] * 1000)), state, 0,
                        "" if ref is None else f"{ref[i]:.4f}",
                        f"{force[i]:.4f}",
                        "", f"{u[i]:.5f}", "" if v is None else f"{v[i]:.4f}",
                        "", "", "", ""])


# ----------------------------------------------------------------------------- #
# study
# ----------------------------------------------------------------------------- #
def mean_metrics(trials, ref_kind, windows=None):
    rows = []
    for tr in trials:
        sel = (tr.t >= 4.0) & (tr.t <= 62.5) if ref_kind == "hold" else np.ones_like(tr.t, bool)
        m = {"rmse_N": metrics.rmse(tr.ref[sel], tr.force_true[sel]),
             "iae_Ns": metrics.iae(tr.t[sel], tr.ref[sel], tr.force_true[sel])}
        m.update(metrics.effort(tr.t, tr.u))
        if ref_kind == "trap":
            m.update(metrics.hold_stats(tr.t, tr.ref, tr.force_true, windows))
        else:
            m.update(metrics.fatigue_stats(tr.t, tr.ref, tr.force_true, 2.5, 62.5))
        rows.append(m)
    keys = rows[0].keys()
    out = {k: float(np.mean([r[k] for r in rows])) for k in keys}
    out["rmse_N_sd"] = float(np.std([r["rmse_N"] for r in rows]))
    return out


def run_study(out_dir: str, fig_dir: str, n_seeds: int = N_SEEDS):
    os.makedirs(out_dir, exist_ok=True)
    os.makedirs(fig_dir, exist_ok=True)
    use_style()
    P = MuscleParams()
    log = []

    # ---- 1. identification: recruitment curve and dynamics -------------------------------
    t_s, u_s, f_s = staircase_test(P)
    write_session_csv(os.path.join(out_dir, "staircase_test.csv"), t_s, u=u_s, force=f_s)
    uu, ff = identify.recruitment_from_staircase(t_s, u_s, f_s, min_plateau_s=1.5, settle_frac=0.6)
    inv = InverseRecruitment(uu, ff)
    with open(os.path.join(out_dir, "recruitment_lut.txt"), "w") as fh:
        fh.write(identify.lut_text(uu, ff))

    t_p, v_p, f_p, u_p = step_test(P, inv)
    write_session_csv(os.path.join(out_dir, "step_test.csv"), t_p, u=u_p, v=v_p, force=f_p)
    w = (t_p >= 1.0) & (t_p <= 5.0)
    model = identify.fit_sopdt(t_p[w], v_p[w], f_p[w], n_pre=100)
    theta_eff = model.theta + 0.5 * model.tau2
    tau_c = TAU_C_FACTOR * theta_eff
    g_pi = identify.simc_pi(model, tau_c)
    g_pid = identify.simc_pid(model, tau_c, ts=TS)
    g_p = PidGains(kp=g_pi.kp, ki=0.0, kd=0.0)
    log.append(f"Identified SOPDT: K={model.k:.3f}  tau1={model.tau1:.3f} s  tau2={model.tau2:.3f} s  "
               f"theta={model.theta:.3f} s  (RMSE {model.rmse:.3f} N, R2 {model.r2:.4f})")
    log.append(f"SIMC tau_c = {tau_c:.3f} s")
    log.append(f"PI : Kp={g_pi.kp:.3f}  Ki={g_pi.ki:.3f} 1/s")
    log.append(f"PID: Kp={g_pid.kp:.3f}  Ki={g_pid.ki:.3f} 1/s  Kd={g_pid.kd:.4f} s  Tf={g_pid.tf:.3f} s")

    # ablation: same PI structure but NO inverse recruitment (plain linear u = v / F_full)
    inv_lin = InverseRecruitment.linear(f_full=float(inv.f[-1]))
    t_l, v_l, f_l, u_l = step_test(P, inv_lin)
    w = (t_l >= 1.0) & (t_l <= 5.0)
    model_lin = identify.fit_sopdt(t_l[w], v_l[w], f_l[w], n_pre=100)
    g_pi_lin = identify.simc_pi(model_lin, TAU_C_FACTOR * (model_lin.theta + 0.5 * model_lin.tau2))
    log.append(f"Linear-map ablation identified at 6 N: K={model_lin.k:.3f}; PI Kp={g_pi_lin.kp:.3f} "
               f"Ki={g_pi_lin.ki:.3f}")

    controllers = {
        "Open loop": lambda: ForceController(PidGains(), inv, ts=TS),
        "P": lambda: ForceController(g_p, inv, ts=TS),
        "PI": lambda: ForceController(g_pi, inv, ts=TS),
        "PID": lambda: ForceController(g_pid, inv, ts=TS),
    }
    windows = trapezoid_hold_windows()
    rows = []
    keep = {}      # (scenario, controller) -> Trial of seed 0, for plotting and CSV

    def study(scenario, ctrl_name, factory, ref_fn, duration, params, kind, store=True):
        trials = [run_trial(factory(), ref_fn, duration, params, dt=DT, seed=s) for s in range(n_seeds)]
        m = mean_metrics(trials, kind, windows)
        rows.append({"scenario": scenario, "controller": ctrl_name, **m})
        if store:
            keep[(scenario, ctrl_name)] = trials[0]
        return trials

    # ---- 2. scenario A: nominal tracking ----------------------------------------------
    for name, fac in controllers.items():
        study("A nominal tracking", name, fac, trapezoid_ref, 36.0, P, "trap")
    # ---- 3. scenario B: electrode shift, gain -30 % and +30 % -------------------------
    for gs in (0.7, 1.3):
        for name, fac in controllers.items():
            study(f"B gain x{gs}", name, fac, trapezoid_ref, 36.0, P.with_(gain_scale=gs), "trap")
    # ---- 4. scenario C: 60 s sustained contraction (fatigue) ----------------------------
    for name, fac in controllers.items():
        study("C fatigue 60 s", name, fac, long_hold_ref, 64.0, P, "hold")
    # ---- 5. scenario D: other force levels, with and without inverse recruitment --------
    for amp in (3.0, 6.0, 9.0):
        ref = partial(trapezoid_ref, amp=amp)
        study(f"D amplitude {amp:g} N", "PI", controllers["PI"], ref, 36.0, P, "trap", store=False)
        study(f"D amplitude {amp:g} N", "PI, no inverse recruitment",
              lambda: ForceController(g_pi_lin, inv_lin, ts=TS), ref, 36.0, P, "trap", store=False)

    # ---- tables -----------------------------------------------------------------------
    cols = ["scenario", "controller", "rmse_N", "rmse_N_sd", "iae_Ns", "bias_N", "ripple_N",
            "overshoot_pct", "force_start_N", "force_end_N", "end_over_start_pct",
            "mean_abs_du_per_s", "pct_time_at_limit"]
    with open(os.path.join(out_dir, "metrics.csv"), "w", newline="") as fh:
        w_ = csv.DictWriter(fh, fieldnames=cols, extrasaction="ignore")
        w_.writeheader()
        for r in rows:
            w_.writerow({k: (f"{v:.4f}" if isinstance(v, float) else v) for k, v in r.items()})
    for (sc, nm), tr in keep.items():
        tag = re.sub(r"[^A-Za-z0-9.]+", "_", f"{sc}_{nm}").strip("_")
        with open(os.path.join(out_dir, f"trial_{tag}.csv"), "w", newline="") as fh:
            w_ = csv.writer(fh)
            w_.writerow(["t_s", "ref_N", "force_true_N", "force_meas_N", "u", "v_N", "fatigue_factor"])
            for i in range(0, len(tr.t), 10):
                w_.writerow([f"{tr.t[i]:.3f}", f"{tr.ref[i]:.4f}", f"{tr.force_true[i]:.4f}",
                             f"{tr.force_meas[i]:.4f}", f"{tr.u[i]:.5f}", f"{tr.v[i]:.4f}",
                             f"{tr.fatigue[i]:.5f}"])

    # ---- figures ----------------------------------------------------------------------
    fig_identification(fig_dir, P, t_s, u_s, f_s, uu, ff, t_p, v_p, f_p, model)
    fig_tracking(fig_dir, keep)
    fig_robustness(fig_dir, keep)
    fig_fatigue(fig_dir, keep)
    fig_amplitudes(fig_dir, rows)
    with open(os.path.join(out_dir, "study_log.txt"), "w") as fh:
        fh.write("\n".join(log) + "\n")
    print("\n".join(log))
    return rows, model, g_pi, g_pid


# ----------------------------------------------------------------------------- #
# figures
# ----------------------------------------------------------------------------- #
def _save(fig, fig_dir, name):
    fig.savefig(os.path.join(fig_dir, name), bbox_inches="tight")
    plt.close(fig)


def fig_identification(fig_dir, P, t_s, u_s, f_s, uu, ff, t_p, v_p, f_p, model):
    fig, ax = plt.subplots(1, 3, figsize=(13.5, 3.4), gridspec_kw={"width_ratios": [1.6, 1.0, 1.3]})
    a = ax[0]
    a.plot(t_s, f_s, color="0.45", lw=0.9, label="Force (N)")
    a.set_xlabel("Time (s)")
    a.set_ylabel("Force (N)")
    a2 = a.twinx()
    a2.spines["right"].set_visible(True)
    a2.plot(t_s, u_s, color="black", lw=1.0, ls=(0, (5, 3)), label="Intensity u")
    a2.set_ylabel("Stimulation intensity u (-)")
    a2.grid(False)
    a.set_title("(a) Staircase test, randomised order")
    b = ax[1]
    ug = np.linspace(0, 1, 200)
    b.plot(ug, static_force(ug, P), color="black", lw=1.4, label="Assumed muscle")
    b.plot(uu, ff, "o", mfc="white", mec="black", ms=5, label="Identified from test\n(fatigue lowers later points)")
    b.set_xlabel("Stimulation intensity u (-)")
    b.set_ylabel("Steady force (N)")
    b.set_title("(b) Recruitment curve")
    b.legend(loc="upper left")
    c = ax[2]
    w = (t_p >= 1.0) & (t_p <= 5.0)           # same window as the fit
    pre = w & (t_p < 2.0)
    sim = identify.simulate_sopdt(t_p[w], v_p[w], model.k, model.tau1, model.tau2, model.theta,
                                  v0=float(np.mean(v_p[pre])))
    c.plot(t_p, f_p, color="0.6", lw=0.9, label="Measured force")
    c.plot(t_p[w], float(np.mean(f_p[pre])) + sim, color="black", lw=1.6, label="SOPDT fit")
    c.step(t_p, v_p, where="post", color="black", lw=1.0, ls=(0, (5, 3)), label="Force demand v")
    c.set_xlabel("Time (s)")
    c.set_ylabel("Force (N)")
    c.set_title(f"(c) Step test, $\\tau_1$={model.tau1:.2f} s, $\\tau_2$={model.tau2:.2f} s, "
                f"$\\theta$={model.theta:.2f} s")
    c.legend(loc="lower right")
    fig.tight_layout()
    _save(fig, fig_dir, "fig_identification.png")


def _hold_shading(ax):
    for a, b in trapezoid_hold_windows():
        ax.axvspan(a, b, color="0.95", lw=0)


def fig_tracking(fig_dir, keep):
    sc = "A nominal tracking"
    fig, ax = plt.subplots(2, 1, figsize=(10, 5.4), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]})
    tr0 = keep[(sc, "PID")]
    _hold_shading(ax[0])
    ax[0].plot(tr0.t, tr0.ref, label="Reference", **STYLES["Reference"])
    for nm in ("Open loop", "P", "PID"):
        tr = keep[(sc, nm)]
        ax[0].plot(tr.t, tr.force_true, label=nm, **STYLES[nm])
    ax[0].set_ylabel("Force (N)")
    ax[0].set_ylim(-0.3, 8.2)
    ax[0].legend(ncol=4, loc="upper right")
    ax[0].set_title("Nominal muscle, three contractions of 6 N (simulation)")
    for nm in ("Open loop", "PID"):
        tr = keep[(sc, nm)]
        ax[1].plot(tr.t, tr.u, label=nm, **STYLES[nm])
    ax[1].set_ylabel("Intensity u (-)")
    ax[1].set_xlabel("Time (s)")
    ax[1].legend(loc="upper right", ncol=2)
    fig.tight_layout()
    _save(fig, fig_dir, "fig_tracking.png")


def fig_robustness(fig_dir, keep):
    fig, ax = plt.subplots(1, 2, figsize=(12, 3.8), sharey=True)
    for a, gs in zip(ax, (0.7, 1.3)):
        sc = f"B gain x{gs}"
        tr0 = keep[(sc, "PID")]
        _hold_shading(a)
        a.plot(tr0.t, tr0.ref, label="Reference", **STYLES["Reference"])
        for nm in ("Open loop", "PID"):
            a.plot(keep[(sc, nm)].t, keep[(sc, nm)].force_true, label=nm, **STYLES[nm])
        a.set_title(f"Electrode shift: muscle gain {int(round((gs - 1) * 100)):+d} %")
        a.set_xlabel("Time (s)")
    ax[0].set_ylabel("Force (N)")
    ax[0].legend(loc="upper right")
    fig.tight_layout()
    _save(fig, fig_dir, "fig_robustness.png")


def fig_fatigue(fig_dir, keep):
    sc = "C fatigue 60 s"
    fig, ax = plt.subplots(2, 1, figsize=(10, 5.4), sharex=True, gridspec_kw={"height_ratios": [2.2, 1]})
    tr0 = keep[(sc, "PID")]
    ax[0].plot(tr0.t, tr0.ref, label="Reference", **STYLES["Reference"])
    for nm in ("Open loop", "P", "PID"):
        ax[0].plot(keep[(sc, nm)].t, keep[(sc, nm)].force_true, label=nm, **STYLES[nm])
    ax[0].set_ylabel("Force (N)")
    ax[0].set_ylim(0, 8)
    ax[0].legend(ncol=4, loc="lower center")
    ax[0].set_title("One sustained 6 N contraction of 60 s: muscle fatigue (simulation)")
    for nm in ("Open loop", "P", "PID"):
        ax[1].plot(keep[(sc, nm)].t, keep[(sc, nm)].u, label=nm, **STYLES[nm])
    ax[1].set_ylabel("Intensity u (-)")
    ax[1].set_xlabel("Time (s)")
    ax[1].legend(loc="upper left", ncol=2)
    fig.tight_layout()
    _save(fig, fig_dir, "fig_fatigue.png")


def fig_amplitudes(fig_dir, rows):
    amps = (3.0, 6.0, 9.0)
    fig, ax = plt.subplots(1, 2, figsize=(9, 3.4))
    for nm, mk, ls in (("PI", "o", "-"), ("PI, no inverse recruitment", "s", "--")):
        r = [next(x for x in rows if x["scenario"] == f"D amplitude {a:g} N" and x["controller"] == nm)
             for a in amps]
        ax[0].plot(amps, [x["rmse_N"] for x in r], mk + ls, color="black" if nm == "PI" else "0.5",
                   mfc="white", lw=1.5, label=nm)
        ax[1].plot(amps, [x["overshoot_pct"] for x in r], mk + ls, color="black" if nm == "PI" else "0.5",
                   mfc="white", lw=1.5, label=nm)
    ax[0].set_xlabel("Contraction level (N)")
    ax[0].set_ylabel("Tracking RMSE (N)")
    ax[1].set_xlabel("Contraction level (N)")
    ax[1].set_ylabel("Overshoot in hold (%)")
    ax[0].set_xticks(amps)
    ax[1].set_xticks(amps)
    ax[0].legend(loc="upper left")
    fig.tight_layout()
    _save(fig, fig_dir, "fig_amplitudes.png")


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
    ap.add_argument("--out", default=os.path.join(root, "data", "sim"))
    ap.add_argument("--figs", default=os.path.join(root, "docs", "figures"))
    ap.add_argument("--seeds", type=int, default=N_SEEDS)
    args = ap.parse_args(argv)
    run_study(args.out, args.figs, args.seeds)


if __name__ == "__main__":
    main()
