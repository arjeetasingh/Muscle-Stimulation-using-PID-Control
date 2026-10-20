# Theory

## Control problem

One output is controlled (muscle force, N) with one manipulated input (stimulation intensity u, 0 to 1).
The loop is **single-input single-output (SISO)**. Heart rate, temperature, current and voltage are
measured to protect the person; they do not enter the controller.

## Muscle model (simulation)

Hammerstein structure: a static nonlinearity followed by linear dynamics.

* Recruitment: r(u) = (sigmoid(k (u - u50)) - sigmoid(-k u50)) / (sigmoid(k (1 - u50)) - sigmoid(-k u50)),
  k = 9, u50 = 0.45, so r(0) = 0 and r(1) = 1.
* Dynamics: dead time 40 ms, then two first-order lags (60 ms, 100 ms).
* Force F = F_max x gain x fatigue x x2, F_max = 15 N.
* Fatigue factor falls towards f_min = 0.3 with time constant 40 s while active and recovers with 150 s at rest.
* Sensor noise: 0.08 N standard deviation.

**These parameters are assumptions** typical of the NMES literature, chosen for a realistic shape. They are
not measurements of any person.

## Controller

Sample time 10 ms. Force-demand domain: v = F_ref + Kp e + I + D, with e = F_ref - F_meas, clamped to
[0, F(u_max)] and mapped to u by the inverse recruitment table. Because the table cancels the S-shaped
curve, the PID sees an almost linear plant with gain about 1 N/N, so one set of gains works at 3, 6 and 9 N.

* Reference feed-forward: the PID only corrects what the model gets wrong (fatigue, electrode shift).
* Back-calculation anti-windup, rate k_aw = Ki / Kp.
* Derivative on the measurement with a first-order filter (no kick on reference changes).
* u slew limit 2 per second (comfort, safety). Stimulation off when F_ref < 0.05 N.

Units: Kp [N/N], Ki [1/s], Kd [s], Tf [s], F [N], u [-], I [mA].

## Identification and tuning

1. Staircase (open loop): plateau forces give the static recruitment curve. Isotonic (monotone) fit,
   flat top trimmed, written as a table.
2. Step (3 to 6 N force demand): fit K exp(-theta s) / ((tau1 s + 1)(tau2 s + 1)).
   Simulated result K = 1.11, tau1 = tau2 = 0.074 s, theta = 0.056 s (R2 = 0.997).
3. SIMC rules with tau_c = 1.0 x effective dead time (0.094 s): PI Kp = 0.536, Ki = 4.80 1/s;
   PID Kp = 0.891, Ki = 5.99 1/s, Kd = 0.033 s.

## Simulation study (`python/muscle_stim/run_simulations.py`, 5 noise seeds, `data/sim/metrics.csv`)

All numbers are **simulated**. RMSE in N, effort = mean |du/dt| in 1/s.

| Scenario | Open loop | P | PI | PID |
|---|---|---|---|---|
| A nominal, 6 N trapezoids, RMSE | 0.454 | 0.304 | 0.133 | 0.120 |
| B gain x 0.7, RMSE | 1.181 | 0.908 | 0.240 | 0.207 |
| B gain x 1.3, RMSE | 1.605 | 1.020 | 0.165 | 0.136 |
| C fatigue, 60 s hold: force at end / start | 72 % | 79 % | 99.9 % | 99.9 % |
| A control effort (1/s) | 0.072 | 0.149 | 0.144 | 0.442 |

Reading the results:

* Feedback matters most under disturbance: with fatigue the open-loop force falls to 72 % of its start value,
  PI and PID hold it.
* The inverse recruitment table matters at every amplitude: RMSE of PI at 3, 6, 9 N is 0.066, 0.133, 0.213 N,
  against 0.200, 0.338, 0.487 N without it.
* PID improves RMSE only a little over PI (0.120 vs 0.133 N in A) and uses about three times the
  control effort. PI is the sensible default.
* The simulated plant and the controller share the same structure, so these results show that the method works,
  not how well it will work on a person.

## Limits

Model parameters are assumed. The firmware is host-tested and syntax-checked but not run on hardware. Real muscle
changes with electrode position, temperature, joint angle and training.
