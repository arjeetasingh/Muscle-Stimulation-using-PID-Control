# Muscle Stimulation using PID Control

Closed-loop force control of electrically stimulated muscle: an ESP32 stimulator with a force sensor,
a PID controller that works through an identified recruitment curve, a safety supervisor, a
simulation study, and the tools to calibrate and identify the real system.

Author: Shashank, M.Tech Biomedical Devices Engineering, IIT Indore.
Course project: Modelling and Control in Biomedical Devices (BSE 649).

> **Safety.** This drives current through the body. Read [SAFETY.md](SAFETY.md) before building anything.
> It is a student prototype and not a medical device.

## Status (what is and is not done)

| Part | Status |
|---|---|
| Muscle model, controller, identification, tuning, simulation study | Done, results in `data/sim` and `docs/figures` (all **simulated**) |
| FSR calibration tool, serial logger, session analysis | Done, tested on synthetic data |
| ESP32 firmware | Written. Core logic unit-tested on a PC (1203 checks), C++ controller matches the Python one, sketch compiles against stub headers. **Not run on hardware** |
| High-voltage stimulator stage | Specified by function in `docs/hardware.md`. No verified schematic |
| Measurements on a person | **None in this repository.** `data/measured/` is empty on purpose |

## The control problem

![block diagram](docs/figures/fig_block_diagram.png)

One input (stimulation intensity `u`), one output (force in N): a **SISO** loop. The controller works in the
force-demand domain: reference plus PID correction gives a demand `v` in N, and an identified inverse
recruitment table converts it to `u`. Heart rate, skin temperature, pulse current and rail voltage are
safety checks. Mirror mode sets the reference from the EMG of the healthy opposite limb (in % of its maximum
voluntary contraction).

![stimulator](docs/figures/fig_stimulator.png)

## Simulation results (simulated, 5 noise seeds)

| Scenario | Open loop | P | PI | PID |
|---|---|---|---|---|
| Nominal 6 N trapezoids, RMSE (N) | 0.454 | 0.304 | 0.133 | 0.120 |
| Plant gain x 0.7, RMSE (N) | 1.181 | 0.908 | 0.240 | 0.207 |
| Plant gain x 1.3, RMSE (N) | 1.605 | 1.020 | 0.165 | 0.136 |
| 60 s hold with fatigue, final force / initial force | 72 % | 79 % | 99.9 % | 99.9 % |

Feedback removes the effect of fatigue and gain changes. Inverse recruitment cuts the PI error by a factor of
2.3 to 3 at every amplitude. PID is only slightly better than PI and moves the output about three times more,
so PI is the default. The plant is a model with assumed parameters, see [docs/theory.md](docs/theory.md).

![tracking](docs/figures/fig_tracking.png)
![fatigue](docs/figures/fig_fatigue.png)

Poster (A1 landscape): [docs/poster/poster.pdf](docs/poster/poster.pdf), source `docs/poster/poster.html`, rebuilt with `docs/poster/build_poster.py`.

## Repository layout

```
firmware/muscle_stim_pid/   ESP32 sketch (.ino) and portable core headers (controller, safety, EMG, FSR, profiles)
firmware/test/              host unit tests, controller trace for the cross-check, stub headers, Makefile
python/muscle_stim/         model, controllers, identification, simulations, FSR calibration, logger, analysis
python/tests/               pytest suite (includes the C++ vs Python controller check)
data/sim/                   simulation outputs (metrics.csv, trial logs, identified tables)
data/measured/              put real session logs here
docs/                       theory, hardware, protocol, figures, poster
```

## Quick start

```
pip install -r requirements.txt
PYTHONPATH=python python -m pytest python/tests -q          # Python tests + C++ cross-check (needs g++)
PYTHONPATH=python python -m muscle_stim.run_simulations     # regenerates data/sim and docs/figures
PYTHONPATH=python python -m muscle_stim.diagrams            # block diagrams
cd firmware/test && make test                               # C++ unit tests and sketch syntax check
```

On Windows set `PYTHONPATH` with `set PYTHONPATH=python`. The simulation output is deterministic (fixed seeds).

## Firmware

Arduino IDE, board "ESP32 Dev Module" (Arduino-ESP32 2.x or 3.x). Open `firmware/muscle_stim_pid/muscle_stim_pid.ino`.
Install the library "SparkFun MAX3010x Pulse and Proximity Sensor Library", or set `USE_HEART_RATE 0` in `config.h`
(then heart rate is no longer a safety input). All settings and limits are in `config.h` and the compile-time
checks reject unsafe values. Commands and telemetry: [docs/protocol.md](docs/protocol.md).

Order of work: dummy-load tests, FSR calibration, staircase, step, gains, closed loop
([docs/hardware.md](docs/hardware.md)). Until `FSR_CALIBRATED` and `RECRUIT_IDENTIFIED` are set to `true` the
firmware refuses to arm and to run closed loop.

## What was corrected from the first version of this project

The first version (a course poster and a half-built prototype) was a good idea with several errors. This
version fixes them.

| First version | Problem | Now |
|---|---|---|
| "MIMO" control | One quantity (force) is controlled with one input | SISO, the other sensors are safety monitors |
| Brain/EMG signal and FSR force compared directly | Different physical quantities and units | EMG normalised to % MVC, force in calibrated N, linked by an explicit scale |
| FSR described as muscle force | An FSR reads force on its pad only | Rigid mounted pad, calibrated against known masses, leave-one-out error reported |
| Stimulation by PWM, about 490 Hz, one polarity | Net DC through the skin (electrolysis, burns), frequency too high, current depends on skin impedance | Biphasic, charge balanced, 35 Hz, 300 us per phase, constant current, DC-blocking capacitor |
| Current sensor on an Uno ADC | About 26 mA per ADC step with an ACS712 5 A, useless for 10 to 30 mA | Shunt and amplifier into a 12-bit ADC, mid-pulse sampling, hardware limit |
| Generic double-integrator muscle model | Not a muscle | Recruitment nonlinearity, dead time, two lags, fatigue, identified from data |
| PID on an unknown plant | No identification, no tuning rule | Staircase and step tests, SOPDT fit, SIMC tuning |
| No stimulator hardware listed | Cannot stimulate | Boost supply, current source, H-bridge, blocking capacitor, current limit, kill switch, battery only, isolated USB |
| Mixed or missing units in errors | Ambiguous | Force N, intensity 0 to 1, current mA, Kp N/N, Ki 1/s, Kd s, errors RMSE (N) and IAE (N s) |

## License

MIT, see [LICENSE](LICENSE). No warranty, not a medical device.
