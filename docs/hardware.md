# Hardware

Read `../SAFETY.md` first. The digital side (ESP32, sensors, firmware) is fully specified here. The
high-voltage analogue stimulator stage is specified by **function and part class** (see
`figures/fig_stimulator.png`); build and verify it on a dummy load before connecting a person.

## Bill of materials

| Part | Purpose | Note |
|---|---|---|
| ESP32 DevKit (WROOM-32, 30 pin) | Controller | ADC1 pins only, 12 bit |
| FSR 402 or 406 + rigid backing + force-spreading pad | Force | Measures force on the pad, calibrate in place |
| EMG sensor module (AD8232 or MyoWare) | Healthy-limb EMG for mirror mode | Analogue output, own electrodes |
| MCP4725 12-bit DAC breakout (0x60) | Sets the stimulation current | I2C |
| MAX30102 breakout (0x57) | Heart rate | Finger sensor, plausibility check |
| 10 k NTC (beta 3950) + 10 k resistor | Skin temperature | Under or next to an electrode |
| Shunt (10 ohm) + amplifier (gain 8) | Pulse current measurement | Mid-pulse ADC sample |
| 1 M / 22 k divider | Rail voltage | |
| Boost converter (9 V to 50-60 V) | Compliance voltage | Battery fed, current limited |
| Voltage-controlled current source (op-amp + MOSFET, 110 ohm sense) | I = V_DAC / R_sense | 3.3 V gives 30 mA |
| H-bridge (high-voltage MOSFET driver stage) | Two phases of opposite sign | Never both on |
| DC-blocking capacitor (about 10 uF, voltage rated above the rail) | No DC through the skin | |
| Hardware current limit, fuse, kill switch | First layer of protection | Independent of the ESP32 |
| Dead-man push button | Hold to run | GPIO13 to GND |
| Two 9 V batteries | Supply | Battery only |
| USB isolator | Serial logging with electrodes on a person | |
| Self-adhesive hydrogel electrodes | Stimulation and EMG | |
| Pull-down resistor on `STIM_ENABLE` | Output off while the ESP32 boots | Required |

## Pin map (`firmware/muscle_stim_pid/config.h`)

| Signal | GPIO | Note |
|---|---|---|
| Force sensor | 34 | ADC1, input only |
| EMG | 35 | ADC1, input only |
| Pulse-current sense | 32 | ADC1 |
| Rail-voltage sense | 33 | ADC1 |
| Skin NTC | 39 (VN) | ADC1, input only |
| EMG lead-off LO+, LO- | 25, 26 | Set `USE_EMG_LEADOFF 0` if the module has none |
| Phase A, phase B | 18, 19 | H-bridge drive |
| Stimulation enable | 4 | External pull-down |
| Dead-man | 13 | Input with pull-up, active low |
| I2C SDA, SCL | 21, 22 | DAC and heart-rate sensor |

Strapping pins (0, 2 as input, 5, 12, 15) are not used for signals. Do not use ADC2 pins: they do not work reliably when
other peripherals are active.

## Stimulation waveform

Symmetric biphasic constant-current pulses: 300 us per phase, 50 us gap, 35 Hz. Intensity `u` (0 to 1) sets the
current, `I = u x I_MAX_MA` (default maximum 30 mA). Duty cycle is about 2 %. Charge per phase at 30 mA is
9 uC, equal and opposite in the two phases. Constant current means the delivered stimulation does not
depend on skin impedance (the original project used PWM voltage, where it does).

The ESP32 only commands the current. The current source and the hardware limit set what can actually
flow. The firmware compares the measured pulse current with the command and stops if they disagree.

## FSR calibration

An FSR is repeatable only to about 10 % and drifts, so calibrate the exact mounted sensor.

1. Mount the FSR as it will be used (rigid backing, same force-spreading pad).
2. In a serial terminal type `ADC`. Record the counts at 0 g, then place known masses on the pad
   (for example 50, 100, 200, 300, 500, 700, 1000 g), wait 5 s each, `ADC`, write `mass_g,adc` rows to
   `fsr_points.csv`. Repeat for at least 8 masses, going up and down.
3. `python -m muscle_stim.calibrate_fsr fsr_points.csv --out fsr_lut.txt --plot fsr_calibration.png`
   fits two models (power law and divider saturation), picks the one with the lower leave-one-out
   error and writes a 16-point table.
4. Paste the table into `config.h` (`FSR_LUT_*`), set `FSR_CALIBRATED = true`, upload. Report the
   leave-one-out RMSE in your results.

## Identification workflow (on a person, after the dummy-load tests)

1. `TARE` with nothing on the sensor. Fix the limb so it presses on the pad isometrically.
2. `ARM`, hold the dead-man button, `START STAIRCASE`. Intensity rises in ten ascending steps up to
   `STAIRCASE_U_MAX` (0.6), 2 s each with 8 s rest. Stop at any discomfort.
   Fatigue biases a long staircase, the ascending order is used for comfort, not accuracy.
3. `python -m muscle_stim.log_serial --port <port> --out staircase.csv` records it (start it before the test).
   `python -m muscle_stim.identify recruitment staircase.csv` writes the recruitment table.
   Paste it into `config.h`, set `RECRUIT_IDENTIFIED = true`, upload.
4. `START STEP 3 6` gives a force-demand step. `python -m muscle_stim.identify dynamics step.csv --t0 <s> --t1 <s>`
   fits the dead-time-plus-two-lags model and prints PI and PID gains (SIMC rules). Send them with `SET KP ...`.
5. `START TRAP 4` for closed-loop trials, `python -m muscle_stim.analyze_session session.csv` for the metrics.

The table in `config.h` as shipped comes from the **simulation** and is not any real muscle.

## Mirror mode (healthy limb sets the target)

EMG electrodes on the healthy limb, stimulation electrodes on the affected limb. `CAL EMG REST`, then
`CAL EMG MVC`. The EMG envelope in % MVC, times `MIRROR_FMAX_N`, becomes the force reference (rate limited to
8 N/s). EMG is not sampled during a pulse and a short time after it (stimulation artefact). The EMG and
force are in different units, so they are linked only through % MVC and a chosen force scale.
