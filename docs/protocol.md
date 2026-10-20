# Serial protocol

230400 baud, 8N1, one command per line (case-insensitive). Lines that start with `#` are messages,
all other lines are CSV telemetry. `python -m muscle_stim.log_serial` handles both.

## Commands

| Command | State needed | Action |
|---|---|---|
| `HELP`, `STATUS`, `HEADER` | any | Print help, state, flags, gains and limits, or the CSV header |
| `ADC` | any | Print averaged raw force-sensor counts (for FSR calibration) |
| `TARE` | IDLE | Average 1 s of the unloaded force sensor and store it as zero offset |
| `CAL EMG REST` / `CAL EMG MVC` | IDLE | 3 s of relaxed / maximum voluntary contraction of the healthy muscle |
| `SET KP\|KI\|KD\|TF <value>` | not RUNNING | Change a controller gain (Kp [N/N], Ki [1/s], Kd [s], Tf [s]) |
| `ARM` | IDLE | Needs `FSR_CALIBRATED` and a TARE. Enables the stimulation stage |
| `START TRAP [amp_N]` | ARMED | Repeated trapezoid contractions, closed loop. Default 4 N, maximum `F_REF_MAX_N` |
| `START HOLD <amp_N> <sec>` | ARMED | One ramp and constant hold, closed loop (fatigue tests) |
| `START MIRROR` | ARMED | Reference from the healthy-limb EMG. Needs EMG calibration |
| `START STAIRCASE` | ARMED | Open loop, ascending intensity steps. Used for the recruitment curve |
| `START STEP [lo hi]` | ARMED | Force-demand step through the inverse recruitment. Used for the dynamic model |
| `STOP` | ARMED, RUNNING | Stop, state IDLE |
| `RESET` | not RUNNING | Clear latched faults |

`START` needs the dead-man button held. Closed-loop modes need `RECRUIT_IDENTIFIED = true`.

## Telemetry (100 Hz while armed or running, 5 Hz otherwise)

`t_ms,state,faults,ref_N,force_N,emg_pct,u,v_N,i_peak_mA,t_skin_C,v_rail_V,hr_bpm`

| Column | Unit | Meaning |
|---|---|---|
| t_ms | ms | Time since boot |
| state | - | 0 IDLE, 1 ARMED, 2 RUNNING, 3 FAULT |
| faults | bit mask | See below |
| ref_N | N | Force reference |
| force_N | N | Calibrated, tared measured force |
| emg_pct | % MVC | Healthy-limb EMG envelope (nan if not calibrated) |
| u | 0 to 1 | Stimulation intensity. Pulse current = u x `I_MAX_MA` |
| v_N | N | Force demand before the inverse recruitment |
| i_peak_mA | mA | Measured pulse current (mid-pulse sample) |
| t_skin_C | C | NTC temperature |
| v_rail_V | V | Stimulation supply rail |
| hr_bpm | 1/min | Heart rate (nan if not valid) |

## Fault bits

| Bit | Name | Bit | Name |
|---|---|---|---|
| 0 | OVERCURRENT | 7 | EMG_LEADOFF |
| 1 | OVERVOLTAGE | 8 | WATCHDOG |
| 2 | OVERTEMP | 9 | SESSION_TIME |
| 3 | HR_RANGE | 10 | CONTRACTION |
| 4 | HR_MISSING | 11 | NO_CONTACT |
| 5 | FORCE_LIMIT | 12 | TEMP_SENSOR |
| 6 | FORCE_SENSOR | | |
