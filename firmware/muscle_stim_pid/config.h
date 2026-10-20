// config.h - every setting of the stimulator in one place.
//
// READ docs/hardware.md and SAFETY.md before changing any limit. Limits are written so that a
// wrong value is caught by the static_asserts at the bottom of this file at compile time.
#pragma once
#include <stdint.h>

// ============================================================================================
// 1. Pin map (ESP32-WROOM-32 DevKit, 30-pin). Change for other boards.
//    ADC inputs are on ADC1 only (GPIO32-39). GPIO34-39 are input-only pins.
// ============================================================================================
constexpr int PIN_FSR_ADC      = 34;   // force sensor divider output
constexpr int PIN_EMG_ADC      = 35;   // EMG sensor analog output (contralateral / healthy limb)
constexpr int PIN_ISENSE_ADC   = 32;   // amplified shunt voltage = pulse current
constexpr int PIN_VRAIL_ADC    = 33;   // stimulation supply rail through a divider
constexpr int PIN_NTC_ADC      = 39;   // skin-temperature NTC divider (GPIO39 = VN)
constexpr int PIN_EMG_LO_PLUS  = 25;   // AD8232 LO+ lead-off output; -1 if the sensor has none
constexpr int PIN_EMG_LO_MINUS = 26;   // AD8232 LO- lead-off output; -1 if the sensor has none
constexpr int PIN_PHASE_A      = 18;   // H-bridge drive, first (positive) phase
constexpr int PIN_PHASE_B      = 19;   // H-bridge drive, second (negative) phase
constexpr int PIN_STIM_ENABLE  = 4;    // enables the high-voltage stage. External pull-DOWN required
constexpr int PIN_DEADMAN      = 13;   // hold-to-run push button to GND (INPUT_PULLUP)
constexpr int PIN_STATUS_LED   = 2;    // on-board LED
constexpr int PIN_I2C_SDA      = 21;   // MCP4725 DAC (0x60) and MAX30102 (0x57)
constexpr int PIN_I2C_SCL      = 22;
constexpr uint8_t DAC_I2C_ADDR = 0x60; // MCP4725 with A0 = GND

constexpr long SERIAL_BAUD     = 230400;

// ============================================================================================
// 2. Timing
// ============================================================================================
constexpr float CONTROL_TS_S   = 0.010f;   // control sample time: 100 Hz (same as the simulations)
constexpr int   EMG_FS_HZ      = 1000;     // EMG sampling rate
constexpr int   FSR_OVERSAMPLE = 16;       // ADC readings averaged per control step

// ============================================================================================
// 3. Stimulation waveform: symmetric biphasic pulses, amplitude set by the controller
// ============================================================================================
constexpr float STIM_FREQ_HZ     = 35.0f;  // pulse repetition rate (20-50 Hz is usual for NMES)
constexpr int   PULSE_WIDTH_US   = 300;    // width of EACH phase (200-400 us is usual)
constexpr int   INTERPHASE_US    = 50;     // gap between the two phases
constexpr float I_MAX_MA         = 30.0f;  // current at u = 1. Start LOWER (see SAFETY.md)
constexpr float R_SENSE_OHM      = 110.0f; // command resistor of the constant-current stage:
                                           //   current = DAC volts / R_SENSE (set by the circuit)
constexpr float DAC_VREF_V       = 3.3f;   // MCP4725 supply = full scale

// pulse-current measurement (shunt + amplifier into the ADC)
constexpr float I_SENSE_SHUNT_OHM = 10.0f;
constexpr float I_SENSE_AMP_GAIN  = 8.0f;
constexpr float ADC_FULLSCALE_V   = 3.1f;  // ESP32 ADC at 11 dB attenuation; calibrate on your board

// rail voltage divider: V_rail = V_adc * (R_TOP + R_BOTTOM) / R_BOTTOM
constexpr float VRAIL_R_TOP_OHM    = 1000000.0f;
constexpr float VRAIL_R_BOTTOM_OHM = 22000.0f;

// skin-temperature NTC (10 k at 25 C, beta 3950) with a 10 k resistor to 3.3 V
constexpr float NTC_R_FIXED_OHM = 10000.0f;
constexpr float NTC_R0_OHM      = 10000.0f;
constexpr float NTC_BETA        = 3950.0f;

// ============================================================================================
// 4. Safety limits (see SAFETY.md). All are enforced by safety_supervisor.h
// ============================================================================================
constexpr float F_REF_MAX_N        = 8.0f;   // highest force reference accepted
constexpr float F_LIMIT_N          = 10.0f;  // measured force above this stops the stimulator
constexpr float I_PEAK_LIMIT_MA    = 35.0f;  // measured pulse current above this stops it
constexpr float V_RAIL_MAX_V       = 70.0f;  // measured rail above this stops it
constexpr float T_SKIN_MAX_C       = 40.0f;  // absolute skin temperature limit
constexpr float T_SKIN_RISE_MAX_C  = 4.0f;   // rise above the temperature at ARM
constexpr bool  HR_REQUIRED        = true;   // need a valid heart rate while running
constexpr float HR_MIN_BPM         = 40.0f;
constexpr float HR_MAX_BPM         = 150.0f;
constexpr float HR_RANGE_TIME_S    = 3.0f;   // out of range this long -> stop
constexpr float HR_MISSING_TIME_S  = 15.0f;  // no valid HR this long -> stop
constexpr float SESSION_MAX_S      = 300.0f; // longest RUNNING session
constexpr float ACTIVE_BUCKET_MAX_S = 15.0f; // "stimulation-on" budget (leaky bucket) ...
constexpr float ACTIVE_RECOVERY_RATE = 1.0f; // ... refilled at this many seconds per second of rest
constexpr float NO_CONTACT_TIME_S  = 1.0f;   // commanded current but none measured this long -> stop
constexpr float LOOP_LATE_MS       = 50.0f;  // control loop not run for this long -> stop
#define USE_HEART_RATE 1   // 1 = MAX30102 heart-rate monitor (needs the "SparkFun MAX3010x" library); 0 = none
#define USE_EMG_LEADOFF 1  // 1 = read AD8232 LO+/LO- lead-off pins; 0 = sensor without them (e.g. MyoWare)

// ============================================================================================
// 5. Controller defaults (change at run time with SET KP / KI / KD / TF)
//    Initial values are the SIMC PI gains from the simulation study: they are STARTING POINTS,
//    identify your own muscle with the staircase and step tests (docs/hardware.md).
// ============================================================================================
constexpr float DEFAULT_KP = 0.53f;   // N/N
constexpr float DEFAULT_KI = 4.8f;    // 1/s
constexpr float DEFAULT_KD = 0.0f;    // s    (0 = PI controller)
constexpr float DEFAULT_TF = 0.02f;   // s    derivative filter
constexpr float U_RATE_MAX_PER_S = 2.0f;   // slew limit of the stimulation intensity
constexpr float REF_OFF_N        = 0.05f;  // reference below this switches stimulation off

// preset trapezoid profile (same shape as the simulation, longer rests for safety)
constexpr float TRAP_REST0_S = 1.0f;
constexpr float TRAP_RAMP_S  = 1.5f;
constexpr float TRAP_HOLD_S  = 4.0f;
constexpr float TRAP_REST_S  = 8.0f;
constexpr int   TRAP_REPS    = 3;
constexpr float TRAP_DEFAULT_AMP_N = 4.0f;

// mirror mode: EMG of the healthy limb sets the target force of the stimulated limb
constexpr float MIRROR_FMAX_N     = 6.0f;   // target force at 100 % MVC
constexpr float MIRROR_DEADBAND   = 0.05f;  // fraction of MVC treated as rest
constexpr float MIRROR_REF_RATE   = 8.0f;   // N/s, limit on how fast the target may change
constexpr float EMG_DC_TC_S       = 0.5f;   // DC-removal time constant
constexpr float EMG_LP_HZ         = 3.0f;   // envelope low-pass

// identification tests
constexpr float STAIRCASE_U_MAX   = 0.6f;   // highest intensity of the staircase, raise slowly
constexpr float STAIRCASE_HOLD_S  = 2.0f;
constexpr float STAIRCASE_REST_S  = 8.0f;
constexpr int   STAIRCASE_STEPS   = 10;     // levels = k/STEPS * STAIRCASE_U_MAX, ascending
constexpr float STEP_V_LO_N       = 3.0f;
constexpr float STEP_V_HI_N       = 6.0f;
constexpr float STEP_T_STEP_S     = 2.0f;
constexpr float STEP_TOTAL_S      = 8.0f;

// ============================================================================================
// 6. Calibration tables
// ============================================================================================
// 6a. Force sensor: raw ADC counts -> newtons.   Produce with:
//       python -m muscle_stim.calibrate_fsr fsr_points.csv
//     then paste the three lines here and set FSR_CALIBRATED = true.
//     The numbers below are ILLUSTRATIVE ONLY and are not a calibration of any real sensor.
constexpr bool FSR_CALIBRATED = false;
constexpr int FSR_LUT_N = 6;
constexpr float FSR_LUT_ADC[FSR_LUT_N]     = { 100.0f, 500.0f, 1000.0f, 1700.0f, 2500.0f, 3300.0f };
constexpr float FSR_LUT_N_FORCE[FSR_LUT_N] = { 0.0000f, 0.5000f, 1.5000f, 3.5000f, 6.5000f, 10.0000f };

// 6b. Recruitment curve: stimulation intensity u -> steady force (N). Produce with:
//       python -m muscle_stim.identify recruitment staircase_session.csv
//     The values below come from the SIMULATION (data/sim) and are NOT your muscle.
constexpr bool RECRUIT_IDENTIFIED = false;
constexpr int RECRUIT_N = 10;
constexpr float RECRUIT_U[RECRUIT_N] = { 0.0000f, 0.1000f, 0.2000f, 0.3000f, 0.4000f, 0.5000f, 0.6000f, 0.7000f, 0.8000f, 0.9000f };
constexpr float RECRUIT_F_N[RECRUIT_N] = { 0.0000f, 0.3093f, 1.0693f, 2.6468f, 4.8453f, 8.6674f, 10.8191f, 11.7228f, 13.3972f, 13.8375f };

// ============================================================================================
// 7. Compile-time sanity checks
// ============================================================================================
static_assert(PULSE_WIDTH_US >= 100 && PULSE_WIDTH_US <= 400, "pulse width must be 100-400 us");
static_assert(INTERPHASE_US >= 20 && INTERPHASE_US <= 200, "interphase gap must be 20-200 us");
static_assert(STIM_FREQ_HZ >= 10.0f && STIM_FREQ_HZ <= 60.0f, "pulse rate must be 10-60 Hz");
static_assert(I_MAX_MA > 0.0f && I_MAX_MA <= 50.0f, "I_MAX_MA must be 0-50 mA (SAFETY.md)");
static_assert(I_PEAK_LIMIT_MA > I_MAX_MA, "peak-current limit must be above the commanded maximum");
static_assert(I_PEAK_LIMIT_MA <= 60.0f, "peak-current limit must not exceed 60 mA");
static_assert(F_LIMIT_N > F_REF_MAX_N, "force limit must be above the highest reference");
static_assert(STAIRCASE_U_MAX > 0.0f && STAIRCASE_U_MAX <= 1.0f, "staircase intensity must be 0-1");
static_assert(STIM_FREQ_HZ * (2 * PULSE_WIDTH_US + INTERPHASE_US) < 100000.0f,
              "pulse duty cycle must stay below 10 %");
static_assert(I_MAX_MA * R_SENSE_OHM / 1000.0f <= DAC_VREF_V, "DAC cannot reach I_MAX_MA with this R_SENSE_OHM");
