// muscle_stim_pid.ino - closed-loop force control of electrically stimulated muscle (ESP32).
//
// READ SAFETY.md FIRST. This firmware drives a hardware stimulator stage. Test with a resistor +
// capacitor dummy load before any person is connected, and only use it on people with supervisor
// approval.
//
// Block diagram of the software (details in docs/theory.md and docs/protocol.md):
//
//   emgTask   (1 kHz)  : EMG of the healthy limb -> envelope -> % MVC (mirror mode)
//   hrTask    (100 Hz) : MAX30102 -> heart rate (safety monitor)
//   controlTask (100 Hz): force sensor -> reference -> PID + inverse recruitment -> u
//                         safety supervisor -> telemetry. All state changes happen here.
//   pulseTask : turns u (0..1) into biphasic constant-current pulses (35 Hz) via DAC + H-bridge
//   loop()    : reads the serial port, parses commands, hands them to controlTask through a queue
//
// Libraries: "SparkFun MAX3010x Pulse and Proximity Sensor Library" (only if USE_HEART_RATE = 1).
// Board: ESP32 Dev Module (Arduino-ESP32 core 2.x or 3.x).

#include <Arduino.h>
#include <Wire.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "config.h"
#include "emg_envelope.h"
#include "fsr_calibration.h"
#include "pid_controller.h"
#include "profiles.h"
#include "safety_supervisor.h"

#if USE_HEART_RATE
#include "MAX30105.h"
#include "heartRate.h"
#endif

// ============================================================================================
// state and shared data
// ============================================================================================
enum State : uint8_t { ST_IDLE = 0, ST_ARMED = 1, ST_RUNNING = 2, ST_FAULT = 3 };
enum Mode : uint8_t { MODE_NONE = 0, MODE_TRAP, MODE_HOLD, MODE_MIRROR, MODE_STAIRCASE, MODE_STEP };
enum CmdType : uint8_t {
  C_ARM, C_START, C_STOP, C_RESET, C_TARE, C_ADC, C_CAL_REST, C_CAL_MVC, C_SET, C_STATUS, C_HEADER
};
enum SetWhich : uint8_t { S_KP, S_KI, S_KD, S_TF };

struct Cmd {
  uint8_t type;
  uint8_t mode;   // for C_START
  uint8_t which;  // for C_SET
  float a;
  float b;
};

// written by one task, read by another (single 32-bit values, so reads and writes are atomic)
static volatile float    g_u_cmd = 0.0f;          // control -> pulse
static volatile bool     g_stim_on = false;       // control -> pulse: pulses allowed
static volatile uint32_t g_heartbeat_us = 0;      // control -> pulse: loop is alive
static volatile float    g_i_peak_ma = 0.0f;      // pulse -> control: measured pulse current
static volatile uint32_t g_i_peak_us = 0;         // time of that measurement
static volatile uint32_t g_blank_start_us = 0;    // pulse -> emg: pulse window start
static volatile uint32_t g_blank_len_us = 0;      // pulse -> emg: pulse window length
static volatile float    g_emg_env = 0.0f;        // emg -> control (ADC counts)
static volatile float    g_hr_bpm = NAN;          // hr -> control
static volatile bool     g_hr_valid = false;

static SemaphoreHandle_t g_i2c_mutex;
static QueueHandle_t     g_cmd_queue;

// objects owned by the control task (initialised in setup)
static InverseRecruitment g_inv;
static ForceController*   g_ctrl = nullptr;
static FsrCalibration     g_fsr;
static EmgNormalizer      g_emg_norm;
static EmgEnvelope        g_emg_env_filter(EMG_FS_HZ, EMG_DC_TC_S, EMG_LP_HZ);  // used by emgTask only

static SafetyLimits makeLimits() {
  SafetyLimits l;
  l.i_peak_limit_ma = I_PEAK_LIMIT_MA;
  l.v_rail_max_v = V_RAIL_MAX_V;
  l.t_skin_max_c = T_SKIN_MAX_C;
  l.t_skin_rise_max_c = T_SKIN_RISE_MAX_C;
  l.hr_required = HR_REQUIRED && (USE_HEART_RATE != 0);
  l.hr_min_bpm = HR_MIN_BPM;
  l.hr_max_bpm = HR_MAX_BPM;
  l.hr_range_time_s = HR_RANGE_TIME_S;
  l.hr_missing_time_s = HR_MISSING_TIME_S;
  l.force_limit_n = F_LIMIT_N;
  l.session_max_s = SESSION_MAX_S;
  l.bucket_max_s = ACTIVE_BUCKET_MAX_S;
  l.bucket_recovery_rate = ACTIVE_RECOVERY_RATE;
  l.no_contact_time_s = NO_CONTACT_TIME_S;
  l.loop_late_ms = LOOP_LATE_MS;
  return l;
}
static SafetySupervisor g_safety(makeLimits());

// ============================================================================================
// small helpers
// ============================================================================================
static void logMsg(const char* fmt, ...) {
  char buf[160];
  buf[0] = '#';
  buf[1] = ' ';
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf + 2, sizeof(buf) - 4, fmt, ap);
  va_end(ap);
  if (n < 0) return;
  int len = 2 + n;
  if (len > (int)sizeof(buf) - 2) len = sizeof(buf) - 2;
  buf[len++] = '\n';
  Serial.write((const uint8_t*)buf, len);      // one write call per line, lines never interleave
}

static float readAdcAvg(int pin, int n) {
  uint32_t sum = 0;
  for (int i = 0; i < n; ++i) sum += analogRead(pin);
  return (float)sum / (float)n;
}

static float ntcCelsius(float counts) {
  const float c = fminf(fmaxf(counts, 1.0f), 4094.0f);
  const float r = NTC_R_FIXED_OHM * c / (4095.0f - c);
  const float inv_t = 1.0f / 298.15f + logf(r / NTC_R0_OHM) / NTC_BETA;
  return 1.0f / inv_t - 273.15f;
}

static float railVolts(float counts) {
  const float v_adc = counts / 4095.0f * ADC_FULLSCALE_V;
  return v_adc * (VRAIL_R_TOP_OHM + VRAIL_R_BOTTOM_OHM) / VRAIL_R_BOTTOM_OHM;
}

static float shuntMilliamps(float counts) {
  const float v_adc = counts / 4095.0f * ADC_FULLSCALE_V;
  return v_adc / (I_SENSE_AMP_GAIN * I_SENSE_SHUNT_OHM) * 1000.0f;
}

static inline void waitUntilUs(uint32_t target_us) {
  while ((int32_t)(target_us - micros()) > 0) {
  }
}

// MCP4725 fast-mode write (12 bit). The DAC voltage commands the pulse current: I = V / R_SENSE.
static bool dacWrite(uint16_t code, uint32_t timeout_ms) {
  if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) return false;
  Wire.beginTransmission(DAC_I2C_ADDR);
  Wire.write((uint8_t)((code >> 8) & 0x0F));
  Wire.write((uint8_t)(code & 0xFF));
  const uint8_t err = Wire.endTransmission();
  xSemaphoreGive(g_i2c_mutex);
  return err == 0;
}

static uint16_t dacCodeForU(float u) {
  float i_ma = fminf(fmaxf(u, 0.0f), 1.0f) * I_MAX_MA;
  float volts = i_ma * R_SENSE_OHM / 1000.0f;
  float code = volts / DAC_VREF_V * 4095.0f;
  if (code < 0.0f) code = 0.0f;
  if (code > 4095.0f) code = 4095.0f;
  return (uint16_t)(code + 0.5f);
}

static void outputsOff() {
  digitalWrite(PIN_PHASE_A, LOW);
  digitalWrite(PIN_PHASE_B, LOW);
}

// ============================================================================================
// pulse task: biphasic constant-current pulses
// ============================================================================================
static void pulseTask(void*) {
  const uint32_t period_us = (uint32_t)(1000000.0f / STIM_FREQ_HZ);
  uint32_t next_us = micros() + period_us;
  uint16_t last_code = 0;
  for (;;) {
    const uint32_t now = micros();
    const bool alive = (uint32_t)(now - g_heartbeat_us) < 100000UL;   // control loop ran < 100 ms ago
    const float u = g_u_cmd;
    const bool go = g_stim_on && alive && u > 0.005f;
    if (!go) {
      outputsOff();
      if (last_code != 0) {
        if (dacWrite(0, 50)) last_code = 0;
      }
      vTaskDelay(pdMS_TO_TICKS(2));
      next_us = micros() + period_us;
      continue;
    }
    const uint16_t code = dacCodeForU(u);
    if (code != last_code) {
      if (dacWrite(code, 5)) last_code = code;
    }
    // sleep until just before the next pulse, then wait precisely
    while ((int32_t)(next_us - micros()) > 1500) vTaskDelay(1);
    waitUntilUs(next_us);
    if (!g_stim_on) continue;                       // disabled while waiting

    g_blank_start_us = micros();
    g_blank_len_us = 2 * PULSE_WIDTH_US + INTERPHASE_US + 3000;   // EMG ignores this window

    const uint32_t t0 = micros();
    digitalWrite(PIN_PHASE_A, HIGH);                                // phase 1 (positive)
    waitUntilUs(t0 + PULSE_WIDTH_US / 2);
    const int c1 = analogRead(PIN_ISENSE_ADC);                      // mid-pulse current sample
    waitUntilUs(t0 + PULSE_WIDTH_US);
    digitalWrite(PIN_PHASE_A, LOW);
    waitUntilUs(t0 + PULSE_WIDTH_US + INTERPHASE_US);               // dead time (no shoot-through)
    const uint32_t t1 = micros();
    digitalWrite(PIN_PHASE_B, HIGH);                                // phase 2 (negative)
    waitUntilUs(t1 + PULSE_WIDTH_US / 2);
    const int c2 = analogRead(PIN_ISENSE_ADC);
    waitUntilUs(t1 + PULSE_WIDTH_US);
    digitalWrite(PIN_PHASE_B, LOW);

    g_i_peak_ma = shuntMilliamps((float)(c1 > c2 ? c1 : c2));
    g_i_peak_us = micros();

    next_us += period_us;
    if ((int32_t)(micros() - next_us) > (int32_t)period_us) next_us = micros() + period_us;   // fell behind
  }
}

// ============================================================================================
// EMG task (1 kHz) and heart-rate task
// ============================================================================================
static void emgTask(void*) {
  TickType_t last = xTaskGetTickCount();
  TickType_t period = pdMS_TO_TICKS(1000 / EMG_FS_HZ);
  if (period < 1) period = 1;
  for (;;) {
    vTaskDelayUntil(&last, period);
    if ((uint32_t)(micros() - g_blank_start_us) < g_blank_len_us) continue;   // stimulation artifact
    const float x = readAdcAvg(PIN_EMG_ADC, 4);
    g_emg_env = g_emg_env_filter.update(x);
  }
}

#if USE_HEART_RATE
static MAX30105 g_hrSensor;

static void hrTask(void*) {
  bool ok = false;
  if (xSemaphoreTake(g_i2c_mutex, portMAX_DELAY) == pdTRUE) {
    ok = g_hrSensor.begin(Wire, I2C_SPEED_FAST);
    if (ok) {
      g_hrSensor.setup();
      g_hrSensor.setPulseAmplitudeRed(0x0A);
      g_hrSensor.setPulseAmplitudeGreen(0);
    }
    xSemaphoreGive(g_i2c_mutex);
  }
  if (!ok) {
    logMsg("WARN MAX30102 not found: heart rate unavailable (stimulation will refuse to run with HR_REQUIRED)");
    for (;;) {
      g_hr_valid = false;
      vTaskDelay(pdMS_TO_TICKS(1000));
    }
  }
  const int N = 4;
  float rates[N] = {0, 0, 0, 0};
  int n_rates = 0, idx = 0;
  uint32_t last_beat_ms = 0;
  for (;;) {
    long ir = 0;
    if (xSemaphoreTake(g_i2c_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
      ir = g_hrSensor.getIR();
      xSemaphoreGive(g_i2c_mutex);
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }
    const uint32_t now_ms = millis();
    if (ir < 50000) {                                 // no finger on the sensor
      g_hr_valid = false;
      n_rates = 0;
      idx = 0;
      last_beat_ms = now_ms;
    } else if (checkForBeat(ir)) {
      const uint32_t delta = now_ms - last_beat_ms;
      last_beat_ms = now_ms;
      const float bpm = 60000.0f / (float)delta;
      if (delta > 0 && bpm > 20.0f && bpm < 255.0f) {
        rates[idx] = bpm;
        idx = (idx + 1) % N;
        if (n_rates < N) ++n_rates;
        float s = 0;
        for (int i = 0; i < n_rates; ++i) s += rates[i];
        g_hr_bpm = s / (float)n_rates;
        g_hr_valid = (n_rates >= N);
      }
    }
    if (now_ms - last_beat_ms > 5000) g_hr_valid = false;   // beats stopped
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}
#endif

// ============================================================================================
// control task
// ============================================================================================
struct Session {
  State state = ST_IDLE;
  Mode mode = MODE_NONE;
  uint32_t t_arm_us = 0;      // ARM time
  uint32_t t_start_us = 0;    // START time (profile time zero)
  float amp = 0.0f;           // TRAP / HOLD amplitude (N)
  float hold_s = 0.0f;        // HOLD duration
  float step_lo = STEP_V_LO_N, step_hi = STEP_V_HI_N;
  float ref = 0.0f;           // current reference (N)
  float mirror_ref = 0.0f;
  float t_skin_start = 25.0f;
  bool tared = false;
  float force_offset_n = 0.0f;
  int tare_left = 0;
  double tare_sum = 0.0;
  int cal_left = 0;           // 0 none, else steps left of an EMG calibration
  uint8_t cal_kind = 0;       // 1 rest, 2 mvc
  double cal_sum = 0.0;
  float cal_max = 0.0f;
  int cal_n = 0;
  int deadman_released_steps = 0;
  uint32_t last_step_us = 0;
  uint32_t step_count = 0;
};
static Session S;
static PidGains g_gains;

static void enterIdle(const char* reason) {
  S.state = ST_IDLE;
  S.mode = MODE_NONE;
  g_stim_on = false;
  g_u_cmd = 0.0f;
  digitalWrite(PIN_STIM_ENABLE, LOW);
  outputsOff();
  if (g_ctrl) g_ctrl->reset(0.0f);
  S.mirror_ref = 0.0f;
  S.ref = 0.0f;
  logMsg("IDLE (%s)", reason);
}

static void enterFault(uint32_t mask) {
  char names[96];
  SafetySupervisor::describe(mask, names, sizeof(names));
  S.state = ST_FAULT;
  S.mode = MODE_NONE;
  g_stim_on = false;
  g_u_cmd = 0.0f;
  digitalWrite(PIN_STIM_ENABLE, LOW);
  outputsOff();
  logMsg("FAULT 0x%04lX %s  (send RESET after fixing the cause)", (unsigned long)mask, names);
}

static bool closedLoopReady() {
  if (!RECRUIT_IDENTIFIED) {
    logMsg("ERR recruitment curve not identified. Run START STAIRCASE, then python -m muscle_stim.identify, paste the table into config.h and set RECRUIT_IDENTIFIED = true");
    return false;
  }
  if (g_ctrl == nullptr) {
    logMsg("ERR recruitment table in config.h is invalid");
    return false;
  }
  return true;
}

static void handleCommand(const Cmd& c, float adc_counts_now) {
  char buf[8];
  (void)buf;
  switch (c.type) {
    case C_ARM:
      if (S.state != ST_IDLE) { logMsg("ERR ARM only from IDLE"); break; }
      if (!FSR_CALIBRATED) { logMsg("ERR force sensor not calibrated (see docs/hardware.md, set FSR_CALIBRATED = true)"); break; }
      if (!S.tared) { logMsg("ERR do TARE first (no load on the force sensor)"); break; }
      g_safety.reset();
      S.t_arm_us = micros();
      S.state = ST_ARMED;
      digitalWrite(PIN_STIM_ENABLE, HIGH);
      logMsg("ARMED. Hold the dead-man button, then START <mode>. STOP disarms.");
      break;
    case C_START: {
      if (S.state != ST_ARMED) { logMsg("ERR START only from ARMED"); break; }
      if (digitalRead(PIN_DEADMAN) != LOW) { logMsg("ERR hold the dead-man button first"); break; }
      const Mode m = (Mode)c.mode;
      if (m == MODE_TRAP || m == MODE_HOLD || m == MODE_STEP || m == MODE_MIRROR) {
        if (!closedLoopReady()) break;
      }
      if (m == MODE_TRAP || m == MODE_HOLD) {
        if (!(c.a > 0.1f && c.a <= F_REF_MAX_N)) { logMsg("ERR amplitude must be 0.1 to %.1f N", F_REF_MAX_N); break; }
      }
      if (m == MODE_STEP) {
        if (!(c.a >= 0.5f && c.b > c.a && c.b <= F_REF_MAX_N)) { logMsg("ERR step: 0.5 <= lo < hi <= %.1f N", F_REF_MAX_N); break; }
      }
      if (m == MODE_MIRROR && !g_emg_norm.calibrated()) { logMsg("ERR EMG not calibrated: CAL EMG REST, then CAL EMG MVC"); break; }
      S.mode = m;
      S.amp = c.a;
      S.hold_s = c.b;
      S.step_lo = c.a;
      S.step_hi = c.b;
      S.mirror_ref = 0.0f;
      S.ref = 0.0f;
      S.t_start_us = micros();
      if (g_ctrl) { g_ctrl->setGains(g_gains); g_ctrl->reset(0.0f); }
      S.state = ST_RUNNING;
      logMsg("RUNNING mode=%d", (int)m);
      break;
    }
    case C_STOP:
      if (S.state == ST_ARMED || S.state == ST_RUNNING) enterIdle("STOP");
      else logMsg("already stopped");
      break;
    case C_RESET:
      if (S.state == ST_RUNNING) { logMsg("ERR STOP first"); break; }
      g_safety.reset();
      S.state = ST_IDLE;
      digitalWrite(PIN_STIM_ENABLE, LOW);
      logMsg("RESET done, state IDLE");
      break;
    case C_TARE:
      if (S.state != ST_IDLE) { logMsg("ERR TARE only in IDLE"); break; }
      S.tare_left = 100;
      S.tare_sum = 0.0;
      logMsg("TARE: keep the sensor unloaded for 1 s ...");
      break;
    case C_ADC:
      logMsg("ADC %.1f", adc_counts_now);
      break;
    case C_CAL_REST:
    case C_CAL_MVC:
      if (S.state != ST_IDLE) { logMsg("ERR EMG calibration only in IDLE"); break; }
      S.cal_kind = (c.type == C_CAL_REST) ? 1 : 2;
      S.cal_left = 300;                    // 3 s
      S.cal_sum = 0.0;
      S.cal_max = 0.0f;
      S.cal_n = 0;
      logMsg(c.type == C_CAL_REST ? "CAL EMG REST: relax the healthy muscle for 3 s ..."
                                  : "CAL EMG MVC: squeeze the healthy muscle as hard as you comfortably can for 3 s ...");
      break;
    case C_SET:
      if (S.state == ST_RUNNING) { logMsg("ERR SET only when not running"); break; }
      if (c.which == S_KP) g_gains.kp = c.a;
      if (c.which == S_KI) g_gains.ki = c.a;
      if (c.which == S_KD) g_gains.kd = c.a;
      if (c.which == S_TF) g_gains.tf = c.a;
      logMsg("GAINS kp=%.4f ki=%.4f kd=%.5f tf=%.4f", g_gains.kp, g_gains.ki, g_gains.kd, g_gains.tf);
      break;
    case C_STATUS: {
      char names[96];
      SafetySupervisor::describe(g_safety.faults(), names, sizeof(names));
      logMsg("STATUS state=%d mode=%d faults=0x%04lX %s", (int)S.state, (int)S.mode,
             (unsigned long)g_safety.faults(), names);
      logMsg("STATUS FSR_CALIBRATED=%d RECRUIT_IDENTIFIED=%d tared=%d offset=%.3fN emgCal=%d",
             (int)FSR_CALIBRATED, (int)RECRUIT_IDENTIFIED, (int)S.tared, S.force_offset_n,
             (int)g_emg_norm.calibrated());
      logMsg("STATUS gains kp=%.4f ki=%.4f kd=%.5f tf=%.4f", g_gains.kp, g_gains.ki, g_gains.kd, g_gains.tf);
      logMsg("STATUS limits I_MAX=%.1fmA F_LIMIT=%.1fN T_MAX=%.1fC", I_MAX_MA, F_LIMIT_N, T_SKIN_MAX_C);
      break;
    }
    case C_HEADER:
      Serial.println("t_ms,state,faults,ref_N,force_N,emg_pct,u,v_N,i_peak_mA,t_skin_C,v_rail_V,hr_bpm");
      break;
    default:
      break;
  }
}

static void controlStep() {
  const uint32_t now_us = micros();
  const float loop_age_ms = (S.last_step_us == 0) ? 0.0f : (float)(uint32_t)(now_us - S.last_step_us) / 1000.0f;
  S.last_step_us = now_us;
  g_heartbeat_us = now_us;
  ++S.step_count;

  // ---- sensors ----------------------------------------------------------------------------
  const float fsr_counts = readAdcAvg(PIN_FSR_ADC, FSR_OVERSAMPLE);
  const float f_raw = g_fsr.newtons(fsr_counts);
  float force = f_raw - S.force_offset_n;
  if (!(force > 0.0f)) force = 0.0f;                 // also catches NaN
  const bool force_ok = !g_fsr.outOfRange(fsr_counts) && isfinite(f_raw);
  const float temp_counts = readAdcAvg(PIN_NTC_ADC, 8);
  const bool temp_ok = temp_counts > 100.0f && temp_counts < 3995.0f;
  const float t_skin = ntcCelsius(temp_counts);
  const float v_rail = railVolts(readAdcAvg(PIN_VRAIL_ADC, 8));
  const float emg_env = g_emg_env;
  bool leadoff = false;
#if USE_EMG_LEADOFF
  if (PIN_EMG_LO_PLUS >= 0 && PIN_EMG_LO_MINUS >= 0)
    leadoff = digitalRead(PIN_EMG_LO_PLUS) == HIGH || digitalRead(PIN_EMG_LO_MINUS) == HIGH;
#endif
  const bool hr_valid = g_hr_valid;
  const float hr_bpm = g_hr_bpm;
  float i_peak = g_i_peak_ma;
  if ((uint32_t)(now_us - g_i_peak_us) > 150000UL) i_peak = 0.0f;   // no recent pulses
  const bool deadman = digitalRead(PIN_DEADMAN) == LOW;
  if (deadman) S.deadman_released_steps = 0; else ++S.deadman_released_steps;

  // ---- commands from the serial port ------------------------------------------------------
  Cmd c;
  while (xQueueReceive(g_cmd_queue, &c, 0) == pdTRUE) handleCommand(c, fsr_counts);

  // ---- tare and EMG calibration (IDLE only) ------------------------------------------------
  if (S.tare_left > 0) {
    S.tare_sum += f_raw;
    if (--S.tare_left == 0) {
      S.force_offset_n = (float)(S.tare_sum / 100.0);
      S.tared = true;
      logMsg("TARE done, offset %.3f N", S.force_offset_n);
    }
  }
  if (S.cal_left > 0) {
    S.cal_sum += emg_env;
    if (emg_env > S.cal_max) S.cal_max = emg_env;
    ++S.cal_n;
    if (--S.cal_left == 0) {
      if (S.cal_kind == 1) {
        g_emg_norm.setRest((float)(S.cal_sum / S.cal_n));
        logMsg("EMG rest level %.2f counts", g_emg_norm.rest());
      } else {
        g_emg_norm.setMvc(S.cal_max);
        logMsg("EMG MVC level %.2f counts", g_emg_norm.mvc());
      }
      if (g_emg_norm.calibrated()) logMsg("EMG calibrated");
    }
  }

  // ---- reference and controller output ----------------------------------------------------
  float u = 0.0f;
  float ref = 0.0f;
  float v_log = 0.0f;
  const float t_run = (float)((uint32_t)(now_us - S.t_start_us)) * 1e-6f;
  if (S.state == ST_RUNNING) {
    if (S.deadman_released_steps >= 3) {
      enterIdle("dead-man released");
    } else {
      bool finished = false;
      switch (S.mode) {
        case MODE_TRAP:
          ref = trapezoidRef(t_run, S.amp, TRAP_REST0_S, TRAP_RAMP_S, TRAP_HOLD_S, TRAP_REST_S, TRAP_REPS);
          u = g_ctrl->update(ref, force);
          v_log = g_ctrl->v();
          finished = t_run > trapezoidDuration(TRAP_REST0_S, TRAP_RAMP_S, TRAP_HOLD_S, TRAP_REST_S, TRAP_REPS) - TRAP_REST_S + 1.0f;
          break;
        case MODE_HOLD:
          ref = holdRef(t_run, S.amp, TRAP_REST0_S, TRAP_RAMP_S, S.hold_s);
          u = g_ctrl->update(ref, force);
          v_log = g_ctrl->v();
          finished = t_run > TRAP_REST0_S + TRAP_RAMP_S + S.hold_s + 1.5f;
          break;
        case MODE_MIRROR: {
          const float frac = g_emg_norm.fraction(emg_env, MIRROR_DEADBAND);
          const float target = frac * MIRROR_FMAX_N;
          const float step = MIRROR_REF_RATE * CONTROL_TS_S;
          float d = target - S.mirror_ref;
          if (d > step) d = step;
          if (d < -step) d = -step;
          S.mirror_ref += d;
          ref = S.mirror_ref;
          u = g_ctrl->update(ref, force);
          v_log = g_ctrl->v();
          break;
        }
        case MODE_STAIRCASE: {
          bool done = false;
          u = staircaseU(t_run, STAIRCASE_U_MAX, STAIRCASE_STEPS, STAIRCASE_HOLD_S, STAIRCASE_REST_S, &done);
          finished = done;
          break;
        }
        case MODE_STEP:
          ref = stepV(t_run, S.step_lo, S.step_hi, STEP_T_STEP_S);
          u = g_ctrl->openLoop(ref);
          v_log = ref;
          finished = t_run > STEP_TOTAL_S;
          break;
        default:
          break;
      }
      if (finished) {
        u = 0.0f;
        enterIdle("profile finished");
      }
    }
  }
  S.ref = ref;

  // ---- safety -----------------------------------------------------------------------------
  if (S.state == ST_ARMED || S.state == ST_RUNNING) {
    SafetyInputs in;
    in.t_s = (float)((uint32_t)(now_us - S.t_arm_us)) * 1e-6f;
    in.running = (S.state == ST_RUNNING);
    in.u = u;
    in.i_cmd_ma = u * I_MAX_MA;
    in.i_peak_ma = i_peak;
    in.v_rail_v = v_rail;
    in.t_skin_c = t_skin;
    in.temp_sensor_ok = temp_ok;
    in.hr_valid = hr_valid;
    in.hr_bpm = hr_bpm;
    in.force_n = force;
    in.force_sensor_ok = force_ok;
    in.emg_required = (S.mode == MODE_MIRROR);
    in.emg_leadoff = leadoff;
    in.loop_age_ms = loop_age_ms;
    if (S.state == ST_ARMED) S.t_skin_start = t_skin;        // reference temperature is the one at ARM
    in.t_skin_start_c = S.t_skin_start;
    const uint32_t f = g_safety.evaluate(in);
    if (f != 0) {
      u = 0.0f;
      enterFault(f);
    }
  }

  // ---- outputs ----------------------------------------------------------------------------
  const bool run = (S.state == ST_RUNNING);
  g_u_cmd = run ? u : 0.0f;
  g_stim_on = run;

  // ---- status LED -------------------------------------------------------------------------
  const uint32_t blink_div = (S.state == ST_IDLE) ? 50 : (S.state == ST_ARMED) ? 10 : (S.state == ST_FAULT) ? 3 : 0;
  digitalWrite(PIN_STATUS_LED, blink_div == 0 ? HIGH : ((S.step_count / blink_div) & 1) ? HIGH : LOW);

  // ---- telemetry: 100 Hz while armed/running, 5 Hz otherwise -------------------------------
  if (S.state == ST_ARMED || S.state == ST_RUNNING || (S.step_count % 20) == 0) {
    const float emg_pct = g_emg_norm.calibrated() ? g_emg_norm.fraction(emg_env, 0.0f) * 100.0f : NAN;
    char line[128];
    const int n = snprintf(line, sizeof(line), "%lu,%d,%lu,%.3f,%.3f,%.1f,%.4f,%.3f,%.1f,%.2f,%.1f,%.0f\n",
                           (unsigned long)millis(), (int)S.state, (unsigned long)g_safety.faults(),
                           ref, force, emg_pct, u, v_log, i_peak, t_skin, v_rail, hr_valid ? hr_bpm : NAN);
    if (n > 0) Serial.write((const uint8_t*)line, (size_t)((n < (int)sizeof(line)) ? n : (int)sizeof(line) - 1));
  }
}

static void controlTask(void*) {
  TickType_t last = xTaskGetTickCount();
  TickType_t period = pdMS_TO_TICKS((uint32_t)(CONTROL_TS_S * 1000.0f));
  if (period < 1) period = 1;
  for (;;) {
    vTaskDelayUntil(&last, period);
    controlStep();
  }
}

// ============================================================================================
// serial command parser (runs in loop(), pushes commands to the control task)
// ============================================================================================
static void sendCmd(uint8_t type, uint8_t mode, uint8_t which, float a, float b) {
  Cmd c;
  c.type = type;
  c.mode = mode;
  c.which = which;
  c.a = a;
  c.b = b;
  if (xQueueSend(g_cmd_queue, &c, 0) != pdTRUE) logMsg("ERR command queue full");
}

static void printHelp() {
  logMsg("Commands: HELP STATUS HEADER TARE ADC ARM STOP RESET");
  logMsg("  START TRAP [amp_N] | START HOLD amp_N sec | START MIRROR | START STAIRCASE | START STEP [lo hi]");
  logMsg("  CAL EMG REST | CAL EMG MVC | SET KP|KI|KD|TF value");
}

static void parseLine(char* line) {
  for (char* p = line; *p; ++p) *p = (char)toupper((unsigned char)*p);
  char* save = nullptr;
  char* t0 = strtok_r(line, " \t\r", &save);
  if (!t0) return;
  char* t1 = strtok_r(nullptr, " \t\r", &save);
  char* t2 = strtok_r(nullptr, " \t\r", &save);
  char* t3 = strtok_r(nullptr, " \t\r", &save);

  if (!strcmp(t0, "HELP")) printHelp();
  else if (!strcmp(t0, "STATUS")) sendCmd(C_STATUS, 0, 0, 0, 0);
  else if (!strcmp(t0, "HEADER")) sendCmd(C_HEADER, 0, 0, 0, 0);
  else if (!strcmp(t0, "TARE")) sendCmd(C_TARE, 0, 0, 0, 0);
  else if (!strcmp(t0, "ADC")) sendCmd(C_ADC, 0, 0, 0, 0);
  else if (!strcmp(t0, "ARM")) sendCmd(C_ARM, 0, 0, 0, 0);
  else if (!strcmp(t0, "STOP")) sendCmd(C_STOP, 0, 0, 0, 0);
  else if (!strcmp(t0, "RESET")) sendCmd(C_RESET, 0, 0, 0, 0);
  else if (!strcmp(t0, "START") && t1) {
    if (!strcmp(t1, "TRAP")) sendCmd(C_START, MODE_TRAP, 0, t2 ? (float)atof(t2) : TRAP_DEFAULT_AMP_N, 0);
    else if (!strcmp(t1, "HOLD") && t2 && t3) sendCmd(C_START, MODE_HOLD, 0, (float)atof(t2), (float)atof(t3));
    else if (!strcmp(t1, "MIRROR")) sendCmd(C_START, MODE_MIRROR, 0, 0, 0);
    else if (!strcmp(t1, "STAIRCASE")) sendCmd(C_START, MODE_STAIRCASE, 0, 0, 0);
    else if (!strcmp(t1, "STEP")) sendCmd(C_START, MODE_STEP, 0, t2 ? (float)atof(t2) : STEP_V_LO_N, t3 ? (float)atof(t3) : STEP_V_HI_N);
    else logMsg("ERR bad START arguments, see HELP");
  } else if (!strcmp(t0, "CAL") && t1 && !strcmp(t1, "EMG") && t2) {
    if (!strcmp(t2, "REST")) sendCmd(C_CAL_REST, 0, 0, 0, 0);
    else if (!strcmp(t2, "MVC")) sendCmd(C_CAL_MVC, 0, 0, 0, 0);
    else logMsg("ERR CAL EMG REST or MVC");
  } else if (!strcmp(t0, "SET") && t1 && t2) {
    const float v = (float)atof(t2);
    if (v < 0.0f || v > 100.0f) { logMsg("ERR value out of range"); return; }
    if (!strcmp(t1, "KP")) sendCmd(C_SET, 0, S_KP, v, 0);
    else if (!strcmp(t1, "KI")) sendCmd(C_SET, 0, S_KI, v, 0);
    else if (!strcmp(t1, "KD")) sendCmd(C_SET, 0, S_KD, v, 0);
    else if (!strcmp(t1, "TF") && v > 0.0f) sendCmd(C_SET, 0, S_TF, v, 0);
    else logMsg("ERR SET KP|KI|KD|TF value");
  } else {
    logMsg("ERR unknown command, type HELP");
  }
}

// ============================================================================================
// setup / loop
// ============================================================================================
void setup() {
  // outputs to a safe state before anything else
  pinMode(PIN_PHASE_A, OUTPUT);
  pinMode(PIN_PHASE_B, OUTPUT);
  pinMode(PIN_STIM_ENABLE, OUTPUT);
  digitalWrite(PIN_PHASE_A, LOW);
  digitalWrite(PIN_PHASE_B, LOW);
  digitalWrite(PIN_STIM_ENABLE, LOW);
  pinMode(PIN_STATUS_LED, OUTPUT);
  pinMode(PIN_DEADMAN, INPUT_PULLUP);
#if USE_EMG_LEADOFF
  if (PIN_EMG_LO_PLUS >= 0) pinMode(PIN_EMG_LO_PLUS, INPUT);
  if (PIN_EMG_LO_MINUS >= 0) pinMode(PIN_EMG_LO_MINUS, INPUT);
#endif

  Serial.begin(SERIAL_BAUD);
  delay(300);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_FSR_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_EMG_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_ISENSE_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_VRAIL_ADC, ADC_11db);
  analogSetPinAttenuation(PIN_NTC_ADC, ADC_11db);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(400000);
  g_i2c_mutex = xSemaphoreCreateMutex();
  g_cmd_queue = xQueueCreate(8, sizeof(Cmd));
  dacWrite(0, 50);                                   // current command to zero

  g_gains.kp = DEFAULT_KP;
  g_gains.ki = DEFAULT_KI;
  g_gains.kd = DEFAULT_KD;
  g_gains.tf = DEFAULT_TF;

  if (!g_fsr.set(FSR_LUT_ADC, FSR_LUT_N_FORCE, FSR_LUT_N)) logMsg("ERR force-sensor table in config.h is invalid");
  if (g_inv.set(RECRUIT_U, RECRUIT_F_N, RECRUIT_N)) {
    g_ctrl = new ForceController(&g_inv, CONTROL_TS_S, 1.0f, U_RATE_MAX_PER_S, REF_OFF_N);
    g_ctrl->setGains(g_gains);
  } else {
    logMsg("ERR recruitment table in config.h is invalid");
  }

  logMsg("muscle_stim_pid 1.0  I_MAX=%.0f mA  FSR_CALIBRATED=%d  RECRUIT_IDENTIFIED=%d",
         I_MAX_MA, (int)FSR_CALIBRATED, (int)RECRUIT_IDENTIFIED);
  logMsg("Read SAFETY.md. Type HELP.");
  Serial.println("t_ms,state,faults,ref_N,force_N,emg_pct,u,v_N,i_peak_mA,t_skin_C,v_rail_V,hr_bpm");

  xTaskCreatePinnedToCore(pulseTask, "pulse", 4096, nullptr, 10, nullptr, 0);
  xTaskCreatePinnedToCore(emgTask, "emg", 4096, nullptr, 4, nullptr, 0);
#if USE_HEART_RATE
  xTaskCreatePinnedToCore(hrTask, "hr", 4096, nullptr, 2, nullptr, 0);
#endif
  xTaskCreatePinnedToCore(controlTask, "control", 8192, nullptr, 5, nullptr, 1);
}

void loop() {
  static char buf[96];
  static int n = 0;
  while (Serial.available() > 0) {
    const int ch = Serial.read();
    if (ch == '\n') {
      buf[n] = '\0';
      n = 0;
      parseLine(buf);
    } else if (ch != '\r' && n < (int)sizeof(buf) - 1) {
      buf[n++] = (char)ch;
    }
  }
  delay(2);
}
