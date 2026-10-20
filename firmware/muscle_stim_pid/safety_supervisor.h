// safety_supervisor.h - fault detection for the stimulator. Pure logic: time and measurements are
// passed in, nothing is read from hardware, so every rule is covered by a host unit test.
//
// Faults LATCH: once set they stay until reset() (command RESET, only accepted in IDLE).
// This is a software layer. It does NOT replace the hardware protections (current limit,
// DC-blocking capacitor, kill switch) described in SAFETY.md.
#pragma once

#include <stdint.h>
#include <math.h>

enum FaultBit : uint32_t {
  F_OVERCURRENT   = 1u << 0,   // measured pulse current above the limit
  F_OVERVOLTAGE   = 1u << 1,   // stimulation rail above the limit
  F_OVERTEMP      = 1u << 2,   // skin too warm, or warming too fast
  F_HR_RANGE      = 1u << 3,   // heart rate outside limits
  F_HR_MISSING    = 1u << 4,   // no valid heart rate for too long
  F_FORCE_LIMIT   = 1u << 5,   // force above the limit
  F_FORCE_SENSOR  = 1u << 6,   // force sensor out of range / disconnected
  F_EMG_LEADOFF   = 1u << 7,   // EMG electrode lifted (mirror mode)
  F_WATCHDOG      = 1u << 8,   // control loop did not run
  F_SESSION_TIME  = 1u << 9,   // session longer than allowed
  F_CONTRACTION   = 1u << 10,  // too much stimulation-on time without rest
  F_NO_CONTACT    = 1u << 11,  // current commanded but none flows (electrode off / cable broken)
  F_TEMP_SENSOR   = 1u << 12   // temperature sensor open or shorted
};

struct SafetyLimits {
  float i_peak_limit_ma;
  float v_rail_max_v;
  float t_skin_max_c;
  float t_skin_rise_max_c;
  bool  hr_required;
  float hr_min_bpm, hr_max_bpm;
  float hr_range_time_s, hr_missing_time_s;
  float force_limit_n;
  float session_max_s;
  float bucket_max_s, bucket_recovery_rate;
  float no_contact_time_s;
  float loop_late_ms;
};

struct SafetyInputs {
  float t_s;                 // time since the session started (s), monotonic
  bool  running;             // stimulation is commanded
  float u;                   // commanded intensity 0..1
  float i_cmd_ma;            // commanded pulse current
  float i_peak_ma;           // measured pulse current (0 if no pulses are being delivered)
  float v_rail_v;            // measured rail voltage
  float t_skin_c;            // measured skin temperature
  float t_skin_start_c;      // skin temperature when the session was armed
  bool  temp_sensor_ok;      // temperature reading plausible (sensor connected)
  bool  hr_valid;
  float hr_bpm;
  float force_n;
  bool  force_sensor_ok;
  bool  emg_required;        // mirror mode
  bool  emg_leadoff;
  float loop_age_ms;         // time since the control loop last ran
};

class SafetySupervisor {
 public:
  explicit SafetySupervisor(const SafetyLimits& lim) : lim_(lim) { reset(); }

  void reset() {
    faults_ = 0;
    last_t_ = -1.0f;
    hr_range_s_ = hr_missing_s_ = 0.0f;
    force_over_n_ = 0;
    sensor_bad_s_ = leadoff_s_ = no_contact_s_ = temp_bad_s_ = 0.0f;
    bucket_s_ = 0.0f;
    session_start_ = -1.0f;
  }

  uint32_t faults() const { return faults_; }

  // Call every control step. Returns the (latched) fault mask.
  uint32_t evaluate(const SafetyInputs& in) {
    const float dt = (last_t_ < 0.0f) ? 0.0f : (in.t_s - last_t_);
    last_t_ = in.t_s;

    // ---- checks that apply whenever the stimulation stage is armed -------------------------
    if (in.i_peak_ma > lim_.i_peak_limit_ma) faults_ |= F_OVERCURRENT;
    if (in.v_rail_v > lim_.v_rail_max_v) faults_ |= F_OVERVOLTAGE;
    if (in.t_skin_c > lim_.t_skin_max_c ||
        (in.t_skin_c - in.t_skin_start_c) > lim_.t_skin_rise_max_c)
      faults_ |= F_OVERTEMP;
    if (in.loop_age_ms > lim_.loop_late_ms) faults_ |= F_WATCHDOG;
    if (!in.temp_sensor_ok) {
      temp_bad_s_ += dt;
      if (temp_bad_s_ > 0.5f) faults_ |= F_TEMP_SENSOR;
    } else {
      temp_bad_s_ = 0.0f;
    }

    // force above the limit for 3 consecutive samples (one noisy sample must not trip it)
    if (in.force_n > lim_.force_limit_n) {
      if (++force_over_n_ >= 3) faults_ |= F_FORCE_LIMIT;
    } else {
      force_over_n_ = 0;
    }

    if (!in.running) {
      // while not running the timers are held; the contraction budget refills
      hr_range_s_ = hr_missing_s_ = no_contact_s_ = leadoff_s_ = 0.0f;
      bucket_s_ -= lim_.bucket_recovery_rate * dt;
      if (bucket_s_ < 0.0f) bucket_s_ = 0.0f;
      session_start_ = -1.0f;
      return faults_;
    }

    // ---- checks that apply only while running ----------------------------------------------
    if (session_start_ < 0.0f) session_start_ = in.t_s;
    if (in.t_s - session_start_ > lim_.session_max_s) faults_ |= F_SESSION_TIME;

    if (lim_.hr_required) {
      if (in.hr_valid) {
        hr_missing_s_ = 0.0f;
        if (in.hr_bpm < lim_.hr_min_bpm || in.hr_bpm > lim_.hr_max_bpm)
          hr_range_s_ += dt;
        else
          hr_range_s_ = 0.0f;
      } else {
        hr_range_s_ = 0.0f;
        hr_missing_s_ += dt;
      }
      if (hr_range_s_ > lim_.hr_range_time_s) faults_ |= F_HR_RANGE;
      if (hr_missing_s_ > lim_.hr_missing_time_s) faults_ |= F_HR_MISSING;
    }

    if (!in.force_sensor_ok) {
      sensor_bad_s_ += dt;
      if (sensor_bad_s_ > 0.1f) faults_ |= F_FORCE_SENSOR;
    } else {
      sensor_bad_s_ = 0.0f;
    }

    if (in.emg_required && in.emg_leadoff) {
      leadoff_s_ += dt;
      if (leadoff_s_ > 0.2f) faults_ |= F_EMG_LEADOFF;
    } else {
      leadoff_s_ = 0.0f;
    }

    // leaky bucket of stimulation-on time: +1 per second while on, -recovery per second while off
    if (in.u > 0.01f)
      bucket_s_ += dt;
    else
      bucket_s_ = fmaxf(0.0f, bucket_s_ - lim_.bucket_recovery_rate * dt);
    if (bucket_s_ > lim_.bucket_max_s) faults_ |= F_CONTRACTION;

    // current requested (> 5 mA) but less than 30 % of it measured, for too long
    if (in.i_cmd_ma > 5.0f && in.i_peak_ma < 0.3f * in.i_cmd_ma) {
      no_contact_s_ += dt;
      if (no_contact_s_ > lim_.no_contact_time_s) faults_ |= F_NO_CONTACT;
    } else {
      no_contact_s_ = 0.0f;
    }
    return faults_;
  }

  // short text for the log, e.g. "OVERCURRENT|HR_RANGE"
  static const char* describe(uint32_t mask, char* buf, int n) {
    static const char* names[13] = {"OVERCURRENT", "OVERVOLTAGE", "OVERTEMP", "HR_RANGE", "HR_MISSING",
                                    "FORCE_LIMIT", "FORCE_SENSOR", "EMG_LEADOFF", "WATCHDOG",
                                    "SESSION_TIME", "CONTRACTION", "NO_CONTACT", "TEMP_SENSOR"};
    int pos = 0;
    buf[0] = '\0';
    for (int i = 0; i < 13; ++i) {
      if (!(mask & (1u << i))) continue;
      const char* s = names[i];
      if (pos > 0 && pos < n - 1) buf[pos++] = '|';
      while (*s && pos < n - 1) buf[pos++] = *s++;
      buf[pos] = '\0';
    }
    return buf;
  }

 private:
  SafetyLimits lim_;
  uint32_t faults_;
  float last_t_;
  float hr_range_s_, hr_missing_s_, sensor_bad_s_, leadoff_s_, no_contact_s_, temp_bad_s_, bucket_s_;
  int force_over_n_;
  float session_start_;
};
