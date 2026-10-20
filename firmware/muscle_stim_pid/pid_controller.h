// pid_controller.h - force controller: PID in the force-demand domain + inverse recruitment.
//
// This file is a line-by-line twin of python/muscle_stim/controllers.py. The test in
// python/tests/test_firmware_crosscheck.py runs both with the same inputs and compares the output,
// so the controller that was designed and simulated is the one that runs on the ESP32.
//
// Units: Kp [N/N], Ki [1/s], Kd [s], tf [s], ts [s]; force demand v [N]; intensity u [0..1].
// No Arduino or ESP32 headers are used, so it also compiles and is tested on a PC.
#pragma once

#include <math.h>

constexpr int kMaxLut = 32;

// Monotone look-up table: steady force (N) of a fresh muscle at intensity u, and its inverse.
class InverseRecruitment {
 public:
  InverseRecruitment() : n_(0) {}

  // Returns false if the table is unusable (too short, or u not strictly increasing).
  bool set(const float* u, const float* force_n, int n) {
    if (n < 3 || n > kMaxLut) return false;
    for (int i = 1; i < n; ++i)
      if (!(u[i] > u[i - 1])) return false;
    for (int i = 0; i < n; ++i) {
      u_[i] = u[i];
      f_[i] = force_n[i];
      if (i > 0) {
        if (f_[i] < f_[i - 1]) f_[i] = f_[i - 1];                    // make non-decreasing
        if (f_[i] <= f_[i - 1]) f_[i] = f_[i - 1] + 1e-4f;           // strictly increasing
      }
    }
    n_ = n;
    return true;
  }

  bool ready() const { return n_ >= 3; }
  float uMax() const { return u_[n_ - 1]; }

  float forceOfU(float u) const { return interp(u_, f_, u); }
  float uOfForce(float v) const { return interp(f_, u_, v); }

  // Plain linear map u = v / f_full (used only for the "no inverse recruitment" comparison)
  bool setLinear(float f_full) {
    const float us[3] = {0.0f, 0.5f, 1.0f};
    const float fs[3] = {0.0f, 0.5f * f_full, f_full};
    return set(us, fs, 3);
  }

 private:
  float interp(const float* xs, const float* ys, float x) const {
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n_ - 1]) return ys[n_ - 1];
    int i = 1;
    while (i < n_ - 1 && xs[i] < x) ++i;
    const float dx = xs[i] - xs[i - 1];
    if (!(dx > 0.0f)) return ys[i];
    const float t = (x - xs[i - 1]) / dx;
    return ys[i - 1] + t * (ys[i] - ys[i - 1]);
  }
  float u_[kMaxLut];
  float f_[kMaxLut];
  int n_;
};

struct PidGains {
  float kp = 0.0f;  // N/N
  float ki = 0.0f;  // 1/s
  float kd = 0.0f;  // s
  float tf = 0.02f; // s, derivative filter time constant
};

class ForceController {
 public:
  ForceController(const InverseRecruitment* inv, float ts, float u_max, float u_rate_max, float ref_off)
      : inv_(inv), ts_(ts), u_rate_max_(u_rate_max), ref_off_(ref_off) {
    u_max_ = (u_max < inv->uMax()) ? u_max : inv->uMax();   // never beyond the identified range
    v_max_ = inv->forceOfU(u_max_);
    setGains(PidGains());
    reset(0.0f);
  }

  void setGains(const PidGains& g) {
    g_ = g;
    kaw_ = (g.kp > 0.0f && g.ki > 0.0f) ? g.ki / g.kp : 1.0f;   // back-calculation rate (1/s)
  }
  const PidGains& gains() const { return g_; }

  void reset(float meas) {
    i_state_ = 0.0f;
    d_state_ = 0.0f;
    meas_prev_ = meas;
    u_prev_ = 0.0f;
    v_ = 0.0f;
  }

  float v() const { return v_; }

  // Call once every ts seconds. Returns the stimulation intensity u in [0, u_max].
  float update(float ref, float meas) {
    if (ref < ref_off_) {
      // No contraction requested: stimulator off, states cleared (no wind-up, no noise-triggered pulses).
      i_state_ = 0.0f;
      d_state_ = 0.0f;
      meas_prev_ = meas;
      v_ = 0.0f;
      const float step_off = u_rate_max_ * ts_;
      u_prev_ = (u_prev_ - step_off > 0.0f) ? (u_prev_ - step_off) : 0.0f;
      return u_prev_;
    }
    const float e = ref - meas;
    // derivative on the measurement, backward-Euler first-order filter
    if (g_.kd > 0.0f)
      d_state_ = (g_.tf * d_state_ - g_.kd * (meas - meas_prev_)) / (g_.tf + ts_);
    else
      d_state_ = 0.0f;
    meas_prev_ = meas;

    const float v_unsat = ref + g_.kp * e + i_state_ + d_state_;
    float v = v_unsat;
    if (v > v_max_) v = v_max_;
    if (v < 0.0f) v = 0.0f;
    // integrator with back-calculation anti-windup
    i_state_ += ts_ * (g_.ki * e + kaw_ * (v - v_unsat));
    v_ = v;

    float u = inv_->uOfForce(v);
    const float step = u_rate_max_ * ts_;
    const float hi = u_prev_ + step;
    const float lo = u_prev_ - step;
    if (u > hi) u = hi;
    if (u < lo) u = lo;
    if (u > u_max_) u = u_max_;
    if (u < 0.0f) u = 0.0f;
    u_prev_ = u;
    return u;
  }

  // Open-loop use (identification tests): force demand v straight through the inverse recruitment.
  float openLoop(float v_demand) {
    float u = inv_->uOfForce(v_demand);
    if (u > u_max_) u = u_max_;
    if (u < 0.0f) u = 0.0f;
    u_prev_ = u;
    v_ = v_demand;
    return u;
  }

 private:
  const InverseRecruitment* inv_;
  PidGains g_;
  float ts_, u_max_, u_rate_max_, v_max_, ref_off_;
  float kaw_ = 1.0f;
  float i_state_ = 0.0f, d_state_ = 0.0f, meas_prev_ = 0.0f, u_prev_ = 0.0f, v_ = 0.0f;
};
