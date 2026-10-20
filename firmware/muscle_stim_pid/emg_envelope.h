// emg_envelope.h - EMG amplitude envelope and normalisation to % of maximum voluntary contraction.
//
// Chain: remove the sensor's DC offset (slow tracking mean) -> rectify -> two cascaded first-order
// low-pass filters (about 3 Hz) = mean absolute value envelope in ADC counts.
// Normalisation: pct = (env - rest) / (mvc - rest), clipped to 0..1, with a small dead band.
#pragma once

#include <math.h>

class EmgEnvelope {
 public:
  EmgEnvelope(float fs_hz, float dc_tc_s, float lp_hz) {
    a_dc_ = 1.0f - expf(-1.0f / (fs_hz * dc_tc_s));
    a_lp_ = 1.0f - expf(-2.0f * 3.14159265f * lp_hz / fs_hz);
  }

  // x: raw ADC counts. Returns the envelope in counts.
  float update(float x) {
    if (!init_) {
      dc_ = x;
      init_ = true;
    }
    dc_ += a_dc_ * (x - dc_);
    const float r = fabsf(x - dc_);
    e1_ += a_lp_ * (r - e1_);
    e2_ += a_lp_ * (e1_ - e2_);
    return e2_;
  }

  float value() const { return e2_; }
  void reset() { init_ = false; e1_ = e2_ = 0.0f; }

 private:
  float a_dc_, a_lp_;
  float dc_ = 0.0f, e1_ = 0.0f, e2_ = 0.0f;
  bool init_ = false;
};

class EmgNormalizer {
 public:
  void setRest(float env) { rest_ = env; have_rest_ = true; }
  void setMvc(float env) { mvc_ = env; have_mvc_ = true; }
  bool calibrated() const { return have_rest_ && have_mvc_ && mvc_ > rest_ * 1.5f + 1.0f; }

  // returns 0..1 (fraction of MVC); values below the dead band count as rest
  float fraction(float env, float deadband) const {
    if (!calibrated()) return 0.0f;
    float p = (env - rest_) / (mvc_ - rest_);
    if (p < deadband) return 0.0f;
    return (p > 1.0f) ? 1.0f : p;
  }

  float rest() const { return rest_; }
  float mvc() const { return mvc_; }

 private:
  float rest_ = 0.0f, mvc_ = 0.0f;
  bool have_rest_ = false, have_mvc_ = false;
};
