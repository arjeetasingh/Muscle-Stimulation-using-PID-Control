// fsr_calibration.h - raw ADC counts of the force sensor -> newtons, with a validity check.
#pragma once

#include <math.h>

class FsrCalibration {
 public:
  // adc[] must be strictly increasing, force[] non-decreasing. Returns false for a bad table.
  bool set(const float* adc, const float* force_n, int n) {
    if (n < 2 || n > kMax) return false;
    for (int i = 1; i < n; ++i)
      if (!(adc[i] > adc[i - 1]) || force_n[i] < force_n[i - 1]) return false;
    for (int i = 0; i < n; ++i) {
      adc_[i] = adc[i];
      f_[i] = force_n[i];
    }
    n_ = n;
    return true;
  }

  // Linear interpolation. Below the first point the force is 0, above the last point it is
  // clamped to the last force; saturated() tells the caller about that.
  float newtons(float counts) const {
    if (n_ < 2) return NAN;
    if (counts <= adc_[0]) return 0.0f;
    if (counts >= adc_[n_ - 1]) return f_[n_ - 1];
    int i = 1;
    while (i < n_ - 1 && adc_[i] < counts) ++i;
    const float t = (counts - adc_[i - 1]) / (adc_[i] - adc_[i - 1]);
    return f_[i - 1] + t * (f_[i] - f_[i - 1]);
  }

  // true if the reading is outside the calibrated range (sensor disconnected, shorted, overloaded)
  bool outOfRange(float counts) const {
    return n_ < 2 || counts > adc_[n_ - 1] + 50.0f || counts < 20.0f;
  }

  float maxForce() const { return n_ ? f_[n_ - 1] : 0.0f; }

 private:
  static constexpr int kMax = 32;
  float adc_[kMax];
  float f_[kMax];
  int n_ = 0;
};
