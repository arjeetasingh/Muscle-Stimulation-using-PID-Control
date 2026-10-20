// profiles.h - reference profiles and identification test sequences as pure functions of time.
// Same shapes as python/muscle_stim/scenarios.py.
#pragma once

#include <math.h>

// Repeated contractions: rest, ramp up, hold, ramp down, rest. Output: force reference in N.
inline float trapezoidRef(float t, float amp, float rest0, float ramp, float hold, float rest, int reps) {
  const float period = ramp + hold + ramp + rest;
  const float tt = t - rest0;
  if (tt < 0.0f || tt >= reps * period) return 0.0f;
  const float x = fmodf(tt, period);
  if (x < ramp) return amp * x / ramp;
  if (x < ramp + hold) return amp;
  if (x < 2.0f * ramp + hold) return amp * (1.0f - (x - ramp - hold) / ramp);
  return 0.0f;
}

inline float trapezoidDuration(float rest0, float ramp, float hold, float rest, int reps) {
  return rest0 + reps * (2.0f * ramp + hold + rest);
}

// One constant hold: ramp up, hold for hold_s seconds, drop to zero.
inline float holdRef(float t, float amp, float rest0, float ramp, float hold_s) {
  const float tt = t - rest0;
  if (tt < 0.0f) return 0.0f;
  if (tt < ramp) return amp * tt / ramp;
  if (tt < ramp + hold_s) return amp;
  return 0.0f;
}

// Staircase test (open loop): rest, then for k = 1..steps: hold at u = k/steps * u_max, rest.
// Ascending order, which is the gentler order for a person. Returns u; sets *done at the end.
inline float staircaseU(float t, float u_max, int steps, float hold_s, float rest_s, bool* done) {
  const float cycle = hold_s + rest_s;
  const float tt = t - rest_s;                // initial rest
  *done = false;
  if (tt < 0.0f) return 0.0f;
  const int k = static_cast<int>(tt / cycle);
  if (k >= steps) {
    *done = true;
    return 0.0f;
  }
  const float within = tt - k * cycle;
  return (within < hold_s) ? u_max * static_cast<float>(k + 1) / static_cast<float>(steps) : 0.0f;
}

inline float staircaseDuration(int steps, float hold_s, float rest_s) {
  return rest_s + steps * (hold_s + rest_s);
}

// Force-demand step (open loop through the inverse recruitment): v_lo, then v_hi at t_step.
inline float stepV(float t, float v_lo, float v_hi, float t_step) { return (t < t_step) ? v_lo : v_hi; }
