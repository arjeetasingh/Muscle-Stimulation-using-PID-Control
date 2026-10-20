// Host unit tests of the portable firmware core (no Arduino needed).
//   g++ -std=c++17 -Wall -Wextra -I ../muscle_stim_pid test_core.cpp -o test_core && ./test_core
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "config.h"
#include "emg_envelope.h"
#include "fsr_calibration.h"
#include "pid_controller.h"
#include "profiles.h"
#include "safety_supervisor.h"

static int g_fail = 0, g_n = 0;
#define CHECK(cond)                                                      \
  do {                                                                   \
    ++g_n;                                                               \
    if (!(cond)) {                                                       \
      ++g_fail;                                                          \
      std::printf("FAIL line %d: %s\n", __LINE__, #cond);                \
    }                                                                    \
  } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((double)(a) - (double)(b)) <= (tol))

static SafetyLimits limits() {
  SafetyLimits l;
  l.i_peak_limit_ma = I_PEAK_LIMIT_MA; l.v_rail_max_v = V_RAIL_MAX_V;
  l.t_skin_max_c = T_SKIN_MAX_C; l.t_skin_rise_max_c = T_SKIN_RISE_MAX_C;
  l.hr_required = true; l.hr_min_bpm = HR_MIN_BPM; l.hr_max_bpm = HR_MAX_BPM;
  l.hr_range_time_s = HR_RANGE_TIME_S; l.hr_missing_time_s = HR_MISSING_TIME_S;
  l.force_limit_n = F_LIMIT_N; l.session_max_s = SESSION_MAX_S;
  l.bucket_max_s = ACTIVE_BUCKET_MAX_S; l.bucket_recovery_rate = ACTIVE_RECOVERY_RATE;
  l.no_contact_time_s = NO_CONTACT_TIME_S; l.loop_late_ms = LOOP_LATE_MS;
  return l;
}
static SafetyInputs good(float t) {
  SafetyInputs in{};
  in.t_s = t; in.running = true; in.u = 0.3f; in.i_cmd_ma = 9.0f; in.i_peak_ma = 9.0f;
  in.v_rail_v = 50.0f; in.t_skin_c = 30.0f; in.t_skin_start_c = 30.0f; in.temp_sensor_ok = true;
  in.hr_valid = true; in.hr_bpm = 70.0f; in.force_n = 3.0f; in.force_sensor_ok = true;
  in.emg_required = false; in.emg_leadoff = false; in.loop_age_ms = 10.0f;
  return in;
}
// run the supervisor for `secs` with a modifier applied to the inputs; returns final mask
template <class F>
static uint32_t run(SafetySupervisor& s, float secs, F mod, float t0 = 0.0f) {
  uint32_t m = 0;
  for (float t = 0; t <= secs; t += 0.01f) {
    SafetyInputs in = good(t0 + t);
    mod(in);
    m = s.evaluate(in);
  }
  return m;
}

static void test_inverse_recruitment() {
  InverseRecruitment inv;
  CHECK(inv.set(RECRUIT_U, RECRUIT_F_N, RECRUIT_N));
  for (int i = 0; i < RECRUIT_N; ++i) NEAR(inv.forceOfU(RECRUIT_U[i]), RECRUIT_F_N[i], 2e-3);
  for (float u = 0.0f; u <= 0.9f; u += 0.05f) NEAR(inv.uOfForce(inv.forceOfU(u)), u, 2e-3);
  CHECK(inv.uOfForce(-1.0f) == 0.0f);
  CHECK(inv.uOfForce(1e6f) == inv.uMax());
  const float bad_u[4] = {0.0f, 0.5f, 0.5f, 1.0f}, f[4] = {0, 1, 2, 3};
  InverseRecruitment b;
  CHECK(!b.set(bad_u, f, 4));
  CHECK(!b.set(bad_u, f, 2));
  const float flat_u[4] = {0, 0.3f, 0.6f, 1.0f}, flat_f[4] = {0, 5, 5, 5};   // flat top, must stay invertible
  CHECK(b.set(flat_u, flat_f, 4));
  CHECK(std::isfinite(b.uOfForce(5.0f)));
}

static void test_controller() {
  InverseRecruitment inv;
  inv.set(RECRUIT_U, RECRUIT_F_N, RECRUIT_N);
  ForceController c(&inv, 0.01f, 1.0f, 2.0f, 0.05f);
  PidGains g; g.kp = 0.53f; g.ki = 4.8f; g.kd = 0.0f; g.tf = 0.02f;
  c.setGains(g);
  // reference below the switch-off level: output stays zero even with a noisy measurement
  for (int i = 0; i < 100; ++i) CHECK(c.update(0.0f, (i % 2) ? 0.3f : -0.3f) == 0.0f);
  // output never exceeds the identified range and never goes negative, slew-limited
  float prev = 0.0f;
  for (int i = 0; i < 500; ++i) {
    const float u = c.update(8.0f, 0.0f);
    CHECK(u >= 0.0f && u <= inv.uMax() + 1e-6f);
    CHECK(std::fabs(u - prev) <= 2.0f * 0.01f + 1e-6f);
    prev = u;
  }
  // anti-windup: after a long saturated period the integrator recovers quickly when the measurement catches up
  c.reset(0.0f);
  for (int i = 0; i < 1000; ++i) c.update(30.0f, 0.0f);      // far above reachable force -> saturated
  // recovery with a simple lag plant (force follows the demand with a 50 ms lag)
  int steps = 0;
  float meas = 0.0f;
  for (; steps < 400; ++steps) {
    c.update(5.0f, meas);
    meas += 0.2f * (c.v() - meas);
    if (std::fabs(meas - 5.0f) < 0.5f) break;
  }
  CHECK(steps < 150);                                           // back on the reference within 1.5 s
  // open loop passes the demand through the inverse
  ForceController o(&inv, 0.01f, 1.0f, 2.0f, 0.05f);
  NEAR(inv.forceOfU(o.openLoop(4.0f)), 4.0f, 0.02);
}

static void test_fsr() {
  FsrCalibration f;
  CHECK(f.set(FSR_LUT_ADC, FSR_LUT_N_FORCE, FSR_LUT_N));
  NEAR(f.newtons(50.0f), 0.0, 1e-6);
  NEAR(f.newtons(500.0f), 0.5, 1e-5);
  NEAR(f.newtons(750.0f), 1.0, 1e-5);
  NEAR(f.newtons(1e5f), f.maxForce(), 1e-6);
  CHECK(f.outOfRange(5.0f));          // open circuit / short
  CHECK(f.outOfRange(4000.0f));       // above range
  CHECK(!f.outOfRange(1000.0f));
  const float a[3] = {1, 3, 2}, n[3] = {0, 1, 2};
  FsrCalibration g;
  CHECK(!g.set(a, n, 3));
  const float a2[3] = {1, 2, 3}, n2[3] = {0, 2, 1};
  CHECK(!g.set(a2, n2, 3));
}

static void test_emg() {
  EmgEnvelope e(1000, 0.5f, 3.0f);
  float env = 0.0f;
  // rest: constant offset 2048 with tiny noise
  for (int i = 0; i < 3000; ++i) env = e.update(2048.0f + ((i * 7) % 5 - 2));
  const float rest = env;
  CHECK(rest < 5.0f);
  // contraction: zero-mean oscillation of amplitude 200
  for (int i = 0; i < 3000; ++i) env = e.update(2048.0f + ((i % 4 < 2) ? 200.0f : -200.0f));
  CHECK(env > 100.0f);
  EmgNormalizer n;
  CHECK(!n.calibrated());
  n.setRest(rest);
  n.setMvc(env);
  CHECK(n.calibrated());
  NEAR(n.fraction(env, 0.05f), 1.0, 1e-4);
  CHECK(n.fraction(rest, 0.05f) == 0.0f);
  NEAR(n.fraction(rest + 0.5f * (env - rest), 0.05f), 0.5, 1e-3);
  CHECK(n.fraction(env * 3.0f, 0.05f) == 1.0f);
}

static void test_profiles() {
  NEAR(trapezoidRef(0.5f, 4, 1, 1.5f, 4, 8, 3), 0.0, 1e-6);
  NEAR(trapezoidRef(1.0f + 0.75f, 4, 1, 1.5f, 4, 8, 3), 2.0, 1e-5);
  NEAR(trapezoidRef(1.0f + 3.0f, 4, 1, 1.5f, 4, 8, 3), 4.0, 1e-6);
  NEAR(trapezoidRef(1.0f + 1.5f + 4.0f + 0.75f, 4, 1, 1.5f, 4, 8, 3), 2.0, 1e-5);
  NEAR(trapezoidRef(1.0f + 1.5f + 4.0f + 1.5f + 1.0f, 4, 1, 1.5f, 4, 8, 3), 0.0, 1e-6);
  NEAR(trapezoidDuration(1, 1.5f, 4, 8, 3), 1 + 3 * 15.0, 1e-5);
  NEAR(trapezoidRef(1000.0f, 4, 1, 1.5f, 4, 8, 3), 0.0, 1e-6);
  bool done = false;
  NEAR(staircaseU(1.0f, 0.6f, 10, 2, 8, &done), 0.0, 1e-6);
  CHECK(!done);
  NEAR(staircaseU(8.5f, 0.6f, 10, 2, 8, &done), 0.06, 1e-6);          // first hold
  NEAR(staircaseU(8.0f + 10.0f * 9 + 0.5f, 0.6f, 10, 2, 8, &done), 0.6, 1e-5);   // last hold
  NEAR(staircaseU(8.0f + 10.0f + 3.0f, 0.6f, 10, 2, 8, &done), 0.0, 1e-6);       // rest inside cycle
  staircaseU(staircaseDuration(10, 2, 8) + 0.1f, 0.6f, 10, 2, 8, &done);
  CHECK(done);
  NEAR(stepV(1.9f, 3, 6, 2.0f), 3.0, 1e-6);
  NEAR(stepV(2.0f, 3, 6, 2.0f), 6.0, 1e-6);
  NEAR(holdRef(1.0f + 5.0f, 4, 1, 1.5f, 10.0f), 4.0, 1e-6);
  NEAR(holdRef(1.0f + 1.5f + 10.0f + 0.1f, 4, 1, 1.5f, 10.0f), 0.0, 1e-6);
}

static void test_safety() {
  { SafetySupervisor s(limits()); CHECK(run(s, 10.0f, [](SafetyInputs&) {}) == 0); }
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.i_peak_ma = 40; }) & F_OVERCURRENT); }
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.v_rail_v = 80; }) & F_OVERVOLTAGE); }
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.t_skin_c = 41; }) & F_OVERTEMP); }
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.t_skin_c = 35; }) & F_OVERTEMP); }   // +5 rise
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.loop_age_ms = 80; }) & F_WATCHDOG); }
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.temp_sensor_ok = false; }) & F_TEMP_SENSOR); }
  { SafetySupervisor s(limits()); CHECK(run(s, 0.3f, [](SafetyInputs& i) { i.temp_sensor_ok = false; }) == 0); }   // brief glitch
  { SafetySupervisor s(limits()); CHECK(run(s, 5.0f, [](SafetyInputs& i) { i.hr_bpm = 160; }) & F_HR_RANGE); }
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.hr_bpm = 160; }) == 0); }          // shorter than limit
  { SafetySupervisor s(limits()); CHECK(run(s, 20.0f, [](SafetyInputs& i) { i.hr_valid = false; }) & F_HR_MISSING); }
  { SafetySupervisor s(limits()); CHECK(run(s, 5.0f, [](SafetyInputs& i) { i.hr_valid = false; }) == 0); }
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.force_n = 12; }) & F_FORCE_LIMIT); }
  {   // a single noisy sample must not trip the force limit
    SafetySupervisor s(limits());
    uint32_t m = 0;
    for (int k = 0; k < 100; ++k) { SafetyInputs in = good(k * 0.01f); if (k == 50) in.force_n = 12; m = s.evaluate(in); }
    CHECK(m == 0);
  }
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.force_sensor_ok = false; }) & F_FORCE_SENSOR); }
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.emg_required = true; i.emg_leadoff = true; }) & F_EMG_LEADOFF); }
  { SafetySupervisor s(limits()); CHECK(run(s, 1.0f, [](SafetyInputs& i) { i.emg_leadoff = true; }) == 0); }    // not mirror mode
  { SafetySupervisor s(limits()); CHECK(run(s, 3.0f, [](SafetyInputs& i) { i.i_peak_ma = 0; }) & F_NO_CONTACT); }
    { SafetySupervisor s(limits()); CHECK(run(s, 310.0f, [](SafetyInputs& i) { i.u = 0.0f; i.i_cmd_ma = 0.0f; i.i_peak_ma = 0.0f; }) & F_SESSION_TIME); }
  { SafetySupervisor s(limits()); CHECK(run(s, 10.0f, [](SafetyInputs&) {}) == 0); CHECK((run(s, 10.0f, [](SafetyInputs&) {}, 10.0f) & F_CONTRACTION)); }   // 20 s of continuous stimulation
  {   // latching and reset
    SafetySupervisor s(limits());
    run(s, 1.0f, [](SafetyInputs& i) { i.v_rail_v = 80; });
    CHECK(run(s, 5.0f, [](SafetyInputs&) {}, 2.0f) & F_OVERVOLTAGE);
    s.reset();
    CHECK(s.faults() == 0);
  }
  {   // not running: only the always-on checks apply
    SafetySupervisor s(limits());
    CHECK(run(s, 30.0f, [](SafetyInputs& i) { i.running = false; i.hr_valid = false; i.u = 0; i.i_cmd_ma = 0; i.i_peak_ma = 0; }) == 0);
  }
  char buf[96];
  SafetySupervisor::describe(F_OVERCURRENT | F_HR_RANGE, buf, sizeof(buf));
  CHECK(std::strcmp(buf, "OVERCURRENT|HR_RANGE") == 0);
}

static void test_config() {
  CHECK(STIM_FREQ_HZ * (2 * PULSE_WIDTH_US + INTERPHASE_US) * 1e-6f < 0.1f);
  // the DAC code for full scale stays within 12 bit
  const float code = I_MAX_MA * R_SENSE_OHM / 1000.0f / DAC_VREF_V * 4095.0f;
  CHECK(code <= 4095.0f && code > 0.0f);
  // charge per phase at I_MAX (microcoulomb): small, and balanced by construction (two equal phases)
  NEAR(I_MAX_MA * 1e-3 * PULSE_WIDTH_US, I_MAX_MA * 1e-3 * PULSE_WIDTH_US, 0);
  FsrCalibration f;
  CHECK(f.set(FSR_LUT_ADC, FSR_LUT_N_FORCE, FSR_LUT_N));
  InverseRecruitment inv;
  CHECK(inv.set(RECRUIT_U, RECRUIT_F_N, RECRUIT_N));
  CHECK(inv.forceOfU(inv.uMax()) > F_REF_MAX_N);   // identified range reaches the highest allowed reference
}

int main() {
  test_inverse_recruitment();
  test_controller();
  test_fsr();
  test_emg();
  test_profiles();
  test_safety();
  test_config();
  std::printf("%d checks, %d failed\n", g_n, g_fail);
  return g_fail ? 1 : 0;
}
