// Reads "ref meas" pairs (one per line) on stdin, prints the controller output u for each.
// Gains: argv = kp ki kd tf. Uses the recruitment table of config.h. Used by the Python cross-check.
#include <cstdio>
#include <cstdlib>
#include "config.h"
#include "pid_controller.h"

int main(int argc, char** argv) {
  if (argc != 5) return 2;
  InverseRecruitment inv;
  if (!inv.set(RECRUIT_U, RECRUIT_F_N, RECRUIT_N)) return 3;
  ForceController c(&inv, CONTROL_TS_S, 1.0f, U_RATE_MAX_PER_S, REF_OFF_N);
  PidGains g;
  g.kp = (float)atof(argv[1]); g.ki = (float)atof(argv[2]); g.kd = (float)atof(argv[3]); g.tf = (float)atof(argv[4]);
  c.setGains(g);
  double ref, meas;
  while (std::scanf("%lf %lf", &ref, &meas) == 2) std::printf("%.9f\n", c.update((float)ref, (float)meas));
  return 0;
}
