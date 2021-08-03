/* Produced by CVXGEN, 2021-01-22 10:01:18 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.sigma[0] = 0.20319161029830202;
  params.sigma[1] = 0.8325912904724193;
  params.sigma[2] = -0.8363810443482227;
  params.sigma[3] = 0.04331042079065206;
  params.sigma[4] = 1.5717878173906188;
  params.sigma[5] = 1.5851723557337523;
  params.sigma_0[0] = -1.497658758144655;
  params.sigma_0[1] = -1.171028487447253;
  params.sigma_0[2] = -1.7941311867966805;
  params.sigma_0[3] = -0.23676062539745413;
  params.sigma_0[4] = -1.8804951564857322;
  params.sigma_0[5] = -0.17266710242115568;
  params.J[0] = 0.596576190459043;
  params.J[1] = -0.8860508694080989;
  params.J[2] = 0.7050196079205251;
  params.J[3] = 0.3634512696654033;
  params.J[4] = -1.9040724704913385;
  params.J[5] = 0.23541635196352795;
  params.J[6] = -0.9629902123701384;
  params.J[7] = -0.3395952119597214;
  params.J[8] = -0.865899672914725;
  params.J[9] = 0.7725516732519853;
  params.J[10] = -0.23818512931704205;
  params.J[11] = -1.372529046100147;
  params.J[12] = 0.17859607212737894;
  params.J[13] = 1.1212590580454682;
  params.J[14] = -0.774545870495281;
  params.J[15] = -1.1121684642712744;
  params.J[16] = -0.44811496977740495;
  params.J[17] = 1.7455345994417217;
  params.J[18] = 1.9039816898917352;
  params.J[19] = 0.6895347036512547;
  params.J[20] = 1.6113364341535923;
  params.J[21] = 1.383003485172717;
  params.J[22] = -0.48802383468444344;
  params.J[23] = -1.631131964513103;
  params.J[24] = 0.6136436100941447;
  params.J[25] = 0.2313630495538037;
  params.J[26] = -0.5537409477496875;
  params.J[27] = -1.0997819806406723;
  params.J[28] = -0.3739203344950055;
  params.J[29] = -0.12423900520332376;
  params.J[30] = -0.923057686995755;
  params.J[31] = -0.8328289030982696;
  params.J[32] = -0.16925440270808823;
  params.J[33] = 1.442135651787706;
  params.J[34] = 0.34501161787128565;
  params.J[35] = -0.8660485502711608;
  params.h_goal[0] = -0.8880899735055947;
  params.Sigma[0] = -0.1815116979122129;
  params.Sigma[1] = -1.17835862158005;
  params.Sigma[2] = -1.1944851558277074;
  params.Sigma[3] = 0.05614023926976763;
  params.Sigma[4] = -1.6510825248767813;
  params.Sigma[5] = -0.06565787059365391;
  params.dotq_min[0] = -0.5512951504486665;
  params.dotq_min[1] = 0.8307464872626844;
  params.dotq_min[2] = 0.9869848924080182;
  params.dotq_min[3] = 0.7643716874230573;
  params.dotq_min[4] = 0.7567216550196565;
  params.dotq_min[5] = -0.5055995034042868;
  params.dotq_max[0] = 0.6725392189410702;
  params.dotq_max[1] = -0.6406053441727284;
  params.dotq_max[2] = 0.29117547947550015;
  params.dotq_max[3] = -0.6967713677405021;
  params.dotq_max[4] = -0.21941980294587182;
  params.dotq_max[5] = -1.753884276680243;
  params.a_max[0] = -1.0292983112626475;
  params.a_max[1] = 1.8864104246942706;
  params.a_max[2] = -1.077663182579704;
  params.a_max[3] = 0.7659100437893209;
  params.a_max[4] = 0.6019074328549583;
  params.a_max[5] = 0.8957565577499285;
  params.dotq_prev[0] = -0.09964555746227477;
  params.dotq_prev[1] = 0.38665509840745127;
  params.dotq_prev[2] = -1.7321223042686946;
  params.dotq_prev[3] = -1.7097514487110663;
  params.dotq_prev[4] = -1.2040958948116867;
  params.dotq_prev[5] = -1.3925560119658358;
}
