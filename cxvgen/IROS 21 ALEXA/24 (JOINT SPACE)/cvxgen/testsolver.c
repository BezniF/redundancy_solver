/* Produced by CVXGEN, 2021-02-11 11:43:26 -0500.  */
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
  params.dotq_adm[0] = 0.20319161029830202;
  params.dotq_adm[1] = 0.8325912904724193;
  params.dotq_adm[2] = -0.8363810443482227;
  params.dotq_adm[3] = 0.04331042079065206;
  params.dotq_adm[4] = 1.5717878173906188;
  params.dotq_adm[5] = 1.5851723557337523;
  params.M[0] = 1.1255853104638363;
  params.M[1] = 1.2072428781381868;
  params.M[2] = 1.0514672033008299;
  params.M[3] = 1.4408098436506365;
  params.M[4] = 1.0298762108785668;
  params.M[5] = 1.456833224394711;
  params.l[0] = 1.2982880952295215;
  params.sigma[0] = -0.8860508694080989;
  params.sigma[1] = 0.7050196079205251;
  params.sigma[2] = 0.3634512696654033;
  params.sigma[3] = -1.9040724704913385;
  params.sigma[4] = 0.23541635196352795;
  params.sigma[5] = -0.9629902123701384;
  params.sigma_0[0] = -0.3395952119597214;
  params.sigma_0[1] = -0.865899672914725;
  params.sigma_0[2] = 0.7725516732519853;
  params.sigma_0[3] = -0.23818512931704205;
  params.sigma_0[4] = -1.372529046100147;
  params.sigma_0[5] = 0.17859607212737894;
  params.h_goal[0] = 1.1212590580454682;
  params.Sigma[0] = -0.774545870495281;
  params.Sigma[1] = -1.1121684642712744;
  params.Sigma[2] = -0.44811496977740495;
  params.Sigma[3] = 1.7455345994417217;
  params.Sigma[4] = 1.9039816898917352;
  params.Sigma[5] = 0.6895347036512547;
  params.theta[0] = 1.6113364341535923;
  params.theta[1] = 1.383003485172717;
  params.Q_lim[0] = -0.48802383468444344;
  params.Q_lim[1] = -1.631131964513103;
  params.Q_lim[2] = 0.6136436100941447;
  params.Q_lim[3] = 0.2313630495538037;
  params.Q_lim[4] = -0.5537409477496875;
  params.Q_lim[5] = -1.0997819806406723;
  params.h_lim[0] = -0.3739203344950055;
  params.h_lim[1] = -0.12423900520332376;
  params.h_lim[2] = -0.923057686995755;
  params.h_lim[3] = -0.8328289030982696;
  params.h_lim[4] = -0.16925440270808823;
  params.h_lim[5] = 1.442135651787706;
  params.x[0] = 0.34501161787128565;
  params.x[1] = -0.8660485502711608;
  params.x[2] = -0.8880899735055947;
  params.x[3] = -0.1815116979122129;
  params.x[4] = -1.17835862158005;
  params.x[5] = -1.1944851558277074;
  params.sigma_obs[0] = 0.05614023926976763;
  params.sigma_obs[1] = -1.6510825248767813;
  params.sigma_obs[2] = -0.06565787059365391;
  params.sigma_obs[3] = -0.5512951504486665;
  params.sigma_obs[4] = 0.8307464872626844;
  params.sigma_obs[5] = 0.9869848924080182;
  params.J[0] = 0.7643716874230573;
  params.J[1] = 0.7567216550196565;
  params.J[2] = -0.5055995034042868;
  params.J[3] = 0.6725392189410702;
  params.J[4] = -0.6406053441727284;
  params.J[5] = 0.29117547947550015;
  params.J[6] = -0.6967713677405021;
  params.J[7] = -0.21941980294587182;
  params.J[8] = -1.753884276680243;
  params.J[9] = -1.0292983112626475;
  params.J[10] = 1.8864104246942706;
  params.J[11] = -1.077663182579704;
  params.J[12] = 0.7659100437893209;
  params.J[13] = 0.6019074328549583;
  params.J[14] = 0.8957565577499285;
  params.J[15] = -0.09964555746227477;
  params.J[16] = 0.38665509840745127;
  params.J[17] = -1.7321223042686946;
  params.J[18] = -1.7097514487110663;
  params.J[19] = -1.2040958948116867;
  params.J[20] = -1.3925560119658358;
  params.J[21] = -1.5995826216742213;
  params.J[22] = -1.4828245415645833;
  params.J[23] = 0.21311092723061398;
  params.J[24] = -1.248740700304487;
  params.J[25] = 1.808404972124833;
  params.J[26] = 0.7264471152297065;
  params.J[27] = 0.16407869343908477;
  params.J[28] = 0.8287224032315907;
  params.J[29] = -0.9444533161899464;
  params.J[30] = 1.7069027370149112;
  params.J[31] = 1.3567722311998827;
  params.J[32] = 0.9052779937121489;
  params.J[33] = -0.07904017565835986;
  params.J[34] = 1.3684127435065871;
  params.J[35] = 0.979009293697437;
  params.h_safe[0] = 0.6413036255984501;
  params.sigma_obs2[0] = 1.6559010680237511;
  params.sigma_obs2[1] = 0.5346622551502991;
  params.sigma_obs2[2] = -0.5362376605895625;
  params.sigma_obs2[3] = 0.2113782926017822;
  params.sigma_obs2[4] = -1.2144776931994525;
  params.sigma_obs2[5] = -1.2317108144255875;
  params.h_safe_2[0] = 0.9026784957312834;
  params.dotq_min[0] = 1.1397468137245244;
  params.dotq_max[0] = 1.8883934547350631;
  params.a_max[0] = 1.4038856681660068;
  params.a_max[1] = 0.17437730638329096;
  params.a_max[2] = -1.6408365219077408;
  params.a_max[3] = -0.04450702153554875;
  params.a_max[4] = 1.7117453902485025;
  params.a_max[5] = 1.1504727980139053;
  params.dotq_prev[0] = -0.05962309578364744;
  params.dotq_prev[1] = -0.1788825540764547;
  params.dotq_prev[2] = -1.1280569263625857;
  params.dotq_prev[3] = -1.2911464767927057;
  params.dotq_prev[4] = -1.7055053231225696;
  params.dotq_prev[5] = 1.56957275034837;
}
