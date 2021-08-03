/* Produced by CVXGEN, 2021-01-21 06:01:50 -0500.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: matrix_support.c. */
/* Description: Support functions for matrix multiplication and vector filling. */
#include "solver.h"
void multbymA(double *lhs, double *rhs) {
}
void multbymAT(double *lhs, double *rhs) {
  lhs[0] = 0;
  lhs[1] = 0;
  lhs[2] = 0;
  lhs[3] = 0;
  lhs[4] = 0;
  lhs[5] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[0]+(params.sigma[1]-params.sigma_0[1])*params.J[1]+(params.sigma[2]-params.sigma_0[2])*params.J[2]+(params.sigma[3]-params.sigma_0[3])*params.J[3]+(params.sigma[4]-params.sigma_0[4])*params.J[4]+(params.sigma[5]-params.sigma_0[5])*params.J[5]))-rhs[1]*(((params.sigma[0]-params.sigma_0[0])*params.J[6]+(params.sigma[1]-params.sigma_0[1])*params.J[7]+(params.sigma[2]-params.sigma_0[2])*params.J[8]+(params.sigma[3]-params.sigma_0[3])*params.J[9]+(params.sigma[4]-params.sigma_0[4])*params.J[10]+(params.sigma[5]-params.sigma_0[5])*params.J[11]))-rhs[2]*(((params.sigma[0]-params.sigma_0[0])*params.J[12]+(params.sigma[1]-params.sigma_0[1])*params.J[13]+(params.sigma[2]-params.sigma_0[2])*params.J[14]+(params.sigma[3]-params.sigma_0[3])*params.J[15]+(params.sigma[4]-params.sigma_0[4])*params.J[16]+(params.sigma[5]-params.sigma_0[5])*params.J[17]))-rhs[3]*(((params.sigma[0]-params.sigma_0[0])*params.J[18]+(params.sigma[1]-params.sigma_0[1])*params.J[19]+(params.sigma[2]-params.sigma_0[2])*params.J[20]+(params.sigma[3]-params.sigma_0[3])*params.J[21]+(params.sigma[4]-params.sigma_0[4])*params.J[22]+(params.sigma[5]-params.sigma_0[5])*params.J[23]))-rhs[4]*(((params.sigma[0]-params.sigma_0[0])*params.J[24]+(params.sigma[1]-params.sigma_0[1])*params.J[25]+(params.sigma[2]-params.sigma_0[2])*params.J[26]+(params.sigma[3]-params.sigma_0[3])*params.J[27]+(params.sigma[4]-params.sigma_0[4])*params.J[28]+(params.sigma[5]-params.sigma_0[5])*params.J[29]))-rhs[5]*(((params.sigma[0]-params.sigma_0[0])*params.J[30]+(params.sigma[1]-params.sigma_0[1])*params.J[31]+(params.sigma[2]-params.sigma_0[2])*params.J[32]+(params.sigma[3]-params.sigma_0[3])*params.J[33]+(params.sigma[4]-params.sigma_0[4])*params.J[34]+(params.sigma[5]-params.sigma_0[5])*params.J[35]));
  lhs[1] = -rhs[0]*(-1);
  lhs[2] = -rhs[1]*(-1);
  lhs[3] = -rhs[2]*(-1);
  lhs[4] = -rhs[3]*(-1);
  lhs[5] = -rhs[4]*(-1);
  lhs[6] = -rhs[5]*(-1);
  lhs[7] = -rhs[0]*(1);
  lhs[8] = -rhs[1]*(1);
  lhs[9] = -rhs[2]*(1);
  lhs[10] = -rhs[3]*(1);
  lhs[11] = -rhs[4]*(1);
  lhs[12] = -rhs[5]*(1);
  lhs[13] = -rhs[0]*(-1);
  lhs[14] = -rhs[1]*(-1);
  lhs[15] = -rhs[2]*(-1);
  lhs[16] = -rhs[3]*(-1);
  lhs[17] = -rhs[4]*(-1);
  lhs[18] = -rhs[5]*(-1);
  lhs[19] = -rhs[0]*(1);
  lhs[20] = -rhs[1]*(1);
  lhs[21] = -rhs[2]*(1);
  lhs[22] = -rhs[3]*(1);
  lhs[23] = -rhs[4]*(1);
  lhs[24] = -rhs[5]*(1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[0]+(params.sigma[1]-params.sigma_0[1])*params.J[1]+(params.sigma[2]-params.sigma_0[2])*params.J[2]+(params.sigma[3]-params.sigma_0[3])*params.J[3]+(params.sigma[4]-params.sigma_0[4])*params.J[4]+(params.sigma[5]-params.sigma_0[5])*params.J[5]))-rhs[1]*(-1)-rhs[7]*(1)-rhs[13]*(-1)-rhs[19]*(1);
  lhs[1] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[6]+(params.sigma[1]-params.sigma_0[1])*params.J[7]+(params.sigma[2]-params.sigma_0[2])*params.J[8]+(params.sigma[3]-params.sigma_0[3])*params.J[9]+(params.sigma[4]-params.sigma_0[4])*params.J[10]+(params.sigma[5]-params.sigma_0[5])*params.J[11]))-rhs[2]*(-1)-rhs[8]*(1)-rhs[14]*(-1)-rhs[20]*(1);
  lhs[2] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[12]+(params.sigma[1]-params.sigma_0[1])*params.J[13]+(params.sigma[2]-params.sigma_0[2])*params.J[14]+(params.sigma[3]-params.sigma_0[3])*params.J[15]+(params.sigma[4]-params.sigma_0[4])*params.J[16]+(params.sigma[5]-params.sigma_0[5])*params.J[17]))-rhs[3]*(-1)-rhs[9]*(1)-rhs[15]*(-1)-rhs[21]*(1);
  lhs[3] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[18]+(params.sigma[1]-params.sigma_0[1])*params.J[19]+(params.sigma[2]-params.sigma_0[2])*params.J[20]+(params.sigma[3]-params.sigma_0[3])*params.J[21]+(params.sigma[4]-params.sigma_0[4])*params.J[22]+(params.sigma[5]-params.sigma_0[5])*params.J[23]))-rhs[4]*(-1)-rhs[10]*(1)-rhs[16]*(-1)-rhs[22]*(1);
  lhs[4] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[24]+(params.sigma[1]-params.sigma_0[1])*params.J[25]+(params.sigma[2]-params.sigma_0[2])*params.J[26]+(params.sigma[3]-params.sigma_0[3])*params.J[27]+(params.sigma[4]-params.sigma_0[4])*params.J[28]+(params.sigma[5]-params.sigma_0[5])*params.J[29]))-rhs[5]*(-1)-rhs[11]*(1)-rhs[17]*(-1)-rhs[23]*(1);
  lhs[5] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[30]+(params.sigma[1]-params.sigma_0[1])*params.J[31]+(params.sigma[2]-params.sigma_0[2])*params.J[32]+(params.sigma[3]-params.sigma_0[3])*params.J[33]+(params.sigma[4]-params.sigma_0[4])*params.J[34]+(params.sigma[5]-params.sigma_0[5])*params.J[35]))-rhs[6]*(-1)-rhs[12]*(1)-rhs[18]*(-1)-rhs[24]*(1);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2);
  lhs[1] = rhs[1]*(2);
  lhs[2] = rhs[2]*(2);
  lhs[3] = rhs[3]*(2);
  lhs[4] = rhs[4]*(2);
  lhs[5] = rhs[5]*(2);
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = 0;
  work.q[3] = 0;
  work.q[4] = 0;
  work.q[5] = 0;
}
void fillh(void) {
  work.h[0] = -(-params.h_goal[0]-((params.sigma[0]-params.sigma_0[0])*params.Sigma[0]+(params.sigma[1]-params.sigma_0[1])*params.Sigma[1]+(params.sigma[2]-params.sigma_0[2])*params.Sigma[2]+(params.sigma[3]-params.sigma_0[3])*params.Sigma[3]+(params.sigma[4]-params.sigma_0[4])*params.Sigma[4]+(params.sigma[5]-params.sigma_0[5])*params.Sigma[5]));
  work.h[1] = -params.dotq_min[0];
  work.h[2] = -params.dotq_min[0];
  work.h[3] = -params.dotq_min[0];
  work.h[4] = -params.dotq_min[0];
  work.h[5] = -params.dotq_min[0];
  work.h[6] = -params.dotq_min[0];
  work.h[7] = params.dotq_max[0];
  work.h[8] = params.dotq_max[0];
  work.h[9] = params.dotq_max[0];
  work.h[10] = params.dotq_max[0];
  work.h[11] = params.dotq_max[0];
  work.h[12] = params.dotq_max[0];
  work.h[13] = -(-params.a_max[0]+params.dotq_prev[0]);
  work.h[14] = -(-params.a_max[1]+params.dotq_prev[1]);
  work.h[15] = -(-params.a_max[2]+params.dotq_prev[2]);
  work.h[16] = -(-params.a_max[3]+params.dotq_prev[3]);
  work.h[17] = -(-params.a_max[4]+params.dotq_prev[4]);
  work.h[18] = -(-params.a_max[5]+params.dotq_prev[5]);
  work.h[19] = -(-params.dotq_prev[0]-params.a_max[0]);
  work.h[20] = -(-params.dotq_prev[1]-params.a_max[1]);
  work.h[21] = -(-params.dotq_prev[2]-params.a_max[2]);
  work.h[22] = -(-params.dotq_prev[3]-params.a_max[3]);
  work.h[23] = -(-params.dotq_prev[4]-params.a_max[4]);
  work.h[24] = -(-params.dotq_prev[5]-params.a_max[5]);
}
void fillb(void) {
}
void pre_ops(void) {
}
