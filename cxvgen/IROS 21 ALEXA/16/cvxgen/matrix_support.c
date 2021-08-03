/* Produced by CVXGEN, 2021-01-26 06:22:59 -0500.  */
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
  lhs[6] = 0;
  lhs[7] = 0;
}
void multbymG(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.theta[0])-rhs[2]*(((params.sigma[0]-params.sigma_0[0])*params.J[0]+(params.sigma[1]-params.sigma_0[1])*params.J[1]+(params.sigma[2]-params.sigma_0[2])*params.J[2]+(params.sigma[3]-params.sigma_0[3])*params.J[3]+(params.sigma[4]-params.sigma_0[4])*params.J[4]+(params.sigma[5]-params.sigma_0[5])*params.J[5]))-rhs[3]*(((params.sigma[0]-params.sigma_0[0])*params.J[6]+(params.sigma[1]-params.sigma_0[1])*params.J[7]+(params.sigma[2]-params.sigma_0[2])*params.J[8]+(params.sigma[3]-params.sigma_0[3])*params.J[9]+(params.sigma[4]-params.sigma_0[4])*params.J[10]+(params.sigma[5]-params.sigma_0[5])*params.J[11]))-rhs[4]*(((params.sigma[0]-params.sigma_0[0])*params.J[12]+(params.sigma[1]-params.sigma_0[1])*params.J[13]+(params.sigma[2]-params.sigma_0[2])*params.J[14]+(params.sigma[3]-params.sigma_0[3])*params.J[15]+(params.sigma[4]-params.sigma_0[4])*params.J[16]+(params.sigma[5]-params.sigma_0[5])*params.J[17]))-rhs[5]*(((params.sigma[0]-params.sigma_0[0])*params.J[18]+(params.sigma[1]-params.sigma_0[1])*params.J[19]+(params.sigma[2]-params.sigma_0[2])*params.J[20]+(params.sigma[3]-params.sigma_0[3])*params.J[21]+(params.sigma[4]-params.sigma_0[4])*params.J[22]+(params.sigma[5]-params.sigma_0[5])*params.J[23]))-rhs[6]*(((params.sigma[0]-params.sigma_0[0])*params.J[24]+(params.sigma[1]-params.sigma_0[1])*params.J[25]+(params.sigma[2]-params.sigma_0[2])*params.J[26]+(params.sigma[3]-params.sigma_0[3])*params.J[27]+(params.sigma[4]-params.sigma_0[4])*params.J[28]+(params.sigma[5]-params.sigma_0[5])*params.J[29]))-rhs[7]*(((params.sigma[0]-params.sigma_0[0])*params.J[30]+(params.sigma[1]-params.sigma_0[1])*params.J[31]+(params.sigma[2]-params.sigma_0[2])*params.J[32]+(params.sigma[3]-params.sigma_0[3])*params.J[33]+(params.sigma[4]-params.sigma_0[4])*params.J[34]+(params.sigma[5]-params.sigma_0[5])*params.J[35]));
  lhs[1] = -rhs[2]*(-params.Q_lim[0]);
  lhs[2] = -rhs[3]*(-params.Q_lim[1]);
  lhs[3] = -rhs[4]*(-params.Q_lim[2]);
  lhs[4] = -rhs[5]*(-params.Q_lim[3]);
  lhs[5] = -rhs[6]*(-params.Q_lim[4]);
  lhs[6] = -rhs[7]*(-params.Q_lim[5]);
  lhs[7] = -rhs[1]*(-params.theta[1])-rhs[2]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[0]+(params.sigma[1]-params.sigma_obs[1])*params.J[1]+(params.sigma[2]-params.sigma_obs[2])*params.J[2]+(params.sigma[3]-params.sigma_obs[3])*params.J[3]+(params.sigma[4]-params.sigma_obs[4])*params.J[4]+(params.sigma[5]-params.sigma_obs[5])*params.J[5]))-rhs[3]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[6]+(params.sigma[1]-params.sigma_obs[1])*params.J[7]+(params.sigma[2]-params.sigma_obs[2])*params.J[8]+(params.sigma[3]-params.sigma_obs[3])*params.J[9]+(params.sigma[4]-params.sigma_obs[4])*params.J[10]+(params.sigma[5]-params.sigma_obs[5])*params.J[11]))-rhs[4]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[12]+(params.sigma[1]-params.sigma_obs[1])*params.J[13]+(params.sigma[2]-params.sigma_obs[2])*params.J[14]+(params.sigma[3]-params.sigma_obs[3])*params.J[15]+(params.sigma[4]-params.sigma_obs[4])*params.J[16]+(params.sigma[5]-params.sigma_obs[5])*params.J[17]))-rhs[5]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[18]+(params.sigma[1]-params.sigma_obs[1])*params.J[19]+(params.sigma[2]-params.sigma_obs[2])*params.J[20]+(params.sigma[3]-params.sigma_obs[3])*params.J[21]+(params.sigma[4]-params.sigma_obs[4])*params.J[22]+(params.sigma[5]-params.sigma_obs[5])*params.J[23]))-rhs[6]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[24]+(params.sigma[1]-params.sigma_obs[1])*params.J[25]+(params.sigma[2]-params.sigma_obs[2])*params.J[26]+(params.sigma[3]-params.sigma_obs[3])*params.J[27]+(params.sigma[4]-params.sigma_obs[4])*params.J[28]+(params.sigma[5]-params.sigma_obs[5])*params.J[29]))-rhs[7]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[30]+(params.sigma[1]-params.sigma_obs[1])*params.J[31]+(params.sigma[2]-params.sigma_obs[2])*params.J[32]+(params.sigma[3]-params.sigma_obs[3])*params.J[33]+(params.sigma[4]-params.sigma_obs[4])*params.J[34]+(params.sigma[5]-params.sigma_obs[5])*params.J[35]));
  lhs[8] = -rhs[2]*(params.A[0]*params.J[0]+params.A[1]*params.J[1]+params.A[2]*params.J[2]+params.A[3]*params.J[3]+params.A[4]*params.J[4]+params.A[5]*params.J[5])-rhs[3]*(params.A[0]*params.J[6]+params.A[1]*params.J[7]+params.A[2]*params.J[8]+params.A[3]*params.J[9]+params.A[4]*params.J[10]+params.A[5]*params.J[11])-rhs[4]*(params.A[0]*params.J[12]+params.A[1]*params.J[13]+params.A[2]*params.J[14]+params.A[3]*params.J[15]+params.A[4]*params.J[16]+params.A[5]*params.J[17])-rhs[5]*(params.A[0]*params.J[18]+params.A[1]*params.J[19]+params.A[2]*params.J[20]+params.A[3]*params.J[21]+params.A[4]*params.J[22]+params.A[5]*params.J[23])-rhs[6]*(params.A[0]*params.J[24]+params.A[1]*params.J[25]+params.A[2]*params.J[26]+params.A[3]*params.J[27]+params.A[4]*params.J[28]+params.A[5]*params.J[29])-rhs[7]*(params.A[0]*params.J[30]+params.A[1]*params.J[31]+params.A[2]*params.J[32]+params.A[3]*params.J[33]+params.A[4]*params.J[34]+params.A[5]*params.J[35]);
  lhs[9] = -rhs[2]*(-1);
  lhs[10] = -rhs[3]*(-1);
  lhs[11] = -rhs[4]*(-1);
  lhs[12] = -rhs[5]*(-1);
  lhs[13] = -rhs[6]*(-1);
  lhs[14] = -rhs[7]*(-1);
  lhs[15] = -rhs[2]*(1);
  lhs[16] = -rhs[3]*(1);
  lhs[17] = -rhs[4]*(1);
  lhs[18] = -rhs[5]*(1);
  lhs[19] = -rhs[6]*(1);
  lhs[20] = -rhs[7]*(1);
  lhs[21] = -rhs[2]*(-1);
  lhs[22] = -rhs[3]*(-1);
  lhs[23] = -rhs[4]*(-1);
  lhs[24] = -rhs[5]*(-1);
  lhs[25] = -rhs[6]*(-1);
  lhs[26] = -rhs[7]*(-1);
  lhs[27] = -rhs[2]*(1);
  lhs[28] = -rhs[3]*(1);
  lhs[29] = -rhs[4]*(1);
  lhs[30] = -rhs[5]*(1);
  lhs[31] = -rhs[6]*(1);
  lhs[32] = -rhs[7]*(1);
}
void multbymGT(double *lhs, double *rhs) {
  lhs[0] = -rhs[0]*(-params.theta[0]);
  lhs[1] = -rhs[7]*(-params.theta[1]);
  lhs[2] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[0]+(params.sigma[1]-params.sigma_0[1])*params.J[1]+(params.sigma[2]-params.sigma_0[2])*params.J[2]+(params.sigma[3]-params.sigma_0[3])*params.J[3]+(params.sigma[4]-params.sigma_0[4])*params.J[4]+(params.sigma[5]-params.sigma_0[5])*params.J[5]))-rhs[1]*(-params.Q_lim[0])-rhs[7]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[0]+(params.sigma[1]-params.sigma_obs[1])*params.J[1]+(params.sigma[2]-params.sigma_obs[2])*params.J[2]+(params.sigma[3]-params.sigma_obs[3])*params.J[3]+(params.sigma[4]-params.sigma_obs[4])*params.J[4]+(params.sigma[5]-params.sigma_obs[5])*params.J[5]))-rhs[8]*(params.A[0]*params.J[0]+params.A[1]*params.J[1]+params.A[2]*params.J[2]+params.A[3]*params.J[3]+params.A[4]*params.J[4]+params.A[5]*params.J[5])-rhs[9]*(-1)-rhs[15]*(1)-rhs[21]*(-1)-rhs[27]*(1);
  lhs[3] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[6]+(params.sigma[1]-params.sigma_0[1])*params.J[7]+(params.sigma[2]-params.sigma_0[2])*params.J[8]+(params.sigma[3]-params.sigma_0[3])*params.J[9]+(params.sigma[4]-params.sigma_0[4])*params.J[10]+(params.sigma[5]-params.sigma_0[5])*params.J[11]))-rhs[2]*(-params.Q_lim[1])-rhs[7]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[6]+(params.sigma[1]-params.sigma_obs[1])*params.J[7]+(params.sigma[2]-params.sigma_obs[2])*params.J[8]+(params.sigma[3]-params.sigma_obs[3])*params.J[9]+(params.sigma[4]-params.sigma_obs[4])*params.J[10]+(params.sigma[5]-params.sigma_obs[5])*params.J[11]))-rhs[8]*(params.A[0]*params.J[6]+params.A[1]*params.J[7]+params.A[2]*params.J[8]+params.A[3]*params.J[9]+params.A[4]*params.J[10]+params.A[5]*params.J[11])-rhs[10]*(-1)-rhs[16]*(1)-rhs[22]*(-1)-rhs[28]*(1);
  lhs[4] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[12]+(params.sigma[1]-params.sigma_0[1])*params.J[13]+(params.sigma[2]-params.sigma_0[2])*params.J[14]+(params.sigma[3]-params.sigma_0[3])*params.J[15]+(params.sigma[4]-params.sigma_0[4])*params.J[16]+(params.sigma[5]-params.sigma_0[5])*params.J[17]))-rhs[3]*(-params.Q_lim[2])-rhs[7]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[12]+(params.sigma[1]-params.sigma_obs[1])*params.J[13]+(params.sigma[2]-params.sigma_obs[2])*params.J[14]+(params.sigma[3]-params.sigma_obs[3])*params.J[15]+(params.sigma[4]-params.sigma_obs[4])*params.J[16]+(params.sigma[5]-params.sigma_obs[5])*params.J[17]))-rhs[8]*(params.A[0]*params.J[12]+params.A[1]*params.J[13]+params.A[2]*params.J[14]+params.A[3]*params.J[15]+params.A[4]*params.J[16]+params.A[5]*params.J[17])-rhs[11]*(-1)-rhs[17]*(1)-rhs[23]*(-1)-rhs[29]*(1);
  lhs[5] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[18]+(params.sigma[1]-params.sigma_0[1])*params.J[19]+(params.sigma[2]-params.sigma_0[2])*params.J[20]+(params.sigma[3]-params.sigma_0[3])*params.J[21]+(params.sigma[4]-params.sigma_0[4])*params.J[22]+(params.sigma[5]-params.sigma_0[5])*params.J[23]))-rhs[4]*(-params.Q_lim[3])-rhs[7]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[18]+(params.sigma[1]-params.sigma_obs[1])*params.J[19]+(params.sigma[2]-params.sigma_obs[2])*params.J[20]+(params.sigma[3]-params.sigma_obs[3])*params.J[21]+(params.sigma[4]-params.sigma_obs[4])*params.J[22]+(params.sigma[5]-params.sigma_obs[5])*params.J[23]))-rhs[8]*(params.A[0]*params.J[18]+params.A[1]*params.J[19]+params.A[2]*params.J[20]+params.A[3]*params.J[21]+params.A[4]*params.J[22]+params.A[5]*params.J[23])-rhs[12]*(-1)-rhs[18]*(1)-rhs[24]*(-1)-rhs[30]*(1);
  lhs[6] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[24]+(params.sigma[1]-params.sigma_0[1])*params.J[25]+(params.sigma[2]-params.sigma_0[2])*params.J[26]+(params.sigma[3]-params.sigma_0[3])*params.J[27]+(params.sigma[4]-params.sigma_0[4])*params.J[28]+(params.sigma[5]-params.sigma_0[5])*params.J[29]))-rhs[5]*(-params.Q_lim[4])-rhs[7]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[24]+(params.sigma[1]-params.sigma_obs[1])*params.J[25]+(params.sigma[2]-params.sigma_obs[2])*params.J[26]+(params.sigma[3]-params.sigma_obs[3])*params.J[27]+(params.sigma[4]-params.sigma_obs[4])*params.J[28]+(params.sigma[5]-params.sigma_obs[5])*params.J[29]))-rhs[8]*(params.A[0]*params.J[24]+params.A[1]*params.J[25]+params.A[2]*params.J[26]+params.A[3]*params.J[27]+params.A[4]*params.J[28]+params.A[5]*params.J[29])-rhs[13]*(-1)-rhs[19]*(1)-rhs[25]*(-1)-rhs[31]*(1);
  lhs[7] = -rhs[0]*(((params.sigma[0]-params.sigma_0[0])*params.J[30]+(params.sigma[1]-params.sigma_0[1])*params.J[31]+(params.sigma[2]-params.sigma_0[2])*params.J[32]+(params.sigma[3]-params.sigma_0[3])*params.J[33]+(params.sigma[4]-params.sigma_0[4])*params.J[34]+(params.sigma[5]-params.sigma_0[5])*params.J[35]))-rhs[6]*(-params.Q_lim[5])-rhs[7]*(-2*((params.sigma[0]-params.sigma_obs[0])*params.J[30]+(params.sigma[1]-params.sigma_obs[1])*params.J[31]+(params.sigma[2]-params.sigma_obs[2])*params.J[32]+(params.sigma[3]-params.sigma_obs[3])*params.J[33]+(params.sigma[4]-params.sigma_obs[4])*params.J[34]+(params.sigma[5]-params.sigma_obs[5])*params.J[35]))-rhs[8]*(params.A[0]*params.J[30]+params.A[1]*params.J[31]+params.A[2]*params.J[32]+params.A[3]*params.J[33]+params.A[4]*params.J[34]+params.A[5]*params.J[35])-rhs[14]*(-1)-rhs[20]*(1)-rhs[26]*(-1)-rhs[32]*(1);
}
void multbyP(double *lhs, double *rhs) {
  /* TODO use the fact that P is symmetric? */
  /* TODO check doubling / half factor etc. */
  lhs[0] = rhs[0]*(2*params.l[0]);
  lhs[1] = rhs[1]*(2*params.l[0]);
  lhs[2] = rhs[2]*(2*params.M[0]);
  lhs[3] = rhs[3]*(2*params.M[1]);
  lhs[4] = rhs[4]*(2*params.M[2]);
  lhs[5] = rhs[5]*(2*params.M[3]);
  lhs[6] = rhs[6]*(2*params.M[4]);
  lhs[7] = rhs[7]*(2*params.M[5]);
}
void fillq(void) {
  work.q[0] = 0;
  work.q[1] = 0;
  work.q[2] = -2*params.M[0]*params.dotq_adm[0];
  work.q[3] = -2*params.M[1]*params.dotq_adm[1];
  work.q[4] = -2*params.M[2]*params.dotq_adm[2];
  work.q[5] = -2*params.M[3]*params.dotq_adm[3];
  work.q[6] = -2*params.M[4]*params.dotq_adm[4];
  work.q[7] = -2*params.M[5]*params.dotq_adm[5];
}
void fillh(void) {
  work.h[0] = -(-params.h_goal[0]-((params.sigma[0]-params.sigma_0[0])*params.Sigma[0]+(params.sigma[1]-params.sigma_0[1])*params.Sigma[1]+(params.sigma[2]-params.sigma_0[2])*params.Sigma[2]+(params.sigma[3]-params.sigma_0[3])*params.Sigma[3]+(params.sigma[4]-params.sigma_0[4])*params.Sigma[4]+(params.sigma[5]-params.sigma_0[5])*params.Sigma[5]));
  work.h[1] = -params.h_lim[0];
  work.h[2] = -params.h_lim[1];
  work.h[3] = -params.h_lim[2];
  work.h[4] = -params.h_lim[3];
  work.h[5] = -params.h_lim[4];
  work.h[6] = -params.h_lim[5];
  work.h[7] = -params.h_safe[0];
  work.h[8] = params.B[0];
  work.h[9] = -params.dotq_min[0];
  work.h[10] = -params.dotq_min[0];
  work.h[11] = -params.dotq_min[0];
  work.h[12] = -params.dotq_min[0];
  work.h[13] = -params.dotq_min[0];
  work.h[14] = -params.dotq_min[0];
  work.h[15] = params.dotq_max[0];
  work.h[16] = params.dotq_max[0];
  work.h[17] = params.dotq_max[0];
  work.h[18] = params.dotq_max[0];
  work.h[19] = params.dotq_max[0];
  work.h[20] = params.dotq_max[0];
  work.h[21] = -(-params.a_max[0]+params.dotq_prev[0]);
  work.h[22] = -(-params.a_max[1]+params.dotq_prev[1]);
  work.h[23] = -(-params.a_max[2]+params.dotq_prev[2]);
  work.h[24] = -(-params.a_max[3]+params.dotq_prev[3]);
  work.h[25] = -(-params.a_max[4]+params.dotq_prev[4]);
  work.h[26] = -(-params.a_max[5]+params.dotq_prev[5]);
  work.h[27] = -(-params.dotq_prev[0]-params.a_max[0]);
  work.h[28] = -(-params.dotq_prev[1]-params.a_max[1]);
  work.h[29] = -(-params.dotq_prev[2]-params.a_max[2]);
  work.h[30] = -(-params.dotq_prev[3]-params.a_max[3]);
  work.h[31] = -(-params.dotq_prev[4]-params.a_max[4]);
  work.h[32] = -(-params.dotq_prev[5]-params.a_max[5]);
}
void fillb(void) {
}
void pre_ops(void) {
  work.quad_592664313856[0] = params.dotq_adm[0]*params.M[0]*params.dotq_adm[0]+params.dotq_adm[1]*params.M[1]*params.dotq_adm[1]+params.dotq_adm[2]*params.M[2]*params.dotq_adm[2]+params.dotq_adm[3]*params.M[3]*params.dotq_adm[3]+params.dotq_adm[4]*params.M[4]*params.dotq_adm[4]+params.dotq_adm[5]*params.M[5]*params.dotq_adm[5];
}
