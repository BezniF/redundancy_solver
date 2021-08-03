% Produced by CVXGEN, 2021-02-25 05:21:53 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: cvxsolve.m.
% Description: Solution file, via cvx, for use with sample.m.
function [vars, status] = cvxsolve(params, settings)
J = params.J;
M = params.M;
Q_lim = params.Q_lim;
Sigma = params.Sigma;
a_max = params.a_max;
dotq_adm = params.dotq_adm;
dotq_max = params.dotq_max;
dotq_min = params.dotq_min;
dotq_prev = params.dotq_prev;
h_goal = params.h_goal;
h_lim = params.h_lim;
h_safe = params.h_safe;
h_safe_2 = params.h_safe_2;
l = params.l;
sigma = params.sigma;
sigma_0 = params.sigma_0;
sigma_obs = params.sigma_obs;
sigma_obs2 = params.sigma_obs2;
theta = params.theta;
x = params.x;
cvx_begin
  % Caution: automatically generated by cvxgen. May be incorrect.
  variable dotq(6, 1);
  variable delta(3, 1);

  minimize(quad_form(dotq - dotq_adm, M) + l*quad_form(delta, eye(3)));
  subject to
    -(sigma' - sigma_0')*dotq >=  - h_goal - (sigma' - sigma_0')*Sigma - theta(1)*delta(1);
    Q_lim*dotq >= h_lim;
    2*(x' - sigma_obs')*J*dotq >= h_safe - theta(2)*delta(2);
    2*(x' - sigma_obs2')*J*dotq >= h_safe_2 - theta(3)*delta(3);
    dotq_min <= dotq;
    dotq <= dotq_max;
    -a_max <= dotq - dotq_prev;
    dotq - dotq_prev <= a_max;
cvx_end
vars.delta = delta;
vars.dotq = dotq;
status.cvx_status = cvx_status;
% Provide a drop-in replacement for csolve.
status.optval = cvx_optval;
status.converged = strcmp(cvx_status, 'Solved');
