% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(dotq, eye(6)))
%   subject to
%     -(sigma' - sigma_0')*J*dotq >=  - h_goal - (sigma' - sigma_0')*Sigma
%     Q_lim*dotq >= h_lim
%     2*(sigma' - sigma_obs')*J*dotq >= h_safe
%     dotq_min <= dotq
%     dotq <= dotq_max
%     -a_max <= dotq - dotq_prev
%     dotq - dotq_prev <= a_max
%
% with variables
%     dotq   6 x 1
%
% and parameters
%        J   6 x 6
%    Q_lim   6 x 6    diagonal
%    Sigma   6 x 1
%    a_max   6 x 1
% dotq_max   6 x 1
% dotq_min   6 x 1
% dotq_prev   6 x 1
%   h_goal   1 x 1
%    h_lim   6 x 1
%   h_safe   1 x 1
%    sigma   6 x 1
%  sigma_0   6 x 1
% sigma_obs   6 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.J, ..., params.sigma_obs, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2021-01-26 05:50:03 -0500.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
