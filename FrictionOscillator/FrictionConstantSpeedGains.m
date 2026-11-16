% FRICTION_CONSTANT_SPEED_GAINS
% The gains of FrictionConstantSpeedObserver.m (profile 'Both') from the
% LMIs of FrictionConstantSpeedLMI_search.m, written directly with YALMIP:
%   1. the search: flow gains placed at sigma = 0.3, jump gains and P from
%      the contraction LMIs on the orbits of mu_s, mu_d, vb within +/- 5%,
%      bisection on the rate gam;
%   2. the gains used by the observer (rounded) certified by the same LMIs,
%      with P only as unknown.
% The jump gains of step 1 are one point of a feasible set, so the solver
% may return slightly different values: here L_d32(2) = -1.2549, which the
% observer rounds to -1.26. Step 2 is what certifies the gains it uses.
% Requires YALMIP, an SDP solver and the Control System Toolbox.

setupPaths();
addpath(fileparts(mfilename('fullpath')));      % FrictionConstantSpeedLMI_search

%% 1. Search
search = FrictionConstantSpeedLMI_search();
fprintf('\nrounded as in FrictionConstantSpeedObserver.m:\n');
fprintf('L_c2  = %s\nL_c3  = %s\n', mat2str(round(search.best.L_c2', 3)), mat2str(round(search.best.L_c3', 2)));
fprintf('L_d23 = %s\nL_d32 = %s\n', mat2str(round(search.best.L_d23', 2)), mat2str(round(search.best.L_d32', 2)));

%% 2. The gains used by the observer, certified
used.L_c2  = [0.9; 0.27; 0; 0; 0.327];
used.L_c3  = [0.9; 0.18; 0.18; 0; 0];
used.L_d23 = [1; -0.03; -0.03; -2.78; 0.26];
used.L_d32 = [1; -1.26; 0.85; -1.22; -0.03];
fprintf('\n--- gains of FrictionConstantSpeedObserver.m, P only ---\n');
certified = FrictionConstantSpeedLMI_search('L_c2', used.L_c2, 'L_c3', used.L_c3, ...
    'L_d23', used.L_d23, 'L_d32', used.L_d32);
fprintf('\nrate: searched gains %.4f, gains used %.4f (the observer quotes gam = 0.520)\n', ...
    search.best.gam, certified.best.gam);
