function out = FrictionConstantSpeedLMI_search(varargin)
% FRICTIONCONSTANTSPEEDLMI_SEARCH  Jump gains of FrictionConstantSpeedTableObserver
% and a quadratic certificate for the error saltation dynamics along the
% stick-slip orbit, by LMI (same structure as FrictionLMI_search.m).
%
% Along the orbit 3 -> 2 -> 3 the error sampled at the jumps obeys
%
%     e_stick = M23*Phi2*e_slip          (slip 2, then stick entry)
%     e_slip  = M32*Phi3*e_stick         (stick, then breakaway)
%
% The output is y = x: ydot = v is continuous at the switches, so
% M_before = M_after at both jumps (for y = v the state is not observable,
% see FrictionConstantSpeedTableSubSystemClass). As in FrictionLMI_search.m,
% no single P contracts both steps; we take one matrix per phase:
%
%     (M23*Phi2)'*P_stick*(M23*Phi2) <= gam*P_slip
%     (M32*Phi3)'*P_slip*(M32*Phi3)  <= gam*P_stick
%
% WHY IT IS AN LMI. M = Xi - L_d*H is affine in L_d. With
% Y_stick = P_stick*L_d23 and Y_slip = P_slip*L_d32, Schur complements give
% LMIs in (P_slip, P_stick, Y_slip, Y_stick) for fixed gam: bisection.
% I <= P <= kappaP*I keeps the certificate well conditioned.
%
% WHICH GAINS ARE SEARCHED. Only those used along the orbit: L_d23 (stick
% entry) and L_d32 (breakaway). The direct reversal 2 -> 1 never occurs
% near the orbit (the mass always sticks, |z| = 0.2 < mu_s at s = 0): it has
% no saltation matrix in the LMIs, so its gain L_d21 is not searched and
% stays 0.
%
% The flow gains place the observable poles of each mode: slip 2 sees
% (x, v, mu_d), poles -sigma +/- i*w (w = sqrt(k/m)) and -sigma; stick sees
% (x, v), poles -sigma and -2*sigma. In stick v = vb, so the velocity
% correction is applied to vb as well: the stick flow then keeps the
% relative-velocity error s = v - vb, which the stick entry sets to zero,
% and the slope of x recovers vb. mu_s stays at 0 in both modes and is
% corrected by the jumps only. Since mu_s, mu_d and vb are unknown, the LMIs
% are imposed on the orbits of a grid of true values within +/- rel of the
% nominal ones.
%
% REQUIRES: YALMIP + an SDP solver, Control System Toolbox (obsvf, place).
%
% Usage:
%   out = FrictionConstantSpeedLMI_search();                   % nominal mu_s, mu_d, vb +/- 5%
%   out = FrictionConstantSpeedLMI_search('rel', 0);           % nominal orbit only
%   out = FrictionConstantSpeedLMI_search('m', 2, 'k', 0.5);   % other mass and stiffness
%   out = FrictionConstantSpeedLMI_search('L_c2', Lc2, 'L_c3', Lc3, 'L_d23', Ld23, 'L_d32', Ld32);  % P only

parser = inputParser;
addParameter(parser, 'mu_s', 0.6);                % nominal static friction
addParameter(parser, 'mu_d', 0.4);                % nominal dynamic friction
addParameter(parser, 'vb',   0.2);                % nominal table speed
addParameter(parser, 'm',    1);                  % mass (known)
addParameter(parser, 'k',    1);                  % spring stiffness (known)
addParameter(parser, 'rel',  0.05);               % +/- 5% on mu_s, mu_d, vb
addParameter(parser, 'n',    3);                  % samples per parameter
addParameter(parser, 'sigmaGrid', 0.3);           % flow damping
addParameter(parser, 'kappaP', 1e2);
addParameter(parser, 'L_c2', []);                 % given gains (L_d given: search P only)
addParameter(parser, 'L_c3', []);
addParameter(parser, 'L_d23', []);
addParameter(parser, 'L_d32', []);
addParameter(parser, 'gamLo', 0);
addParameter(parser, 'gamHi', 1.5);
addParameter(parser, 'nBisect', 22);
addParameter(parser, 'verbose', true);
parse(parser, varargin{:});
opts = parser.Results;

plant = FrictionConstantSpeedTableSubSystemClass('position');
plant.m = opts.m;  plant.k = opts.k;
obs = FrictionConstantSpeedTableObserver('position');
obs.m = opts.m;  obs.k = opts.k;
H = obs.H();

% ---- orbits for true parameters in the +/- rel box (nominal one first) ----
factors = linspace(1 - opts.rel, 1 + opts.rel, opts.n);     % e.g. [0.95 1 1.05]
[mu_s_factor, mu_d_factor, vb_factor] = ndgrid(factors, factors, factors);
grid_factors = unique([1 1 1; mu_s_factor(:), mu_d_factor(:), vb_factor(:)], 'rows', 'stable');
for k = size(grid_factors, 1):-1:1
    orbits(k) = plant.periodicOrbit(opts.mu_s*grid_factors(k, 1), ...
        opts.mu_d*grid_factors(k, 2), opts.vb*grid_factors(k, 3));
end

% ---- saltation factors along each orbit of the family (Htil = H for y = x) ----
nOrbits = numel(orbits);
family = struct('Xi23', cell(nOrbits, 1), 'Xi32', []);
for k = 1:nOrbits
    family(k).Xi23 = obs.saltationFactors([orbits(k).xi_e; 2]);   % stick entry 2 -> 3
    family(k).Xi32 = obs.saltationFactors([orbits(k).xi_b; 3]);   % breakaway 3 -> 2
end

best = struct('gam', inf);
results = [];
for sigma = opts.sigmaGrid
    [obs.L_c2, obs.L_c3] = place_flow_gains(obs, sigma);
    if ~isempty(opts.L_c2), obs.L_c2 = opts.L_c2(:); obs.L_c3 = opts.L_c3(:); end
    for k = 1:nOrbits
        family(k).Phi2 = expm(obs.errorFlowMatrix(2)*orbits(k).tau_sl);
        family(k).Phi3 = expm(obs.errorFlowMatrix(3)*orbits(k).tau_st);
    end

    % ---------- bisection on the contraction factor gam (per jump) ----------
    lo = opts.gamLo; hi = opts.gamHi; feasible = [];
    for it = 1:opts.nBisect
        gam = 0.5*(lo + hi);
        sol = solve_feas(gam, family, H, opts);
        if sol.ok
            hi = gam; feasible = sol; feasible.gam = gam;
        else
            lo = gam;
        end
        if hi - lo < 1e-4, break; end
    end

    results(end+1, :) = [sigma, hi]; %#ok<AGROW>
    if opts.verbose
        fprintf('sigma = %.2f  ->  gam = %s\n', sigma, ternary(isempty(feasible), 'infeasible', sprintf('%.4f', hi)));
    end
    if ~isempty(feasible) && feasible.gam < best.gam
        best = feasible; best.sigma = sigma;
        best.L_c2 = obs.L_c2; best.L_c3 = obs.L_c3;
        best_family = family;       % Phi of this sigma, for the check below
    end
end

out.best = best;
out.results = results;
out.orbits = orbits;

if opts.verbose && isfinite(best.gam)
    fprintf('\n=== best (%d orbits) ===\n', nOrbits);
    fprintf('L_c2  = %s\nL_c3  = %s\n', mat2str(best.L_c2', 4), mat2str(best.L_c3', 4));
    fprintf('L_d23 = %s\nL_d32 = %s\n', mat2str(best.L_d23', 4), mat2str(best.L_d32', 4));
    fprintf('gam = %.4f per jump (%.4f per period)\n', best.gam, best.gam^2);
    disp('P_slip ='); disp(best.P_slip);
    disp('P_stick ='); disp(best.P_stick);
    verify_bound(best, best_family, H);
end
end

% ------------------------------------------------------------------------
function sol = solve_feas(gam, family, H, opts)
% Feasibility of the two contraction LMIs at level gam.
n = 5;
P_slip = sdpvar(n, n, 'symmetric');
P_stick = sdpvar(n, n, 'symmetric');
if isempty(opts.L_d23)
    Y_stick = sdpvar(n, 1, 'full');           % = P_stick*L_d23
    Y_slip = sdpvar(n, 1, 'full');            % = P_slip*L_d32
else
    Y_stick = P_stick*opts.L_d23(:);
    Y_slip = P_slip*opts.L_d32(:);
end
constraints = [P_slip >= eye(n), P_stick >= eye(n), ...
               P_slip <= opts.kappaP*eye(n), P_stick <= opts.kappaP*eye(n)];
for k = 1:numel(family)
    F = family(k);
    PMPhi = (P_stick*F.Xi23 - Y_stick*H)*F.Phi2;     % = P_stick*M23*Phi2
    constraints = [constraints, [gam*P_slip, PMPhi'; PMPhi, P_stick] >= 0]; %#ok<AGROW>
    PMPhi = (P_slip*F.Xi32 - Y_slip*H)*F.Phi3;       % = P_slip*M32*Phi3
    constraints = [constraints, [gam*P_stick, PMPhi'; PMPhi, P_slip] >= 0]; %#ok<AGROW>
end
diagnostics = optimize(constraints, [], sdpsettings('verbose', 0, 'cachesolvers', 1));
sol.ok = (diagnostics.problem == 0);
if sol.ok
    sol.P_slip = value(P_slip);
    sol.P_stick = value(P_stick);
    sol.L_d23 = sol.P_stick\value(Y_stick);
    sol.L_d32 = sol.P_slip\value(Y_slip);
    sol.ok = worst_bound(sol, family, H) <= gam*(1 + 1e-6);   % do not trust the solver status alone
end
end

% ------------------------------------------------------------------------
function [L_c2, L_c3] = place_flow_gains(obs, sigma)
% Poles of the observable part of A_q - L*H; unobservable modes kept.
w = sqrt(obs.k/obs.m);
L_c2 = observable_placement(obs.flowMatrix(2), obs.H(), [-sigma + w*1i, -sigma - w*1i, -sigma]);
L_c3 = observable_placement(obs.flowMatrix(3), obs.H(), [-sigma, -2*sigma]);
L_c3(3) = L_c3(2);      % in stick v = vb: correct both
end

function L = observable_placement(A, H, poles)
% obsvf puts the observable part last: L = Tobs'*[0; L_observable].
n = size(A, 1);
[Abar, ~, Cbar, Tobs, kObs] = obsvf(A, zeros(n, 1), H);
nObs = sum(kObs);
observable = n-nObs+1:n;
L_observable = place(Abar(observable, observable)', Cbar(:, observable)', poles)';
L = real(Tobs'*[zeros(n - nObs, 1); L_observable]);
end

% ------------------------------------------------------------------------
function worst = worst_bound(sol, family, H)
% max over the family of the generalized eigenvalues of the two LMIs.
worst = -inf;
for k = 1:numel(family)
    F = family(k);
    M = (F.Xi23 - sol.L_d23*H)*F.Phi2;
    worst = max(worst, max(real(eig(M'*sol.P_stick*M, sol.P_slip))));
    M = (F.Xi32 - sol.L_d32*H)*F.Phi3;
    worst = max(worst, max(real(eig(M'*sol.P_slip*M, sol.P_stick))));
end
end

function verify_bound(best, family, H)
% Independent a posteriori check, not through YALMIP.
fprintf('a posteriori check: max eig = %.6f  (= gam)\n', worst_bound(best, family, H));
end

% ------------------------------------------------------------------------
function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
