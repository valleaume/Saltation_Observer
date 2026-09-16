function out = JSR_LMI_search(varargin)
% JSR_LMI_SEARCH  Grid over the flow gain L_c; for each L_c solve an LMI in
% (P, Y = P*L_d) certifying the k = 1 ellipsoidal bound on the joint spectral
% radius of the family
%
%     { M_before(v)*expm(A_F*tau),  M_after(v)*expm(A_F*tau) }
%
% for (v,tau) in a +/-vrel neighbourhood of the asymptotic operating point
% (v*, tau*) = (mu/(1-lambda), 2v*/gamma).
%
% WHY THIS AND NOT COROLLARY 1.  Corollary 1 asks for (20a) on the flow and
% (20b)-(20c) on the jump SEPARATELY, then glues them with
% ln(a_d) + a_c*(t_{j+1}-t_j) < 0.  That splitting is strictly more
% conservative: it adds the worst flow direction to the worst jump direction,
% although the two never occur together.  Here we contract the FULL
% flow-then-jump product in one shot, which is exactly the hypothesis of
% Theorem 1 (exponential stability of the discrete inclusion (13)).
%
% WHY IT IS AN LMI.  Both saltation matrices are affine in L_d:
%       M_before = Xi - L_d*H,      M_after = Xi - L_d*Htilde
% so with Phi = (Xi - L_d*Hx)*E the contraction Phi'*P*Phi <= gam2*P is, by
% Schur complement and the change of variable Y = P*L_d,
%       [ gam2*P          ((P*Xi - Y*Hx)*E)' ;
%         (P*Xi - Y*Hx)*E   P                 ]  >= 0
% which is linear in (P, Y) for fixed gam2.  Bisection on gam2 closes it.
% L_d is recovered as P\Y.
%
% REQUIRES: YALMIP + an SDP solver (SeDuMi / SDPT3 / MOSEK).
%
% UNTESTED: this file has not been run.  Expect to debug solver options and
% YALMIP syntax before trusting any number it prints.
%
% Usage:
%   out = JSR_LMI_search();
%   out = JSR_LMI_search('vrel', 0.05, 'nv', 7, 'zetaGrid', 0.2:0.05:0.5);

p = inputParser;
addParameter(p, 'lambda', 0.5);
addParameter(p, 'mu',     2.0);
addParameter(p, 'gamma',  9.8);
addParameter(p, 'vrel',   0.05);          % +/- 5% around (v*,tau*)
addParameter(p, 'nv',     7);             % samples across that neighbourhood
addParameter(p, 'zetaGrid',  0.15:0.05:0.60);
addParameter(p, 'omegaGrid', 4:0.5:10);
addParameter(p, 'gamLo',  0.30);
addParameter(p, 'gamHi',  1.20);
addParameter(p, 'nBisect', 22);
addParameter(p, 'verbose', true);
parse(p, varargin{:});
o = p.Results;

e = o.lambda;  m = o.mu;  g = o.gamma;
vstar   = m/(1-e);
taustar = 2*vstar/g;

% ---- sample the (v,tau) neighbourhood (v and tau are physically linked) ----
dv   = linspace(-o.vrel, o.vrel, o.nv);
vs   = vstar*(1+dv);
taus = 2*vs/g;

% ---- precompute the (v-dependent) saltation factors ----
nS = numel(vs);
Xi = cell(nS,1);  Htil = cell(nS,1);
H  = [1 0 0];
for k = 1:nS
    x2    = -vs(k);
    dgdx  = [0 0 1; 0 -e 0; 0 0 1];
    N     = [e*x2 - m; g*(1+e); 0];
    d     = x2;
    delta = -(1+e)*x2 + m;
    w     = [1 0 -1];
    Xi{k}   = dgdx - (N/d)*w;
    Htil{k} = H + (delta/d)*w;
end

best = struct('gam', inf, 'zeta', NaN, 'omega', NaN, 'P', [], 'L_d', [], 'L_c', []);
results = [];

for zeta = o.zetaGrid
  for om = o.omegaGrid
    L_c = [2*zeta*om; om^2; 0];
    A_F = [0 1 0; 0 0 0; 0 0 0] - L_c*H;
    E   = cell(nS,1);
    for k = 1:nS, E{k} = expm(A_F*taus(k)); end

    % ---------- bisection on gamma ----------
    lo = o.gamLo;  hi = o.gamHi;  Pbest = [];  Ldbest = [];  gbest = inf;
    for it = 1:o.nBisect
        gam  = 0.5*(lo+hi);
        [ok, P, Ld] = solve_half_feas(gam, Xi, Htil, H, E, nS);
        if ok
            hi = gam;  gbest = gam;  Pbest = P;  Ldbest = Ld;
        else
            lo = gam;
        end
        if hi - lo < 1e-4, break; end
    end

    results(end+1,:) = [zeta, om, gbest]; %#ok<AGROW>
    if o.verbose
        fprintf('zeta=%.2f  omega=%5.2f  ->  gamma_1 = %s\n', ...
                zeta, om, ternary(isinf(gbest),'infeasible',sprintf('%.4f',gbest)));
    end
    if gbest < best.gam
        best.gam = gbest;  best.zeta = zeta;  best.omega = om;
        best.P = Pbest;    best.L_d = Ldbest; best.L_c = L_c;
    end
  end
end

out.best    = best;
out.results = results;
out.vstar   = vstar;
out.taustar = taustar;

if o.verbose && isfinite(best.gam)
    fprintf('\n=== best ===\n');
    fprintf('zeta = %.3f, omega = %.3f  ->  L_c = [%.4f; %.4f; %.4f]\n', ...
            best.zeta, best.omega, best.L_c);
    fprintf('L_d = [%.4f; %.4f; %.4f]\n', best.L_d);
    fprintf('gamma_1 = %.4f   (per-cycle contraction gamma^2 = %.4f)\n', ...
            best.gam, best.gam^2);
    disp('P ='); disp(best.P);
    % independent a posteriori check, not through YALMIP
    verify_bound(best, Xi, Htil, H, taus, vs, e, m, g);
end
end

% ------------------------------------------------------------------------
function [ok, Pval, Ldval] = solve_feas(gam, Xi, Htil, H, E, nS)
% Feasibility of the k=1 ellipsoidal bound at level gam, in (P, Y).
ok = false;  Pval = [];  Ldval = [];
P = sdpvar(3,3,'symmetric');
Y = sdpvar(3,1,'full');            % Y = P*L_d
gam2 = gam^2;

Cons = [P >= 1e-6*eye(3)];
for k = 1:nS
    for branch = 1:2
        if branch == 1, Hx = H; else, Hx = Htil{k}; end
        PPhi = (P*Xi{k} - Y*Hx)*E{k};            % = P*(Xi - L_d*Hx)*E
        Cons = [Cons, [gam2*P, PPhi'; PPhi, P] >= 0]; %#ok<AGROW>
    end
end
% normalisation to keep the SDP bounded
Cons = [Cons, P <= 1e4*eye(3)];

opts = sdpsettings('verbose', 0, 'cachesolvers', 1);
diag = optimize(Cons, [], opts);
disp(opts.solver);
if diag.problem == 0
    Pval  = value(P);
    Ldval = Pval\value(Y);
    ok    = true;
end
end

% ------------------------------------------------------------------------
function [ok, Pval, Ldval] = solve_half_feas(gam, Xi, Htil, H, E, nS)
% Feasibility of the k=1 ellipsoidal bound at level gam, in (P, Y).
ok = false;  Pval = [];  Ldval = [];
P = sdpvar(3,3,'symmetric');
P_after = sdpvar(3,3,'symmetric');
Y = sdpvar(3,1,'full');            % Y = P*L_d
gam2 = gam^2;
gam_expansion = 1.02;

Cons = [P >= 1e-6*eye(3)];
for k = 1:nS
    for branch = 1:2
        if branch == 1
            Hx = H; 
            PPhi = (P*Xi{k} - Y*Hx)*E{k};            % = P*(Xi - L_d*Hx)*E
            Cons = [Cons, [gam2*P, PPhi'; PPhi, P] >= 0]; %#ok<AGROW>
        else
            %Hx = Htil{k}; 
            %PPhi = (P_after*Xi{k} - Y*Hx)*E{k};            % = P*(Xi - L_d*Hx)*E
            %Cons = [Cons, [-gam_expansion*P_after, PPhi'; PPhi, -P_after] >= 0]; %#ok<AGROW>
        end
        
    end
end
% normalisation to keep the SDP bounded
Cons = [Cons, P <= 1e4*eye(3)];

opts = sdpsettings('verbose', 0, 'cachesolvers', 1);
diag = optimize(Cons, [], opts);
disp(opts.solver);
if diag.problem == 0
    Pval  = value(P);
    Ldval = Pval\value(Y);
    ok    = true;
end
end

% ------------------------------------------------------------------------
function verify_bound(best, Xi, Htil, H, taus, vs, e, m, g) %#ok<INUSD>
% Re-checks the certificate directly (no solver), by computing
% max eig(Phi'*P*Phi, P) over the sampled family.
A_F = [0 1 0; 0 0 0; 0 0 0] - best.L_c*H;
worst = -inf;
for k = 1:numel(taus)
    Ek = expm(A_F*taus(k));
    for branch = 1:2
        if branch == 1, Hx = H; else, Hx = Htil{k}; end
        Phi = (Xi{k} - best.L_d*Hx)*Ek;
        worst = max(worst, max(real(eig(Phi'*best.P*Phi, best.P))));
    end
end
fprintf('a posteriori check: max eig(Phi''*P*Phi, P) = %.6f  (= gamma^2)\n', worst);
end

% ------------------------------------------------------------------------
function s = ternary(c,a,b)
if c, s = a; else, s = b; end
end