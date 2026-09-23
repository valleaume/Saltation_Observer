% UNKNOWN_GROUND_OBSERVER
% Bouncing ball above an UNKNOWN ground height, observed from the absolute
% position y = x1 only.  Companion to observers.m of the CDC repo, for the
% journal-version numerical example.
%
%   x = [x1; x2; x3] = [height; velocity; ground height]
%
% Why this example is interesting: (x1,x2) is observable during flow, but x3
% is NOT -- the flow Jacobian A_F has an identically zero row AND column for
% x3, so no flow gain whatsoever corrects x3.  All information on x3 arrives
% through the jumps.  This makes the flow/jump splitting of Corollary 1 bite
% hard, and (see README) Corollary 1 appears infeasible here while Theorem 1
% still applies via the product condition.
%
% UNTESTED: this script has not been run.  Debug before trusting it.

addpath('utils');
close all;

%% Plant
%sys_ball = BouncingBallSubSystemClass();
sys_ball = UnknownGroundBallSubSystemClass();
sys_ball.lambda = 0.5;     % restitution
sys_ball.mu     = 2.0;     % added velocity at impact (mu > 0 => no Zeno)
sys_ball.gamma  = 9.8;
sys_ball.f_air  = 0;

vstar   = sys_ball.vStar();      % = 4
taustar = sys_ball.tauStar();    % = 0.8163
fprintf('asymptotic operating point: v* = %.4f, tau* = %.4f\n', vstar, taustar);

%% Observer
sys_obs = UnknownGroundBallObserver();
sys_obs.lambda = sys_ball.lambda;
sys_obs.mu     = sys_ball.mu;
sys_obs.gamma  = sys_ball.gamma;
sys_obs.f_air  = sys_ball.f_air;

% Gains from the k=1 JSR LMI search (see JSR_LMI_search.m):
%   omega = 7 rad/s, zeta = 0.25  ->  flow poles -1.75 +/- 6.78i
sys_obs.L_c   = [3.5; 49.0; 0.0];
sys_obs.L_d   = [1.5; -3.2; 1.0];
sys_obs.kappa = 0;          % Lemma 2 design (omega_hat independent of y)

% BEWARE: L_d(3) = 0 is a degenerate point -- M_before(3,3) = 1 identically,
% so the x3 error is then strictly invariant and the JSR is exactly 1.
% Any gain search must be kept away from L_d(3) = 0.

%% Certificate P for the Lyapunov plot (from the same LMI search)
P = [  5.35797  -1.79583 -14.14947;
      -1.79583   1.05090   5.71556;
     -14.14947   5.71556  45.89246 ];
gamma2 = 0.4838;            % per-cycle contraction certified on +/-5%
fprintf('certificate on P: max eig(P)= %.4f,  min eig(P) = %.4f\n', max(eig(P)), min(eig(P)));

%% Coupled system
sys = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs);
sys.setInput('Observer', @(y_ball, ~) y_ball);
sys

%% Solver
config = HybridSolverConfig('AbsTol', 1e-8, 'RelTol', 1e-10, 'MaxStep', 1e-3);

% Start ON the period-1 orbit so that (v,tau) sits at (v*,tau*):
% dropping from rest at height v*^2/(2g) above the ground gives impact speed v*.
x3_true = 0.0;
h0      = vstar^2/(2*sys_ball.gamma);
x0      = [x3_true + h0; 0; x3_true];

% Small initial error (Theorem 1 is local).  Note the third component: the
% observer does NOT know the ground height.
theta0  = [0.5; -0.6; -0.62];  theta0 = theta0/norm(theta0);
eps0    = 1e-1;
xhat0   = x0 + eps0*theta0;

x0_cell = {x0; xhat0};
tspan   = [0, 8];
jspan   = [0, 200];

%% Solve
sol = sys.solve(x0_cell, tspan, jspan, config);
sol

%% Who jumps first
mask_after  = (sol('Ball').j - sol('Observer').j) > 0;
mask_before = (sol('Ball').j - sol('Observer').j) < 0;
sign_jump = zeros(1, length(mask_after));
for i = 2:length(mask_after)
    if mask_before(i),     sign_jump(i) = -1;
    elseif mask_after(i),  sign_jump(i) = +1;
    else,                  sign_jump(i) = sign_jump(i-1);
    end
end

%% Figure 1 : states
figure(1)
hpb = HybridPlotBuilder().subplots('on') ...
    .labels('$x_1$','$x_2$','$x_3$') ...
    .legend('$x_1$','$x_2$','$x_3$') ...
    .plotFlows(sol('Ball'));
grid on; hold on
hpb.subplots('on') ...
    .flowColor('#FF8800').jumpColor('m').jumpEndMarker('o') ...
    .legend('$\hat{x}_1$','$\hat{x}_2$','$\hat{x}_3$') ...
    .plotFlows(sol('Observer').select(1:3));

%% Figure 2 : ground-height estimate -- the point of the example
figure(2)
plot(sol('Ball').t, sol('Ball').x(:,3), 'k--', 'LineWidth', 1.2); hold on;
plot(sol('Observer').t, sol('Observer').x(:,3), 'Color', '#FF8800');
grid on;
xlabel('$t$','Interpreter','latex');
ylabel('$x_3,\ \hat{x}_3$','Interpreter','latex');
legend('true ground height','estimate','Location','best');
title('Ground height is corrected only at jumps');

%% Figure 3 : Lyapunov function, sampled at the SYSTEM jump times
% The certificate bounds V from one system jump to the next; V is NOT
% controlled inside the mismatch window, hence the sampling.
e_all = sol('Ball').x - sol('Observer').x(:,1:3);
jump_idx = find(diff(sol('Ball').j) > 0);
V_at_jumps = zeros(numel(jump_idx),1);
t_at_jumps = zeros(numel(jump_idx),1);
for k = 1:numel(jump_idx)
    ek = e_all(jump_idx(k)+1,:)';
    V_at_jumps(k) = ek'*P*ek;
    t_at_jumps(k) = sol('Ball').t(jump_idx(k)+1);
end
figure(3)
semilogy(t_at_jumps, V_at_jumps, 'o-'); hold on; grid on;
semilogy(t_at_jumps, V_at_jumps(1)*gamma2.^(0:numel(V_at_jumps)-1)', 'r--');
xlabel('$t$','Interpreter','latex');
ylabel('$\theta^\top P\,\theta$','Interpreter','latex');
legend('measured at system jumps', ...
       sprintf('certified rate $\\gamma^2 = %.3f$', gamma2), ...
       'Interpreter','latex','Location','best');
title('Per-cycle contraction');

fprintf('\nobserved per-cycle ratios:\n');
disp((V_at_jumps(2:end)./V_at_jumps(1:end-1))');
fprintf('certified bound: %.4f\n', gamma2);

%% Figure 4 : who jumps first
figure(4)
stairs(sol('Ball').t, sol('Ball').j - sol('Observer').j); grid on;
xlabel('$t$','Interpreter','latex'); ylabel('$j - \hat{j}$','Interpreter','latex');
title('Jump-index mismatch (branch selector)');

%% Verification of the certificate at (v*,tau*)
[Mb, Ma] = sys_obs.saltationMatrices(vstar);
E = expm(sys_obs.flowJacobian()*taustar);
fprintf('\n--- certificate check at (v*,tau*) ---\n');
fprintf('max eig((Mb*E)''P(Mb*E), P) = %.6f\n', max(real(eig((Mb*E)'*P*(Mb*E), P))));
fprintf('max eig((Ma*E)''P(Ma*E), P) = %.6f\n', max(real(eig((Ma*E)'*P*(Ma*E), P))));

% For contrast: Corollary 1 evaluated separately with the SAME P.
A_F = sys_obs.flowJacobian();
a_c = max(real(eig(A_F'*P + P*A_F, P)));
a_d = max([max(real(eig(Mb'*P*Mb, P))), max(real(eig(Ma'*P*Ma, P)))]);
fprintf('\n--- Corollary 1 with the same P (expected to FAIL) ---\n');
fprintf('a_c = %.4f,  a_d = %.4f,  ln(a_d) + a_c*tau* = %.4f\n', ...
        a_c, a_d, log(a_d) + a_c*taustar);