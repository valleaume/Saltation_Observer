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
sys_ball.g     = 9.8;
sys_ball.f_air = 0;

vstar   = sys_ball.vStar();      % = 4
taustar = sys_ball.tauStar();    % = 0.8163
fprintf('asymptotic operating point: v* = %.4f, tau* = %.4f\n', vstar, taustar);

%% Observer
sys_obs = UnknownGroundBallObserver();
sys_obs.lambda = sys_ball.lambda;
sys_obs.mu     = sys_ball.mu;
sys_obs.g      = sys_ball.g;
sys_obs.f_air  = sys_ball.f_air;

%% Select observer gain profile
% Set to 'AfterBeforeContracting' for the contracting gains or
% 'BeforeContracting' for the degenerate jump gain that leaves the
% ground-height error invariant.
if ~exist('gainProfile', 'var') || isempty(gainProfile)
    gainProfile = 'AfterBeforeContracting';
end
x3_true = 0.0;
h0      = vstar^2/(2*sys_ball.g);

switch gainProfile
    case 'AfterBeforeContracting'
        
        sys_obs.L_c   = [2.79; 40.48; 0];
        sys_obs.L_d   = [-1.37; 0.41; 0.28];

        gamma2 = 0.609;
        P = [   19.333   13.331     4.491
       13.331   25.106    88.745
        4.491   88.745   509.083];

        theta0  = [-0.526; 0.848; -0.056];  
        eps0    = 5e-2;

        x0      = [x3_true ; vstar; x3_true];
        tspan   = [0, 14];
    case 'BeforeContracting'
        % L_d(3)=0 gives M_before(3,3)=1, so ground-height error persists.
        sys_obs.L_c   = [11.6; 31.8; 0.0];
        sys_obs.L_d   = [3.7; 1.1; -1.2];
        gamma2 = NaN;
        P = [   2.3   -0.32 -19.0110
           -0.32    1.08    4.72
            -19.0110    4.72   326];

        theta0  = [0.83; -0.5; 0.06];  
        eps0    = 1e-3;

        x0      = [x3_true; vstar; x3_true];
        tspan   = [0, 14];
    otherwise
        error('Unknown gain profile "%s". Choose "AfterBeforeContracting" or "BeforeContracting".', gainProfile);
end
sys_obs.kappa = 0;          % Lemma 2 design (omega_hat independent of y)

% BEWARE: L_d(3) = 0 is a degenerate point -- M_before(3,3) = 1 identically,
% so the x3 error is then strictly invariant and the JSR is exactly 1.
% Any gain search must be kept away from L_d(3) = 0.

%% Certificate P for the Lyapunov plot (from the same LMI search)

fprintf('certificate on P: max eig(P)= %.4f,  min eig(P) = %.4f\n', max(eig(P)), min(eig(P)));
fprintf('gain profile: %s\n', gainProfile);

%% Coupled system
sys = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs);
sys.setInput('Observer', @(y_ball, ~) y_ball);
sys

%% Solver
config = HybridSolverConfig('AbsTol', 1e-8, 'RelTol', 1e-10, 'MaxStep', 1e-3);

% Start ON the period-1 orbit so that (v,tau) sits at (v*,tau*):
% dropping from rest at height v*^2/(2g) above the ground gives impact speed v*.
x3_true = 0.0;
h0      = vstar^2/(2*sys_ball.g);

% Small initial error (Theorem 1 is local).  Note the third component: the
% observer does NOT know the ground height.

theta0 = theta0/norm(theta0);

xhat0   = x0 + eps0*theta0;

x0_cell = {x0; xhat0};

jspan   = [0, 200];

%% Solve
sol = sys.solve(x0_cell, tspan, jspan, config);
sol

%% Who jumps first
ball_sol = sol('Ball');
observer_sol = sol('Observer');
mask_jump_after = (ball_sol.j - observer_sol.j) > 0;
mask_jump_before = (ball_sol.j - observer_sol.j) < 0;
synchronized_mask = (ball_sol.j == observer_sol.j);
sign_jump = zeros(1, length(mask_jump_after));
for i = 2:length(mask_jump_after)
    if mask_jump_before(i),
        sign_jump(i) = -1;
    elseif mask_jump_after(i),
        sign_jump(i) = +1;
    else,                  
        sign_jump(i) = sign_jump(i-1);
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
    .flowColor('#FF8800').jumpColor('m').flowLineStyle('--').jumpLineStyle(':').jumpEndMarker('o') ...
    .legend('$\hat{x}_1$','$\hat{x}_2$','$\hat{x}_3$') ...
    .plotFlows(sol('Observer').select(1:3));

%% Figure 2 : ground-height estimate -- the point of the example
figure(2)
plot(sol('Ball').t, sol('Ball').x(:,3), 'k--', 'LineWidth', 1.2); hold on;
plot(sol('Observer').t, sol('Observer').x(:,3), 'Color', '#FF8800');
grid on;
xlabel('$t$','Interpreter','latex');
ylabel('$x_3$','Interpreter','latex');
legend('true ground height','estimate','Location','best', 'Location', 'northeast', 'Interpreter', 'latex', 'Box', 'off');
title('Ground height is corrected only at jumps');

%% Figure 4 : who jumps first
figure(4)
stairs(sol('Ball').t, sol('Ball').j - sol('Observer').j); grid on;
xlabel('$t$','Interpreter','latex'); ylabel('$j - \hat{j}$','Interpreter','latex');
title('Jump-index mismatch (branch selector)');

%% Figure 5 : Norm of the error
figure(5)
e = sol('Ball').x - sol('Observer').x(:,1:3);

% Trying to get rid of spikes
% Remove time when observer and system are not synchronized
far_jump_mask = synchronized_mask';
e_after = e(sign_jump==1 & far_jump_mask,:);
e_before = e(sign_jump==-1 & far_jump_mask,:); 
e_start = e(sign_jump==0 & far_jump_mask,:); 

semilogy(sol('Ball').t(sign_jump==-1 & far_jump_mask), diag(e_before*P*e_before'), color='green'); % When jumping before
hold on;
semilogy(sol('Ball').t(sign_jump==1 & far_jump_mask), diag(e_after*P*e_after'), color='red', LineStyle='--'); % When jumping after
hold on;
semilogy(sol('Ball').t(sign_jump==0 & far_jump_mask), diag(e_start*P*e_start'), color='black'); % When starting
grid on;
legend('Observer jumps before', 'Observer jumps after', 'Box', 'off', 'Location', 'best');
xlabel('$t$', 'Interpreter', 'Latex')
ylabel('$\theta^\top P\,\theta$','Interpreter','latex');
title("Norm error");

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

%{
%% Figure 3 : Lyapunov function, sampled after paired jumps
% Skip the mismatch window and sample once both subsystems reach the same j.
e_all = ball_sol.x - observer_sol.x(:,1:3);
jump_idx = find(diff(ball_sol.j) > 0);
V_at_jumps = zeros(numel(jump_idx),1);
t_at_jumps = zeros(numel(jump_idx),1);
for k = 1:numel(jump_idx)
    post_ball_idx = jump_idx(k) + 1;
    target_j = ball_sol.j(post_ball_idx);
    paired_offset = find(synchronized_mask(post_ball_idx:end) & ...
                         ball_sol.j(post_ball_idx:end) == target_j, 1, 'first');
    assert(~isempty(paired_offset), ...
        'No synchronized sample found after system jump j = %d.', target_j);
    paired_idx = post_ball_idx + paired_offset - 1;
    ek = e_all(paired_idx,:)';
    V_at_jumps(k) = ek'*P*ek;
    t_at_jumps(k) = ball_sol.t(paired_idx);
end
figure(3)
semilogy(t_at_jumps, V_at_jumps, 'o-'); hold on; grid on;
if ~isnan(gamma2)
    semilogy(t_at_jumps, V_at_jumps(1)*gamma2.^(0:numel(V_at_jumps)-1)', 'r--');
end
xlabel('$t$','Interpreter','latex');
ylabel('$\theta^\top P\,\theta$','Interpreter','latex');
if ~isnan(gamma2)
    legend('measured after paired jumps', ...
           sprintf('certified rate $\\gamma^2 = %.3f$', gamma2), ...
           'Interpreter','latex','Location','best',...
           'Box', 'off');
else
    legend('measured after paired jumps', 'Interpreter','latex','Location','best', 'Box', 'off');
end
title('Per-cycle contraction');

fprintf('\nobserved per-cycle ratios:\n');
disp((V_at_jumps(2:end)./V_at_jumps(1:end-1))');
if ~isnan(gamma2)
    fprintf('certified bound: %.4f\n', gamma2);
end
%}