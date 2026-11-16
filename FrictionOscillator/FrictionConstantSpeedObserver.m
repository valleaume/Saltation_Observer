% FRICTION_CONSTANT_SPEED_OBSERVER
% Mass on a spring attached to a fixed wall, resting on a table that moves
% at the constant speed vb, Coulomb friction with UNKNOWN static and dynamic
% levels mu_s > mu_d. The table speed is UNKNOWN too. Observed from the
% mass position y = x.
%
%   x = [x; v; vb; mu_s; mu_d; q],  q = 1, 2 slip, 3 stick
%
% Variant of FrictionObserver.m (vibrating table). The slip always goes in
% the same direction, along the orbit stick -> slip 2 -> stick, and three
% parameters are estimated:
%   - mu_s only enters the breakaway guard |z| = mu_s: like the ground height
%     of UnknownGroundObserver.m, it is invisible during flow and corrected
%     at the jumps only (with L_d = 0 its error never changes);
%   - vb only enters the stick-entry guard s = v - vb = 0: the stick entry
%     copies its error onto the velocity error, and the stick flow corrects
%     both from the slope of x;
%   - mu_d is seen during the slip (center of the oscillation).
% For y = v the shift (x, mu_s, mu_d) + d*(1, k, k) is unobservable, since
% the slip direction never changes: only y = x is used. ydot = v is
% continuous at the switches, so M_before = M_after: a single saltation
% matrix per jump.
%
% Gain profiles (gains and P from FrictionConstantSpeedLMI_search.m, rounded):
%   'Both'        L_d certified on M_before = M_after
%   'NoJumpGain'  L_d = 0: mu_s is not recovered

setupPaths();
close all;

%% Plant
sys_plant = FrictionConstantSpeedTableSubSystemClass('position');
% The gains below are designed for m = k = 1; after changing them, rerun
% FrictionConstantSpeedLMI_search('m', sys_plant.m, 'k', sys_plant.k).
sys_plant.m = 1;
sys_plant.k = 1;
mu_s = 0.6;  mu_d = 0.4;  vb = 0.2;     % true values, unknown to the observer
orb = sys_plant.periodicOrbit(mu_s, mu_d, vb);
fprintf('stick-slip orbit: tau_slip = %.4f, tau_stick = %.4f\n', orb.tau_sl, orb.tau_st);

%% Observer
sys_obs = FrictionConstantSpeedTableObserver('position');
sys_obs.m = sys_plant.m;
sys_obs.k = sys_plant.k;

%% Select observer gain profile
if ~exist('gainProfile', 'var') || isempty(gainProfile)
    gainProfile = 'Both';
end
% gam: certified contraction of e'*P*e per jump. P_slip is used in the slip,
% P_stick in the stick phase.
switch gainProfile
    case {'Both', 'NoJumpGain'}
        sys_obs.L_c2  = [0.9; 0.27; 0; 0; 0.327];
        sys_obs.L_c3  = [0.9; 0.18; 0.18; 0; 0];
        sys_obs.L_d23 = [1; -0.03; -0.03; -2.78; 0.26];
        sys_obs.L_d32 = [1; -1.26; 0.85; -1.22; -0.03];
        gam = 0.520;
        P_slip = [62.7 -18.67 1.23 20.69 -35; -18.67 17.9 -0.98 -18.2 -0.44; 1.23 -0.98 21.51 0.89 0.31;
                  20.69 -18.2 0.89 20.61 -0.2; -35 -0.44 0.31 -0.2 50.64];
        P_stick = [94.88 -1.71 -2.43 2.05 1.33; -1.71 53.25 -47.18 0.48 0.47; -2.43 -47.18 52.28 -0.22 0.65;
                   2.05 0.48 -0.22 7.27 -1.25; 1.33 0.47 0.65 -1.25 99.65];
        eps0 = 5e-2;
        default_tspan = [0, 60];
    otherwise
        error('Unknown gain profile "%s".', gainProfile);
end
if strcmp(gainProfile, 'NoJumpGain')
    sys_obs.L_d23 = zeros(5, 1);
    sys_obs.L_d32 = zeros(5, 1);
    gam = NaN;   % mu_s error invariant
end
if ~exist('tspan', 'var') || isempty(tspan)
    tspan = default_tspan;
end
fprintf('gain profile: %s\n', gainProfile);

%% Coupled system
sys = CompositeHybridSystem('Plant', sys_plant, 'Observer', sys_obs);
sys.setInput('Observer', @(y_plant, ~) y_plant);

%% Solver
config = HybridSolverConfig('AbsTol', 1e-8, 'RelTol', 1e-10, 'MaxStep', 1e-2);

% Start ON the orbit, in the middle of a slip phase. The initial error is
% on every state, the three unknown parameters included.
x0 = [expm(sys_plant.flowMatrix(2)*orb.tau_sl/2)*orb.xi_b; 2];

theta0 = [-5; -6; 0.48; 0.5; -0.5];
%theta0 = theta0/norm(theta0);
xhat0 = x0 + [eps0*theta0; 0];
jspan = [0, 1000];

%% Solve
sol = sys.solve({x0; xhat0}, tspan, jspan, config);
plant_sol = sol('Plant');
observer_sol = sol('Observer');

%% Who jumps first
[sign_jump, synchronized_mask] = whoJumpsFirst(plant_sol.j, observer_sol.j);

%% Figure 1 : mass states (first periods)
figure(1)
hpb = HybridPlotBuilder().subplots('on') ...
    .labels('$x$', '$v$') ...
    .legend('$x$', '$v$') ...
    .plotFlows(plant_sol.select(1:2));
grid on; hold on
hpb.subplots('on') ...
    .flowColor('#FF8800').jumpColor('m').flowLineStyle('--').jumpLineStyle(':').jumpEndMarker('o') ...
    .legend('$\hat{x}$', '$\hat{v}$') ...
    .plotFlows(observer_sol.select(1:2));
set(findobj(gcf, 'Type', 'axes'), 'XLim', [0, min(tspan(2), 25)]);

%% Figure 2 : table speed and friction estimates -- the point of the example
figure(2)
names = {'$v_b$', '$\mu_s$', '$\mu_d$'};
for i = 1:3
    subplot(3, 1, i)
    plot(plant_sol.t, plant_sol.x(:, 2 + i), 'k--', 'LineWidth', 1.2); hold on;
    plot(observer_sol.t, observer_sol.x(:, 2 + i), 'Color', '#FF8800');
    grid on;
    ylabel(names{i}, 'Interpreter', 'latex');
    legend('true', 'estimate', 'Location', 'northeast', 'Interpreter', 'latex', 'Box', 'off');
end
xlabel('$t$', 'Interpreter', 'latex');
subplot(3, 1, 1); title('Unknown parameters ($\hat\mu_s$ changes only at jumps)', 'Interpreter', 'latex');

%% Figure 4 : who jumps first
figure(4)
stairs(plant_sol.t, plant_sol.j - observer_sol.j); grid on;
xlabel('$t$', 'Interpreter', 'latex'); ylabel('$j - \hat{j}$', 'Interpreter', 'latex');
title('Jump-index mismatch (branch selector)');

%% Figure 5 : Norm of the error
figure(5)
% e'*P*e with P = P_slip in slip and P_stick in stick.
% Once |e| reaches round-off (~1e-14), the plant and the observer meet their
% guards closer than the solver resolves. HyEQsolver then separates the two
% jumps by an Euler step of 1e-9, which kicks e by (f+ - f-)*1e-9 ~ 2e-10 at a
% breakaway: V goes back up to ~1e-18 and decreases again. It is a floor of
% the solver, not of the observer (docs/hyeq_solver_jump_timing.md).
e = plant_sol.x(:, 1:5) - observer_sol.x(:, 1:5);
q = plant_sol.x(:, 6);
V = sum((e*P_slip).*e, 2);
V(q == 3) = sum((e(q == 3, :)*P_stick).*e(q == 3, :), 2);

far_jump_mask = synchronized_mask;
semilogy(plant_sol.t(sign_jump == -1 & far_jump_mask), V(sign_jump == -1 & far_jump_mask), 'g.', 'MarkerSize', 4); % When jumping before
hold on;
semilogy(plant_sol.t(sign_jump == 1 & far_jump_mask), V(sign_jump == 1 & far_jump_mask), 'r.', 'MarkerSize', 4);   % When jumping after
semilogy(plant_sol.t(sign_jump == 0 & far_jump_mask), V(sign_jump == 0 & far_jump_mask), 'k.', 'MarkerSize', 4);   % When starting
grid on;
legend('Observer jumps before', 'Observer jumps after', 'Box', 'off', 'Location', 'best');
xlabel('$t$', 'Interpreter', 'Latex')
ylabel('$\theta^\top P\,\theta$', 'Interpreter', 'latex');
title("Norm error");

%% Figure 6 : error in each coordinate, on the samples of figure 5
figure(6)
error_labels = {'$|x - \hat{x}|$', '$|v - \hat{v}|$', '$|v_b - \hat{v}_b|$', ...
                '$|\mu_s - \hat\mu_s|$', '$|\mu_d - \hat\mu_d|$'};
for i = 1:5
    subplot(5, 1, i)
    semilogy(plant_sol.t(far_jump_mask), abs(e(far_jump_mask, i)), '.', 'MarkerSize', 4);
    grid on;
    ylabel(error_labels{i}, 'Interpreter', 'latex');
end
xlabel('$t$', 'Interpreter', 'latex');
subplot(5, 1, 1); title('Error in each coordinate (plant and observer in the same mode)');

%% Verification of the certificate on the orbit
[Mb23, Ma23] = sys_obs.saltationMatrices([orb.xi_e; 2]);   % stick entry 2 -> 3
[Mb32, Ma32] = sys_obs.saltationMatrices([orb.xi_b; 3]);   % breakaway 3 -> 2
E2 = expm(sys_obs.errorFlowMatrix(2)*orb.tau_sl);
E3 = expm(sys_obs.errorFlowMatrix(3)*orb.tau_st);
cert = [max(real(eig((Mb23*E2)'*P_stick*(Mb23*E2), P_slip))), ...
        max(real(eig((Ma23*E2)'*P_stick*(Ma23*E2), P_slip))), ...
        max(real(eig((Mb32*E3)'*P_slip*(Mb32*E3), P_stick))), ...
        max(real(eig((Ma32*E3)'*P_slip*(Ma32*E3), P_stick)))];
fprintf('\n--- certificate on the orbit (gam = %.3f) ---\n', gam);
fprintf('slip then stick entry : before %.4f, after %.4f\n', cert(1), cert(2));
fprintf('stick then breakaway  : before %.4f, after %.4f\n', cert(3), cert(4));
