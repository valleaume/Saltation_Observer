addpath('./utils');
close all;

sys_billard = billard_sys_2d();
sys_copy = billard_obs_2d();
sys_billard.restitution = 1;
sys_copy.restitution = sys_billard.restitution;

Lc = [63, 273, 0.];

sys_copy.L_c = [0, 0;
                0, Lc(1);
                0, 0;
                0, Lc(2);
                0, 0];

Ld = [ 0.99999578; -2.20083779; 0.49959006];

sys_copy.L_d = [0, 0;
               0, Ld(1);
               0, 0;
               0, Ld(2);
               0, Ld(3)];

sys_copy.l = 1;

% Define solver parameters.
max_dt_step = 0.1;
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', max_dt_step);

sys = CompositeHybridSystem('Billard', sys_billard, 'Observer', sys_copy);
obs_input = @(y_ball, ~) y_ball(:);
sys.setInput('Observer', obs_input);

% Initial conditions for the 5-state plant and observer.
x0_ball = [0.; 0.0; 0.1; 0.000; -1.0];
x0_obs  = [0.; -0.012; 0.1; 0.045; -0.999000000002];
x0_cell = {x0_ball, x0_obs};

tspan = [0, 800];
jspan = [0, 200];

%% Solve coupled system.
sol = sys.solve(x0_cell, tspan, jspan, config);

close all;

figure(1)
HybridPlotBuilder().subplots('on')...
    .legend('$x$', '$y$', '$v_x$', '$v_y$', '$h$')...
    .plotFlows(sol('Billard').select(1:5));
grid on;

% figure(2)
% HybridPlotBuilder().subplots('on')...
%     .plotPhase(sol('Billard'));
% grid on;

figure(3)
plot(sol('Billard').x(:,1), sol('Billard').x(:,2), 'LineWidth', 1.8);
hold on;
plot(sol('Observer').x(:,1), sol('Observer').x(:,2), '--', 'LineWidth', 1.2);
grid on;
xlabel('x');
ylabel('y');
legend('plant', 'observer');

figure(4)
subplot(2,1,1);
plot(sol('Billard').t, sol('Billard').x(:,1), 'LineWidth', 1.5);
hold on;
plot(sol('Observer').t, sol('Observer').x(:,1), '--', 'LineWidth', 1.2);
ylabel('x');
legend('plant', 'observer');

subplot(2,1,2);
plot(sol('Billard').t, sol('Billard').x(:,2), 'LineWidth', 1.5);
hold on;
plot(sol('Observer').t, sol('Observer').x(:,2), '--', 'LineWidth', 1.2);
ylabel('y');
xlabel('time');

figure(5)
plot(sol('Billard').t, sol('Billard').x(:,5), 'LineWidth', 1.5);
hold on;
plot(sol('Observer').t, sol('Observer').x(:,5), '--', 'LineWidth', 1.2);
ylabel('ground height h');
xlabel('time');
legend('plant', 'observer');
grid on;

figure(6)
subplot(3,1,1);
plot(sol('Observer').t, sol('Observer').x(:,2) - sol('Billard').x(:,2));
ylabel('y error');

subplot(3,1,2);
plot(sol('Observer').t, sol('Observer').x(:,4) - sol('Billard').x(:,4));
ylabel('v_y error');

subplot(3,1,3);
plot(sol('Observer').t, sol('Observer').x(:,5) - sol('Billard').x(:,5));
ylabel('h error');
xlabel('time');

figure(8)
plot3(sol('Billard').x(:,2), sol('Billard').x(:,4), sol('Billard').x(:,5), 'LineWidth', 1.8);
hold on;
plot3(sol('Observer').x(:,2), sol('Observer').x(:,4), sol('Observer').x(:,5), '--', 'LineWidth', 1.2);
grid on;
xlabel('y');
ylabel('y dot');
zlabel('h');
title('Trajectories in (y, \dot{y}, h) space');
legend('plant', 'observer', 'Location', 'best');
view(3);

e = sol('Observer').x - sol('Billard').x;
far_jump_mask = (abs(e(:,2)) + abs(e(:,4)) < 1)';
far_jump_mask = sol('Observer').j - sol('Billard').j == 0;
e_peakless = e(far_jump_mask,:)*[0, 0, 0;
                                 1, 0, 0; 
                                 0, 0, 0;
                                 0, 1, 0;
                                 0, 0, 1];

P = [ 4.07565513e+02  2.90609975e-03 -7.62809940e-04;
      2.90609975e-03  3.45267495e+02  1.99423655e+00;
      -7.62809940e-04  1.99423655e+00  4.07503579e+02 ];

norm_p_error = diag(e_peakless*P*e_peakless');
figure(7);
plot(sol('Billard').t(far_jump_mask), norm_p_error);