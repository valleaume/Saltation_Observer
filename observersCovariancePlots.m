%% observersCovariancePlots - Plot observer covariance analysis results
%
% This script loads generated or existing covariance data and creates various
% visualizations including:
%   - Flow plots of ball, observer, and Kalman trajectories
%   - Distribution of observer errors before/after jumps
%   - Saltation matrix analysis and covariance evolution
%   - Ellipse plots of error distributions
%
% Requires data to be pre-generated via observersCovarianceDataGeneration.m

addpath('utils');
close all;

% ====== USER CONFIGURATION ======
data_to_load = 'raw-bouncing-ball-after-before-05-Mar-2026.mat';  % Data file to analyze


% ====== LOAD SYSTEM CONFIGURATION ======
[sys, config, sys_ball, sys_obs, sys_obs_ref] = observersCovarianceConfig();


% ====== LOAD DATA ======
fprintf('Loading data from: data/%s\n', data_to_load);
dataset = load("data/"+data_to_load);
data_x = dataset.data_x;
data_v = dataset.data_v;
data_x_ref = dataset.data_x_ref;
data_v_ref = dataset.data_v_ref;
data_t = dataset.data_t;
data_jumps = dataset.data_jumps;
fprintf('Data loaded successfully!\n');


%% ====== SOLVE A SINGLE COUPLED SYSTEM FOR VISUALIZATION ======
x0_cell = {[1; 2]; [data_x(1,1); data_v(1,1)]; [data_x(1,1); data_v(1,1); reshape(eye(2), [4,1])]};
tspan = [0, 2];
jspan = [0, 5000];

sol = sys.solve(x0_cell, tspan, jspan, config);


%% ====== PLOT FLOW TRAJECTORIES ======
figure(2)
hpb = HybridPlotBuilder().subplots('on')...
    .labels('$x_1$', '$x_2$')...
    .legend('$x_1$', '$x_2$')...
    .plotFlows(sol('Ball'));
  
grid on;
hold on
hpb.subplots('on')...
    .flowColor('#FF8800')...
    .jumpColor('m')...
    .jumpEndMarker('o')...
    .legend('$\hat{x}_1$', '$\hat{x}_2$')...
    .plotFlows(sol('Observer').select(1:2))
hold on
hpb.subplots('on')...
    .flowColor('#168f2a')...
    .jumpColor('m')...
    .jumpEndMarker('x')...
    .legend('$\hat{x}^k_1$', '$\hat{x}^k_2$')...
    .plotFlows(sol('Kallman').select(1:2))


%% ====== DEFINE TIME POINTS FOR ANALYSIS ======
t_before = 0.69;      % Time before first jump
t_after = 0.71;       % Time after first jump
t_after_2 = 1.67;     % Time before second jump


%% ====== PLOT ERROR DISTRIBUTION AFTER FIRST JUMP ======
linear_indices_after = indices_from_time(t_after, data_t, data_x);

figure(7);
scatter(data_x(linear_indices_after)-data_x_ref(linear_indices_after), ...
        data_v(linear_indices_after)-data_v_ref(linear_indices_after), ...
        [], data_jumps(linear_indices_after), 'filled');
colormap('jet');
hold on;
scatter([0], [0], 0.100, 'filled', 'color', 'red');
hold on;
xline(0, "LineWidth", 1, "LineStyle", "-.");
xlabel('$x-x_{ref}$', 'Interpreter', 'latex');
ylabel('$v-v_{ref}$', 'Interpreter', 'latex');
title(sprintf('Distribution of observer error after jump (t=%.2f)', t_after));
axis equal;
grid on;


%% ====== PLOT ERROR DISTRIBUTION BEFORE FIRST JUMP ======
linear_indices_before = indices_from_time(t_before, data_t, data_x);

figure(6)
scatter(data_x(linear_indices_before)-data_x_ref(linear_indices_before), ...
        data_v(linear_indices_before)-data_v_ref(linear_indices_before), ...
        [], data_jumps(linear_indices_after), 'filled');
colormap('jet');
hold on;
scatter([0], [0], 0.100, 'filled', 'color', 'red');
hold on;

xlabel('$x-x_{ref}$', 'Interpreter', 'latex');
ylabel('$v-v_{ref}$', 'Interpreter', 'latex');
title(sprintf('Distribution of observer error before jump (t=%.2f)', t_before));
axis equal;
grid on;


%% ====== PLOT ERROR DISTRIBUTION BEFORE SECOND JUMP ======
linear_indices_after_2 = indices_from_time(t_after_2, data_t, data_x);

figure(5);
scatter(data_x(linear_indices_after_2)-data_x_ref(linear_indices_after_2), ...
        data_v(linear_indices_after_2)-data_v_ref(linear_indices_after_2), ...
        [], data_jumps(linear_indices_after), 'filled');
colormap('jet');
hold on;
xline(0, "LineWidth", 1, "LineStyle", "-.")
xlabel('$x-x_{ref}$', 'Interpreter', 'latex');
ylabel('$v-v_{ref}$', 'Interpreter', 'latex');
title(sprintf('Distribution of points before 2nd jump (t=%.2f)', t_after_2));
axis equal;
grid on;


%% ====== PLOT EMPIRICAL PROBABILITY OF w^T x > 0 OVER TIME ======
w = [1; 0];

n_times = size(data_t, 1);
probability_positive = nan(n_times, 1);
time_axis = nan(n_times, 1);

for k = 1:n_times
    current_t = data_t(k);
    % disp(current_t);
    linear_indices_current_t = indices_from_time(current_t, data_t, data_x);

    err = [data_x(linear_indices_current_t) - data_x_ref(linear_indices_current_t); ...
            data_v(linear_indices_current_t) - data_v_ref(linear_indices_current_t)];
    probability_positive(k) = mean(((w'*err + data_x_ref(linear_indices_current_t)) < 0));
    time_axis(k) = current_t;
    
end

%valid_plot = ~isnan(probability_positive) & ~isnan(time_axis);
disp(size(time_axis));
figure(8);
plot(time_axis, probability_positive, 'LineWidth', 2);
xlabel('Time $t$', 'Interpreter', 'latex');
ylabel('Empirical probability $\mathrm{P}(w^\top x > 0)$', ...
    'Interpreter', 'latex');
title('Empirical probability of the event over time');
grid on;
ylim([0, 1]);


%% ====== SALTATION MATRIX ANALYSIS - FIRST JUMP ======

F = [0, 1; 0, 0];
J = [1, 0; 0, -sys_ball.lambda];

H = [1, 0];
w = [1; 0];

x = [0; -4.85];
y = 0;

M_before = J - sys_obs.L_d*H - (J*sys_ball.flowMap(x, 0, 0, 0) - sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) )/(w'*sys_ball.flowMap(x, 0, 0, 0))*w';
M_after = M_before - sys_obs.L_d*H*(sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) - sys_ball.flowMap(x, 0, 0, 0))/(w'*sys_ball.flowMap(x, 0, 0, 0))*w';

fprintf('\n======== SALTATION MATRIX ANALYSIS ========\n');
fprintf('M_before:\n');
disp(M_before);

fprintf('M_after:\n');
disp(M_after);


%% ====== FIT AND COMPARE COVARIANCES ======

data_before = [data_x(linear_indices_before)-data_x_ref(linear_indices_before); ...
               data_v(linear_indices_before)-data_v_ref(linear_indices_before)];
fprintf('\nCovariance data size (before): %d\n', size(data_before, 2));

fprintf('Covariance before jump:\n');
cov_before = cov(data_before');
disp(cov_before);

data_after = [data_x(linear_indices_after)-data_x_ref(linear_indices_after); ...
              data_v(linear_indices_after)-data_v_ref(linear_indices_after)];

fprintf('Covariance after jump:\n');
cov_after = cov(data_after');
disp(cov_after);

mask_jump_before = (data_jumps(linear_indices_after) == 1);
mask_jump_after = (data_jumps(linear_indices_after) == -1);

fprintf('Covariance after jump (split by jump timing):\n');
fprintf('  Jumped before:\n');
disp(cov(data_after(:,mask_jump_before)'));
fprintf('  Jumped after:\n');
disp(cov(data_after(:,mask_jump_after)'));

fprintf('Covariance after jump (predicted via saltation):\n');
fprintf('  M_before prediction:\n');
disp(M_before*cov_before*M_before');
fprintf('  M_after prediction:\n');
disp(M_after*cov_before*M_after');

fprintf('Covariance after jump (theoretical via saltation, split by jump timing):\n');
fprintf('  M_before prediction:\n');
disp(M_before*cov(data_before(:, mask_jump_before)')*M_before');
fprintf('  M_after prediction:\n');
disp(M_after*cov(data_before(:, mask_jump_after)')*M_after');


%% ====== VISUALIZE COVARIANCE ELLIPSES ======
plot_ellipse(cov_before, mean(data_before, 2), 6, ...
    'LineWidth', 1, 'LineStyle', '-.');
xline(0, "LineWidth", 1, "LineStyle", "-.");
legend('jump after $x_{\rm ref}$', 'jump before $x_{\rm ref}$', ...
    "covariance", ...
    'hyperplane $\frac{\partial \hat{\omega}}{\partial \hat{x}}$', ...
    'Interpreter', 'latex', 'Location', 'northeast', ...
    'Box', 'off');


plot_ellipse(M_before*cov_before*M_before', mean(M_before*data_before, 2), 7, ...
    'Color', 'red', 'LineWidth', 3);
plot_ellipse(M_after*cov_before*M_after', mean(M_after*data_before, 2), 7, ...
    'Color', 'blue', 'LineWidth', 3);
legend('jump after $x_{\rm ref}$', 'jump before $x_{\rm ref}$', ...
    'hyperplane $\frac{\partial \hat{\omega}}{\partial \hat{x}}$', ...
    '$M_{\rm before}$', '$M_{\rm after}$', ...
    'Interpreter', 'latex', 'Location', 'northeast', ...
    'Box', 'off');
xlabel('$x-x_{ref}$', 'Interpreter', 'latex');
ylabel('$v-v_{ref}$', 'Interpreter', 'latex');
title('Covariance ellipses: measured vs. saltation predicted');
grid on;


%% ====== HELPER FUNCTIONS ======

function linear_indices = indices_from_time(t, data_t, data_x)
    % Find indices in data_x closest to a specific time t
    % data_t: time matrix (rows are different initial conditions)
    % Returns linear indices for indexing into data_x
    
    differences = abs(data_t - t);
    [~, j_mat] = min(differences, [], 1);

    [rows, cols] = size(data_x);
    linear_indices = sub2ind([rows, cols], j_mat, 1:cols);
end
