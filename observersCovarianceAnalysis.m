addpath('utils');
close all;

GENERATE_POINTS = false;
data_to_load = 'raw-bouncing-ball-after-before-05-Mar-2026.mat';

% Define the plant subsystem
sys_ball = BouncingBallSubSystemClass();

sys_ball.mu = 0;        % Additional velocity at each impact
sys_ball.lambda = 1;    % Restitution coefficient
sys_ball.f_air = 0;     % Friction 

% Define the observer subsystem
sys_obs = BouncingBallObserver();

% Its dynamic is a copy of the plant's dynamic
sys_obs.mu = sys_ball.mu;
sys_obs.lambda = sys_ball.lambda;
sys_obs.f_air = sys_ball.f_air;

% Choose the observer gains

sys_obs.L_c = 2.6*[1.8; 1.6];   % Flow gains for a stable observer (not enough for convergence in every case, see the 20th init conditions for instance)
sys_obs.L_d = 2*[0.0; 1.1];    % Jump gain (make it high to see a discrepancy)
sys_obs.K = [0, 0];             % Gain on jump detection
%BEWARE: K(1) < 0.5 is necessary to enforce transversality


% Define the observer subsystem, regular kallman
sys_obs_ref = BouncingBallKallmanObserver();

% Its dynamic is a copy of the plant's dynamic
sys_obs_ref.mu = sys_ball.mu;
sys_obs_ref.lambda = sys_ball.lambda;
sys_obs_ref.f_air = sys_ball.f_air;

% Choose the observer gains
sys_obs_ref.gain = 0.23;
sys_obs_ref.lambda_kallman = 0.4;
sys_obs_ref.gamma_kallman = 1;
%sys_obs_ref.K = [0.24, -20];

% Deactivate saltation
sys_obs_ref.salted = false;


%% Define the coupled observerver-plant system 
sys = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs, 'Kallman', sys_obs_ref);
obs_input = @(y_ball, ~) y_ball;
sys.setInput('Observer', obs_input);

sys.setInput('Kallman', obs_input);

sys

% Define solver's parameter
max_dt_step = 0.03;
config = HybridSolverConfig('AbsTol', 1e-7, 'RelTol', 1e-7, 'MaxStep', max_dt_step);

% Number of random points to generate
n = 1000;

mu = [1; 2];          % Mean vector (expectation)
sigma = 1e-6*[2 0; 0 2];      % Covariance matrix

% Generate random points
rng('default'); % For reproducibility (optional)
points = mvnrnd(mu, sigma, n);

% Plot the trajectory
index = 1;
x0_cell = {[1; 2]; [points(index, 1); points(index, 2)]; [points(index, 1); points(index, 2); reshape(sigma, [4,1])]};
tspan = [0, 15];
jspan = [0, 5000];

% Generate Points
if GENERATE_POINTS

    % Plot the initial conditions
    figure(1);
    scatter(points(:,1), points(:,2), 'filled');
    xlabel('x');
    ylabel('v');
    title('Initial distribution of points (t=0)');
    axis equal;
    grid on;


    % Propagate those points
    disp('Solving for those initial conditions')
    data_x = {};
    data_t = {};
    data_v = {};
    data_x_ref = {};
    data_v_ref = {};
    data_t_ref = {};
    observer_jumps_before = {};

    for index = 1:n
        % X_0 is first element of cell, hat{X_0} is the second
        x0_cell = {[1; 2]; [points(index, 1); points(index, 2)]; [points(index, 1); points(index, 2); reshape(eye(2), [4,1])]};
        tspan = [0, 2];
        jspan = [0, 15];

        % Solve coupled system 
        sol = sys.solve(x0_cell, tspan, jspan, config);
        data_x{end+1}  = sol('Observer').x(:,1);
        data_v{end+1}  = sol('Observer').x(:,2);
        data_x_ref{end+1}  = sol('Ball').x(:,1);
        data_v_ref{end+1}  = sol('Ball').x(:,2);
        data_t_ref{end+1}  = sol('Ball').t;
        data_t{end+1} = sol('Observer').t;

        assert(isequal(sol('Ball').t, sol('Observer').t), "discrepancy in time index")
        mask_jump_after = (sol('Ball').j - sol('Observer').j) > 0;
        mask_jump_before = (sol('Ball').j - sol('Observer').j) < 0;
        sign_jump = zeros(1,length(mask_jump_after));
        for i=2:length(mask_jump_after)
            if mask_jump_before(i)
                sign_jump(i) = +1;
            else
                if mask_jump_after(i)
                    sign_jump(i) = -1;
                else
                    sign_jump(i) = sign_jump(i-1);
                end
            end
        end
        observer_jumps_before{end+1} = sign_jump;
    end

    %{  
    one big matrix, not really better for indexing 

    data = cat(3, data{:});
    [rows, cols, depths] = size(data);
    disp([rows, cols, depths])
    pos_indices = sub2ind([rows, cols, depths], j_mat, 1, 1:depths);
    velocity_indices = sub2ind([rows, cols, depths], j_mat, 2, 1:depths);
    jump_indices = sub2ind([rows, depths], j_mat, 1:depths);
    %}
    
    

    data_x = cell2mat(padCellToUniformSize(data_x, NaN));
    data_t = cell2mat(padCellToUniformSize(data_t, NaN));
    data_v = cell2mat(padCellToUniformSize(data_v, NaN));

    data_x_ref = cell2mat(padCellToUniformSize(data_x_ref, NaN));
    data_t_ref = cell2mat(padCellToUniformSize(data_t_ref, NaN));
    data_v_ref = cell2mat(padCellToUniformSize(data_v_ref, NaN));
    data_jumps = cell2mat(padCellToUniformSize(observer_jumps_before, NaN));

    % Save dataset
    today = string(datetime("today"));
    datas_filename = strcat('data/raw-bouncing-ball-kallman-', today);
    save(datas_filename, "data_x", "data_v", "data_t", "data_v_ref", "data_x_ref", "data_jumps")   %"data_t_ref"

else
    % Load previously computed points
    dataset = load("data/"+data_to_load);
    data_x = dataset.data_x;
    data_v = dataset.data_v;
    data_x_ref = dataset.data_x_ref;
    data_v_ref = dataset.data_v_ref;
    data_t = dataset.data_t;
    data_jumps = dataset.data_jumps;
end 

%% Solve coupled system 
sol = sys.solve(x0_cell, tspan, jspan, config);

% Plot flow 
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
 


%% Define auxiliary function
function linear_indices = indices_from_time(t, data_t, data_x)
    differences = abs(data_t - t);
    [~, j_mat] = min(differences, [], 1);

    [rows, cols] = size(data_x);
    linear_indices = sub2ind([rows, cols], j_mat, 1:cols);
end

function sigma_points = unscentedTranform(point, cov)
 %Todo
end
% unscentedFilter = unscentedKalmanFilter()

%% Plot the distributions for different time steps

t_after = 0.71;
t_before = 0.69;
t_after_2 = 1.67;

% Plot the points after a jump
linear_indices_after = indices_from_time(t_after, data_t, data_x);
mask_jump_before = (data_jumps(linear_indices_after) == 1);
mask_jump_after = (data_jumps(linear_indices_after) == -1);

fig_after = figure(7);
scatter(data_x(linear_indices_after)-data_x_ref(linear_indices_after), data_v(linear_indices_after)-data_v_ref(linear_indices_after), [], data_jumps(linear_indices_after), 'filled');
colormap('jet');
hold on;
scatter([0], [0], 0.100, 'filled', 'color', 'red');
hold on;
xline(0, "LineWidth", 1, "LineStyle", "-.")
xlabel('$x-x_{ref}$', 'Interpreter', 'latex');
ylabel('$v-v_{ref}$', 'Interpreter', 'latex');
%title(sprintf('Distribution of observer error after jump (t=%.2f)', t_after));
axis equal;
grid on;

% Plot the points error distribution before a jump
linear_indices_before = indices_from_time(t_before, data_t, data_x);


fig_before = figure(6);
scatter(data_x(linear_indices_before)-data_x_ref(linear_indices_before), data_v(linear_indices_before)-data_v_ref(linear_indices_before), [], data_jumps(linear_indices_after), 'filled'); % use the color of whether they will jump or not
colormap('jet');
hold on;
scatter([0], [0], 0.100, 'filled', 'color', 'red');
xlabel('$x-x_{ref}$', 'Interpreter', 'latex');
ylabel('$v-v_{ref}$', 'Interpreter', 'latex');
%title(sprintf('Distribution of observer error before jump (t=%.2f)', t_before));
axis equal;
grid on;

% Plot the points before second jump

linear_indices_after_2 = indices_from_time(t_after_2, data_t, data_x);

figure(5);
scatter(data_x(linear_indices_after_2)-data_x_ref(linear_indices_after_2), data_v(linear_indices_after_2)-data_v_ref(linear_indices_after_2), [], data_jumps(linear_indices_after), 'filled'); % use the color of whether they will jump or not
colormap('jet');
hold on;
xline(0, "LineWidth", 1, "LineStyle", "-.")
xlabel('$x-x_{ref}$', 'Interpreter', 'latex');
ylabel('$v-v_{ref}$', 'Interpreter', 'latex');
title(sprintf('Distribution of points before 2nd jump (t=%.2f)', t_after_2));
axis equal;
grid on;
%% Saltation matrices 1rst jump

F = [0, 1; 0, 0];
J = [1, 0; 0, -sys_ball.lambda];

H = [1, 0];
w = [1; 0];

x = [0; -4.85];
y = 0;

M_before = J - sys_obs.L_d*H - (J*sys_ball.flowMap(x, 0, 0, 0) - sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) )/(w'*sys_ball.flowMap(x, 0, 0, 0))*w';
M_after = M_before - sys_obs.L_d*H*(sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) - sys_ball.flowMap(x, 0, 0, 0))/(w'*sys_ball.flowMap(x, 0, 0, 0))*w';

disp('M_before');
disp(M_before);

disp('M_after');
disp(M_after);

% Fit covariances
data_before = [data_x(linear_indices_before)-data_x_ref(linear_indices_before); data_v(linear_indices_before)-data_v_ref(linear_indices_before)];
disp(size(data_before));
disp('Covariance before jump');
cov_before = cov(data_before');
disp(cov_before);

data_after = [data_x(linear_indices_after)-data_x_ref(linear_indices_after); data_v(linear_indices_after)-data_v_ref(linear_indices_after)];

disp('Covariance after jump');
cov_after = cov(data_after');
disp(cov_after);

mask_jump_before = (data_jumps(linear_indices_after) == 1);
mask_jump_after = (data_jumps(linear_indices_after) == -1);

disp('Covariance after jump, 2 datasets');
disp(cov(data_after(:,mask_jump_before)'));
disp(cov(data_after(:,mask_jump_after)'));

disp('Covariance after jump, multiplied');
disp(M_before*cov_before*M_before');
disp(M_after*cov_before*M_after');

disp('Covariance after jump, theoritical');
disp(M_before*cov(data_before(:, mask_jump_before)')*M_before');
disp(M_after*cov(data_before(:, mask_jump_after)')*M_after');


plot_ellipse(cov_before, mean(data_before, 2), 6, 'linewidth', 1, 'linestyle', '-.');

sigma = 1e-6*[2 0; 0 2];
flow = exp(t_before*F);
cov_before_th = flow*sigma*flow';
%plot_ellipse(cov_before_th, mean(data_before, 2), 6, 'linewidth', 2, 'linestyle', '-.');
legend('jump after $x_{\rm ref}$', 'jump before $x_{\rm ref}$', 'covariance', 'interpreter', 'latex', 'location', 'northwest');
xlim([-3.8e-3, 2.e-3]);
ylim([-4.1e-3, 4.3e-3]);

%plot_ellipse(cov_after, mean(data_after, 2), 7);
plot_ellipse(M_before*cov_before*M_before', mean(M_before*data_before, 2), 7, 'color', 'red', 'linewidth', 3);
plot_ellipse(M_after*cov_before*M_after', mean(M_after*data_before, 2), 7, 'color', 'blue', 'linewidth', 3);
legend('jump after $x_{\rm ref}$', 'jump before $x_{\rm ref}$','hyperplane $\frac{\partial \hat{\omega}}{\partial \hat{x}}$', '$M_{\rm before}$', '$M_{\rm after}$', 'interpreter', 'latex', 'location', 'northeast');
xlim([-2e-3, 3.2e-3]);

%plot_ellipse(cov(data_after(:,mask_jump_before)'), mean(data_after(:,mask_jump_before), 2), 4);
%plot_ellipse(cov(data_after(:,mask_jump_after)'), mean(data_after(:,mask_jump_after), 2), 4);

myPrintPDF(fig_before, 'figures/Before_covariance_ellipse_2024', [10,12]);
myPrintPDF(fig_after, 'figures/M_after_before_ellipses_2024');
