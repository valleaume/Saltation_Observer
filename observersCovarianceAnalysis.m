addpath('utils');
close all;

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

sys_obs.L_c = [1.8; 1.6];   % Flow gains for a stable observer (not enough for convergence in every case, see the 20th init conditions for instance)
sys_obs.L_d = 1*[0.0; 0.1]; % Jump gain
sys_obs.K = [0, 0];         % Gain on jump detection
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


% Deactivate saltation
sys_obs_ref.salted = false;


% Define the coupled observerver-plant system 
sys = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs, 'Kallman_Ref', sys_obs_ref);
obs_input = @(y_ball, ~) y_ball;
sys.setInput('Observer', obs_input);

sys.setInput('Kallman_Ref', obs_input);

sys

% Define solver's parameter
max_dt_step = 0.1;
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', max_dt_step);

% Generate random points
mu = [1; 2];          % Mean vector (expectation)
sigma = 1e-5*[2 0;       % Covariance matrix
         0 2];

% Number of random points to generate
n = 1000;

% Generate random points
rng('default'); % For reproducibility (optional)
points = mvnrnd(mu, sigma, n);

% Plot the initial conditions
figure(1);
scatter(points(:,1), points(:,2), 'filled');
xlabel('x');
ylabel('v');
title('Initial distribution of points (t=0)');
axis equal;
grid on;

data = {};
data_t = {};
data_v = {};
observer_jumps_before = zeros(1, n);


for index = 1:n
    % X_0 is first element of cell, hat{X_0} is the second
    x0_cell = {[1; 2]; [points(index, 1); points(index, 2)]; [points(index, 1); points(index, 2); reshape(eye(2), [4,1])]};
    tspan = [0, 2];
    jspan = [0, 15];
    % Solve coupled system 
    sol = sys.solve(x0_cell, tspan, jspan, config);
    data{end+1}  = sol('Observer').x(:,1);
    data_v{end+1}  = sol('Observer').x(:,2);
    data_t{end+1} = sol('Observer').t;

end

data = cell2mat(data);
data_t = cell2mat(data_t);
data_v = cell2mat(data_v)

x0_cell = {[1; 2]; [points(index, 1); points(index, 2)]; [points(index, 1); points(index, 2); reshape(eye(2), [4,1])]};
tspan = [0, 25];
jspan = [0, 5000];

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
    .plotFlows(sol('Kallman_Ref').select(1:2))
 

data;

% Plot the points distribution before a jump
t_before = 0.66;
differences = abs(data_t - t_before);
[~, j_mat] = min(differences, [], 1);
disp(j_mat);

[rows, cols] = size(data);
linear_indices = sub2ind([rows, cols], j_mat, 1:cols);

figure(3);
scatter(data(linear_indices), data_v(linear_indices), 'filled');
xlabel('x');
ylabel('v');
title(sprintf('Distribution of points before jump (t=%.2f)', t_before));
axis equal;
grid on;

% Plot the points after a jump
t_after = 0.74;
differences = abs(data_t - t_after);
[~, j_mat] = min(differences, [], 1);
disp(j_mat);

[rows, cols] = size(data);
linear_indices = sub2ind([rows, cols], j_mat, 1:cols);

%disp(data(linear_indices))  for test purposes, 41
%disp(data(41,:))

figure(4);
scatter(data(linear_indices), data_v(linear_indices), 'filled');
xlabel('x');
ylabel('v');
title(sprintf('Distribution of points after jump (t=%.2f)', t_after));
axis equal;
grid on;

% Plot the points before second jump
t_after = 1.67;
differences = abs(data_t - t_after);
[~, j_mat] = min(differences, [], 1);
disp(j_mat);

[rows, cols] = size(data);
linear_indices = sub2ind([rows, cols], j_mat, 1:cols);


figure(5);
scatter(data(linear_indices), data_v(linear_indices), 'filled');
xlabel('x');
ylabel('v');
title(sprintf('Distribution of points before 2nd jump (t=%.2f)', t_after));
axis equal;
grid on;