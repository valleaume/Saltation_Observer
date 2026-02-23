addpath('utils');
close all;

GENERATE_POINTS = false;
data_to_load = 'raw-bouncing-ball-11-Feb-2026.mat';

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


%% Define the coupled observerver-plant system 
sys = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs, 'Kallman_Ref', sys_obs_ref);
obs_input = @(y_ball, ~) y_ball;
sys.setInput('Observer', obs_input);

sys.setInput('Kallman_Ref', obs_input);

sys

% Define solver's parameter
max_dt_step = 0.05;
config = HybridSolverConfig('AbsTol', 1e-4, 'RelTol', 1e-7, 'MaxStep', max_dt_step);

if GENERATE_POINTS

    % Number of random points to generate
    n = 3000;

    mu = [1; 2];          % Mean vector (expectation)
    sigma = 1e-5*[2 0; 0 2];      % Covariance matrix

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


    % Propagate those points
    disp('Solving for those initial conditions')
    data_x = {};
    data_t = {};
    data_v = {};
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
        data_t{end+1} = sol('Observer').t;

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


    data_x = cell2mat(data_x);
    data_t = cell2mat(data_t);
    data_v = cell2mat(data_v);
    data_jumps = cell2mat(observer_jumps_before);

    % Save dataset
    today = string(datetime("today"));
    datas_filename = strcat('data/raw-bouncing-ball-', today);
    save(datas_filename, "data_x", "data_v", "data_t", "data_jumps")  

else
    % Load previously computed points
    dataset = load("data/"+data_to_load);
    data_x = dataset.data_x;
    data_v = dataset.data_v;
    data_t = dataset.data_t;
    data_jumps = dataset.data_jumps;
end 

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

%% Define auxiliary function
function linear_indices = indices_from_time(t, data_t, data_x)
    differences = abs(data_t - t);
    [~, j_mat] = min(differences, [], 1);

    [rows, cols] = size(data_x);
    linear_indices = sub2ind([rows, cols], j_mat, 1:cols);
end

%% Plot the points distribution before a jump
t_before = 0.69;
linear_indices_before = indices_from_time(t_before, data_t, data_x);

figure(3)
scatter(data_x(linear_indices_before), data_v(linear_indices_before), [], data_jumps(linear_indices_before), 'filled');
colormap('jet');
xlabel('x');
ylabel('v');
title(sprintf('Distribution of points before jump (t=%.2f)', t_before));
axis equal;
grid on;

% Plot the points after a jump
t_after = 0.73;
linear_indices_after = indices_from_time(t_after, data_t, data_x);

%disp(data_x(linear_indices))  for test purposes, 41
%disp(data_x(41,:))

figure(4);
scatter(data_x(linear_indices_after), data_v(linear_indices_after), [], data_jumps(linear_indices_after), 'filled');
colormap('jet');
xlabel('x');
ylabel('v');
title(sprintf('Distribution of points after jump (t=%.2f)', t_after));
axis equal;
grid on;

% Plot the points before second jump
t_after = 1.67;
linear_indices = indices_from_time(t_after, data_t, data_x);


figure(5);
scatter(data_x(linear_indices), data_v(linear_indices), [], data_jumps(linear_indices), 'filled');
colormap('jet');
xlabel('x');
ylabel('v');
title(sprintf('Distribution of points before 2nd jump (t=%.2f)', t_after));
axis equal;
grid on;

%% Saltation matrices 1rst jump

F = [0, 1; 0, 0];
J = [1, 0; 0, -sys_ball.lambda];

H = [1, 0];
w = [1; 0];

x = [0; -4.85];
M_before = J - sys_obs.L_d*H - (J*sys_ball.flowMap(x, 0, 0, 0) - sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) )/x(2)*w';
M_after = M_before - sys_obs.L_d*H*(sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) - sys_ball.flowMap(x, 0, 0, 0))/x(2)*w';

% Fit covariances
data_before = [data_x(linear_indices_before); data_v(linear_indices_before)];
disp(size(data_before));
disp('Covariance before jump');
cov_before = cov(data_before');
disp(cov_before);

data_after = [data_x(linear_indices_after); data_v(linear_indices_after)];

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

% Source - https://stackoverflow.com/a/34308544
% Posted by fhoussiau, modified by community. See post 'Timeline' for change history
% Retrieved 2026-02-11, License - CC BY-SA 3.0

function plot_ellipse(C, mean, num_fig)
    npts=50;
    % plot the gaussian fits
    tt=linspace(0,2*pi,npts)';
    x = cos(tt); y=sin(tt);
    ap = [x(:) y(:)]';
    [v,d]=eig(C); 
    d = 2 * sqrt(d); % convert variance to sdwidth*sd
    disp(v*d*ap)
    disp(repmat(mean, 1, size(ap,2)))
    bp = (v*d*ap) + repmat(mean, 1, size(ap,2)); 
    figure(num_fig);
    hold on;
    plot(bp(1,:), bp(2,:));
end 

plot_ellipse(cov_before, mean(data_before, 2), 3);

plot_ellipse(cov_after, mean(data_after, 2), 4);
plot_ellipse(cov(data_after(:,mask_jump_before)'), mean(data_after(:,mask_jump_before), 2), 4);
plot_ellipse(cov(data_after(:,mask_jump_after)'), mean(data_after(:,mask_jump_after), 2), 4);