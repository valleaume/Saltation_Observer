addpath('utils');
close all;

% ReCreate the BouncingBall object.
sys = ObserverBouncingBallSystemClass();
sys.mu =0;% 2; % Additional velocity at each impact %2
%sys.mu = 0;
sys.lambda = 1;%0.8;
sys.f_air = 0; %0.01
sys.k1 = 3;
sys.Lc_1 = 10;
sys.Lc_2 = 25;
sys.Ld_1 = 0.00;
sys.Ld_2 = 0.00;

% Initial condition
X0 = [[5.; 2.]; (1 + 0.06)*[ 5.0; 2.0]];  % system initial condition %[4.05; 2.; 5; 2.05]]

% Time spans
tspan = [0, 90];
jspan = [0, 204];

% Specify solver options.
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', 0.01 );

% Compute solution of the cascade system - observer
sol_test = sys.solve(X0, tspan, jspan, config);

%% Phase

cf = figure(1);
plot(sol_test.x(:,1), sol_test.x(:,2));
hold on;
plot(sol_test.x(:,3), sol_test.x(:,4), "LineStyle",":");
grid on;
xlabel('$x_1$', Interpreter= 'latex');
ylabel('$x_2$', Interpreter= 'latex');
cl = legend('$x$', '$\hat{x}$', Interpreter = 'latex');
cf.CurrentAxes.XLim = [-0.5, 5.5];

myPrintPDF(cf,  'figures\Miss_jump');

%% Position
cf = figure(2);
plot(sol_test.t, sol_test.x(:,1));
hold on;
plot(sol_test.t, sol_test.x(:,3), "LineStyle", ":");
grid on;
xlabel('$t$', Interpreter='latex');
ylabel('$x_1$', Interpreter= 'latex');
cl =legend('$x$', '$\hat{x}$',Interpreter = 'latex');
cf.CurrentAxes.XLim = [0, 16];
cf.CurrentAxes.YLim = [-0.6, 6];

myPrintPDF(cf,  'figures\Miss_jump_x');
%% Velocity

cf = figure(3);
plot(sol_test.t, sol_test.x(:,2));
hold on;
plot(sol_test.t, sol_test.x(:,4), "LineStyle", ":");
grid on;
xlabel('$t$', Interpreter= 'latex');
ylabel('$x_2$', Interpreter= 'latex');
legend('$x$', '$\hat{x}$',Interpreter = 'latex');
cf.CurrentAxes.XLim = [0, 16];

myPrintPDF(cf,  'figures\Miss_jump_v');


%%

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
sys_obs.L_c = [0.1; 0.25];   % Flow gain
sys_obs.L_d = [0.1; 0.1]; % Jump gain
sys_obs.K = [0, 0];         % Gain on jump detection
%BEWARE: K(1) < 0.5 is necessary to enforce transversality


% Define the coupled observerver-plant system 
sys = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs);
obs_input = @(y_ball, ~) y_ball;
sys.setInput('Observer', obs_input);


% Define solver's parameter
max_dt_step = 0.1;
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', max_dt_step);

% X_0 is first element of cell, hat{X_0} is the second
x0_cell = {[5; 2]; (1 - 6e-3)*[5; 2]};
tspan = [0, 125];
jspan = [0, 2450];

%% Solve coupled system 
sol_unstable = sys.solve(x0_cell, tspan, jspan, config);

%% Generate Stable Result

sys_obs.L_c = [0.8; 0.6];   % Flow gains for a stable observer

% Define the modified observerver-plant system 
sys_stable = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs);
obs_input = @(y_ball, ~) y_ball;
sys_stable.setInput('Observer', obs_input);

x0_cell = {[5; 2]; (1 - 6e-2)*[5; 2]};
sol_synchronized = sys_stable.solve(x0_cell, tspan, jspan, config);


%% Plot position error
cf = figure(4);
plot(sol_stable('Ball').t, sol_stable('Ball').x(:,1) - sol_stable('Observer').x(:,1), 'Color', 'green');
hold on;
plot(sol_unstable('Ball').t, sol_unstable('Ball').x(:,1) - sol_unstable('Observer').x(:,1), 'Linestyle', ":", 'Color', 'red');
grid on;
xlabel('$t$', Interpreter= 'latex');
ylabel('$x_1- \hat{x}_1$', Interpreter= 'latex');
legend("$\mathcal{M}_{ \rm after}$ stable", "$\mathcal{M}_{ \rm after}$ unstable")
title( "Position error");
cf.CurrentAxes.XLim = [0, 85];

myPrintPDF(cf, 'figures\Position error')

%% Plot velocity error
cf = figure(5);
plot(sol_stable('Ball').t, sol_stable('Ball').x(:,2) - sol_stable('Observer').x(:,2), 'Color', 'green');
hold on;
plot(sol_unstable('Ball').t, sol_unstable('Ball').x(:,2) - sol_unstable('Observer').x(:,2), 'Linestyle', ":", 'Color', 'red');
grid on;
xlabel('$t$', Interpreter= 'latex');
ylabel('$x_1- \hat{x}_1$', Interpreter= 'latex');
legend("$\mathcal{M}_{ \rm after}$ stable", "$\mathcal{M}_{ \rm after}$ unstable")
title( "Velocity error");

cf.CurrentAxes.XLim = [0, 75];

myPrintPDF(cf, 'figures\Velocity error')

%% Plot norm of error

% Important : for clarity we used initial conditions closer to each other (10-3,10-2)

cf = figure(6);
e_stable = sol_stable('Ball').x - sol_stable('Observer').x;
e_unstable = sol_unstable('Ball').x - sol_unstable('Observer').x;
P = eye(2);  %Here you can put a P calculated in K_search_LMI

% Trying to get rid of spikes
% Arbitrary treshold on velocity error to get rid of them
far_jump_mask_stable = (abs(e_stable(:,2))<10)';
far_jump_mask_unstable = (abs(e_unstable(:,2))<10)';
e_stable = e_stable(far_jump_mask_stable,:);
e_unstable = e_unstable(far_jump_mask_unstable,:);
 

plot(sol_stable('Ball').t(far_jump_mask_stable), diag(e_stable*P*e_stable'), color='green'); 
hold on;
grid on;
plot(sol_unstable('Ball').t(far_jump_mask_unstable), diag(e_unstable*P*e_unstable'),  'Linestyle', ":", color='red'); 
legend("$\mathcal{M}_{ \rm after}$ stable", "$\mathcal{M}_{ \rm after}$ unstable")
xlabel('$t$', 'Interpreter', 'Latex')
ylabel('$\|x-\hat{x}\|^2$',  'Interpreter', 'Latex')
%title("Norm error");

cf.CurrentAxes.XLim = [0, 70];
cf.CurrentAxes.YLim = [0, 0.16];

myPrintPDF(cf, 'figures\Norm_error')

%% Synchronization error

tspan = [0, 25];

% Synchronized system
sys_obs.L_c = [0; 0];       % Flow gain
sys_obs.L_d = [1; -0.392];  % Jump gain
sys_obs.K = [1, 0];         % Gain on jump detection that gives perfect jump detection

% Define the modified observerver-plant system 
sys_synchronized = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs);
obs_input = @(y_ball, ~) y_ball;
sys_synchronized.setInput('Observer', obs_input);

x0_cell = {[5; 2]; (1 + 6e-1)*[5; 2]};
sol_synchronized = sys_synchronized.solve(x0_cell, tspan, jspan, config);

% Not synchronized system

sys_obs.K = [0, 0];         % Gain on jump detection

% Define the modified observerver-plant system 
sys_non_synchronized = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs);
obs_input = @(y_ball, ~) y_ball;
sys_non_synchronized.setInput('Observer', obs_input);

x0_cell = {[5; 2]; (1 + 6e-3)*[5; 2]};
sol_non_synchronized = sys_non_synchronized.solve(x0_cell, tspan, jspan, config);


%% Plot position
cf = figure(7);
plot(sol_synchronized('Ball').t, sol_synchronized('Ball').x(:,1), 'Color', 'blue');
hold on;
plot(sol_synchronized('Ball').t, sol_synchronized('Observer').x(:,1), 'Color', 'green', 'Linestyle', "-");
hold on;
plot(sol_non_synchronized('Ball').t, sol_non_synchronized('Observer').x(:,1), 'Linestyle', ":", 'Color', 'red');
grid on;
xlabel('$t$', Interpreter= 'latex');
ylabel('$x_1$', Interpreter= 'latex');
legend("$x$", "$\hat{x}_{\rm known}$", "$\hat{x}_{\rm unknown}$")
cf.CurrentAxes.XLim = [0, 16];
cf.CurrentAxes.YLim = [-0.5, 11];

myPrintPDF(cf, 'figures\Position_synchronization')
