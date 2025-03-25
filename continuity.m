addpath('utils', 'Examples/BouncingBall', "Examples/StickSlip");
close all; % close all previously opened figures

% ReCreate the BouncingBall object.
sys = BouncingBallSystemClass();
sys.mu = 2; % Additional velocity at each impact

% Initial condition
X0_s = [[5.; 2.], [5.01; 2.01], [5.05; 2.], [5; 2.05], [5.1; 2.1], [5.1; 2.], [5; 2.1], [6; 7], [10; 1]];  % system initial condition

% Time spans
tspan = [0, 56];
jspan = [0, 245];

% Specify solver options.
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', 0.01 );
figure(1);
for i=1:9
    X0 = X0_s(:,i);
    % Compute solution of the cascade system - observer
    sol_test = sys.solve(X0, tspan, jspan, config);
    plot(sol_test.x(:,1), sol_test.x(:,2));
    hold on
end
%%
figure(2);

X0 = X0_s(:,1);
X1 = X0_s(:,3);
X2 = X0_s(:,8);
% Compute solution of the cascade system - observer
sol_test = sys.solve(X0, tspan, jspan, config);
sol_1 = sys.solve(X1, tspan, jspan, config);
sol_2 = sys.solve(X2, tspan, jspan, config);
plot(sol_test.t, sol_test.x(:,2))
hold on;
plot(sol_1.t, sol_1.x(:,2));
hold on;
plot(sol_2.t, sol_2.x(:,2));

%% StickSlip
close all;
sys = StickSlipSystemClass();

% Initial condition
X0_s = [[0; sys.v_t; 0.7; 0.1; 0], [0; sys.v_t; 0.7; 0.102; 0], [0; sys.v_t; 0.702; 0.1; 0], [0; sys.v_t; 0.701; 0.101; 0], [0; sys.v_t; 0.701; 0.1; 0], [0; sys.v_t; 0.7; 0.101; 0], [0; sys.v_t; 0.609; 0.1; 0], [0; sys.v_t; 0.7; 0.099; 0], [0; sys.v_t; 0.699; 0.096; 0], [0; sys.v_t+0.02; 0.7; 0.1; 1], [0; sys.v_t+0.1; 0.7; 0.1; 1], [0; sys.v_t+0.1; 0.698; 0.1; 1], [0; sys.v_t+0.1; 0.698; 0.1; 1]];  % system initial condition

% Time spans
tspan = [0, 56];
jspan = [0, 245];

% Specify solver options.
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', 0.01 );
figure(3);
disp(size(X0_s,2));
for i=1:size(X0_s,2)
    X0 = X0_s(:,i);
    % Compute solution of the cascade system - observer
    sol_test = sys.solve(X0, tspan, jspan, config);
    plot(sol_test.x(:,1), sol_test.x(:,2));
    hold on
end

figure(4);

X0 = X0_s(:,1);
X1 = X0_s(:,3);
X2 = X0_s(:,8);
% Compute solution of the cascade system - observer
sol_test = sys.solve(X0, tspan, jspan, config);
sol_1 = sys.solve(X1, tspan, jspan, config);
sol_2 = sys.solve(X2, tspan, jspan, config);
plot(sol_test.t, sol_test.x(:,2))
hold on;
plot(sol_1.t, sol_1.x(:,2));
hold on;
plot(sol_2.t, sol_2.x(:,2));

figure(5);
plot(sol_test.t, sol_test.x(:,1))
hold on;
plot(sol_1.t, sol_1.x(:,1));
hold on;
plot(sol_2.t, sol_2.x(:,1));

