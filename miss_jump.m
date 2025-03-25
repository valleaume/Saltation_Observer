addpath('utils', 'Examples/BouncingBall', "Examples/StickSlip");
close all; % close all previously opened figures

% ReCreate the BouncingBall object.
sys = ObserverBouncingBallSystemClass();
sys.mu = 2; % Additional velocity at each impact
%sys.mu = 0;
sys.lambda = 0.8;
sys.f_air = 0;
sys.k1 = 0;
sys.Lc_1 = 0.10;
sys.Lc_2 = 0.25;
sys.Ld_1 = 0.10;
sys.Ld_2 = 0.10;

% Initial condition
X0 = [[5.; 2.]; (1 - 0.06)*[ 5.0; 2.0]];  % system initial condition %[4.05; 2.; 5; 2.05]]

% Time spans
tspan = [0, 900];
jspan = [0, 2045];

% Specify solver options.
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', 0.01 );

% Compute solution of the cascade system - observer
sol_test = sys.solve(X0, tspan, jspan, config);
sol_test
figure(1)
plot(sol_test.x(:,1), sol_test.x(:,2));
hold on;
plot(sol_test.x(:,3), sol_test.x(:,4));

figure(2)
plot(sol_test.t, sol_test.x(:,1));
hold on;
plot(sol_test.t, sol_test.x(:,3));

figure(3)
plot(sol_test.t, sol_test.x(:,2));
hold on;
plot(sol_test.t, sol_test.x(:,4));



