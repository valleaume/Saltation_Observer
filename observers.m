addpath('utils', 'Examples/BouncingBall');

close all;
sys_ball = BouncingBallSubSystemClass();

sys_obs = BouncingBallObserver();
sys_obs.L_c = [0.1; 0.25];
sys_obs.L_d = 1*[0.1; 0.1];

sys_ball.mu = 0; % Additional velocity at each impact

sys_ball.lambda = 1;
sys_ball.f_air = 0;

sys_obs.mu = sys_ball.mu;
sys_obs.lambda = sys_ball.lambda;
sys_obs.f_air = sys_ball.f_air;


sys = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs);

obs_input = @(y_ball, ~) y_ball;
sys.setInput('Observer', obs_input);

sys

x0_cell = {[5; 2]; (1 - 0.0006)*[5; 2]};
tspan = [0, 250];
jspan = [0, 2450];

sol = sys.solve(x0_cell, tspan, jspan);
sol
figure(1)
HybridPlotBuilder().subplots('on')...
    .labels('$x_1$', '$x_2$')...
    .plotFlows(sol('Ball'))

hold on
HybridPlotBuilder().subplots('on')...
    .labels('$x_1$', '$x_2$')...
    .flowColor('#FF8800')...
    .jumpColor('m')...
    .jumpEndMarker('o')...
    .plotFlows(sol('Observer'))

figure(2)

plot(sol('Ball').t, sol('Ball').x(:,1) - sol('Observer').x(:,1));
title( "Position error");

figure(3)

plot(sol('Ball').t, sol('Ball').x(:,2) - sol('Observer').x(:,2));
title( "Velocity error");

figure(4)

e = sol('Ball').x - sol('Observer').x;
P = eye(2);
plot(sol('Ball').t, diag(e*P*e'));
title("Norm error");

figure(5)
plot(sol('Ball').t, sol('Ball').j - sol("Observer").j);


% BEWARE : only true for no air friction
F = [0, 1; 0, 0];
J = [1, 0; 0, -sys_ball.lambda];

H = [1, 0];
w = [1; 0];
tau = 15.6718 - 13.6045;
x = [0; -10.0995];
M_before = J - sys_obs.L_d*H - (J*sys_ball.flowMap(x, 0, 0, 0) - sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) )/x(2)*w';
M_after = M_before - sys_obs.L_d*H*(sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) - sys_ball.flowMap(x, 0, 0, 0))/x(2)*w';


abs(eig(M_before*expm((F-sys_obs.L_c*H)*tau)))
abs(eig(M_after*expm((F-sys_obs.L_c*H)*tau)))


