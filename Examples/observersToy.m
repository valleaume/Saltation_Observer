addpath('utils');
close all;

% Define the plant subsystem
sys_ball = ToySystemClass();
sys_ball.speed = 0.7;


% Define the observer subsystem
sys_obs = ToySystemObserverClass();
sys_obs.speed = sys_ball.speed;

% Choose the observer gains

sys_obs.L_c = 0;   % Flow gain
sys_obs.L_d = 0; % Jump gain
sys_obs.K_jump = -0.05;



% Define the coupled observerver-plant system 
sys = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs);
obs_input = @(y_ball, ~) y_ball;
sys.setInput('Observer', obs_input);

sys

% Define solver's parameter
max_dt_step = 0.1;
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', max_dt_step);

% X_0 is first element of cell, hat{X_0} is the second
x0_cell = {[-2]; (1-6e-2)*[-2]};
tspan = [0, 50];
jspan = [0, 2450];

%% Solve coupled system 
sol = sys.solve(x0_cell, tspan, jspan, config);
sol

%% Plot results

% Preprocess : detect if observer jumps before or after the system
mask_jump_after = (sol('Ball').j - sol("Observer").j) > 0;
mask_jump_before = (sol('Ball').j - sol("Observer").j) < 0;

sign_jump = zeros(1,length(mask_jump_after));
for i=2:length(mask_jump_after)
    if mask_jump_before(i)
        sign_jump(i) = -1;
    else
        if mask_jump_after(i)
            sign_jump(i) = +1;
        else
            sign_jump(i) = sign_jump(i-1);
        end
    end
end

% Plot Flow 
figure(1)
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
    .plotFlows(sol('Observer').select(1))
 

% Plot output
figure(2)
plot(sol('Ball').t, sol('Ball').x(:,1).^2);
grid on;
hold on;
plot(sol('Observer').t, sol('Observer').x(:,1).^2);

xlabel('$t$', Interpreter= 'latex');
ylabel('$y$', Interpreter= 'latex');
legend('$y$', '$\hat{y}$', Interpreter= 'latex');
title( "Output");



% Plot position error
figure(3)
plot(sol('Ball').t, sol('Ball').x(:,1) - sol('Observer').x(:,1));
grid on;
xlabel('$t$', Interpreter= 'latex');
ylabel('$x_1- \hat{x}_1$', Interpreter= 'latex');
title( "Position error");

% "Phase" output
figure(4)
plot(sol('Ball').x(:,1), sol('Ball').x(:,1).^2);
grid on;
hold on;
plot(sol('Observer').x(:,1), sol('Ball').x(:,1).^2);

xlabel('$x$', Interpreter= 'latex');
ylabel('$y$', Interpreter= 'latex');
legend('system', 'observer', Interpreter= 'latex');
title( "Phase-ish plot");

