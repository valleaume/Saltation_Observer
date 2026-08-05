addpath('utils');
close all;

% Define the plant and observer subsystems.
sys_plant = Toy3DSystemClass();
sys_obs = Toy3DObserverClass();

% Tune the observer gain and reset law.    1.4200
    0.4680
         0
sys_obs.Lc = [3; 2; 0; 0]; 
sys_obs.Ld_plus = [  379.2145;
                     295.8483;
                     -0.9388;
                    0];
sys_obs.Ld_minus = [46.1840;
                    29.9735;
                    -0.2051; 
                    0];
sys_obs.alpha = 0.;

% Build the coupled plant-observer system.
sys = CompositeHybridSystem('Plant', sys_plant, 'Observer', sys_obs);
obs_input = @(y_plant, ~) y_plant(:);
sys.setInput('Observer', obs_input);

% Initial conditions for the 4-state plant and observer.
x0_plant = [0.0; 0.1; 1; 1.0];
x0_obs = [0.02; 0.12; 1.01; 1.0];
x0_cell = {x0_plant, x0_obs};

% Simulation horizon.
tspan = [0, 100];
jspan = [0, 700];

% Define solver parameters.
max_dt_step = 0.05;
config = HybridSolverConfig('AbsTol', 1e-4, 'RelTol', 1e-7, 'MaxStep', max_dt_step);

% Solve the coupled system.
sol = sys.solve(x0_cell, tspan, jspan, config);

% Plot the plant and observer states.
figure(1)
HybridPlotBuilder().subplots('on')...
    .legend('$x_1$', '$x_2$', '$x_3$', '$q$')...
    .plotFlows(sol('Plant'));
grid on;

hold on;
HybridPlotBuilder().subplots('on')...
    .flowColor('#FF8800')...
    .jumpColor('m')...
    .jumpEndMarker('o')...
    .legend('$\hat{x}_1$', '$\hat{x}_2$', '$\hat{x}_3$', '$\hat{q}$')...
    .plotFlows(sol('Observer'));

figure(2)
subplot(2,1,1)
plot(sol('Plant').t, sol('Plant').x(:,1), 'LineWidth', 1.5);
hold on;
plot(sol('Observer').t, sol('Observer').x(:,1), '--', 'LineWidth', 1.2);
ylabel('x_1');
legend('plant', 'observer');
grid on;

subplot(2,1,2)
plot(sol('Plant').t, sol('Plant').x(:,2), 'LineWidth', 1.5);
hold on;
plot(sol('Observer').t, sol('Observer').x(:,2), '--', 'LineWidth', 1.2);
ylabel('x_2');
xlabel('time');
grid on;

figure(3)
plot(sol('Plant').t, sol('Plant').x(:,1) - sol('Observer').x(:,1), 'LineWidth', 1.2);
grid on;
xlabel('time');
ylabel('estimation error x_1');

title('ToySystem3D plant-observer cascade');
