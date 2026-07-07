addpath('./utils');
close;

sys_billard = Billard_sys();
sys_copy = Billard_obs();
%sys_copy.L = [0; 0; 0; 0 ;0];

% Define solver's parameter
max_dt_step = 0.1;
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', max_dt_step);
sys = CompositeHybridSystem('Billard', sys_billard, 'Observer', sys_copy);
obs_input = @(y_ball, ~) y_ball;
sys.setInput('Observer', obs_input);

% X_0 is first element of cell
disp(sys_copy.init_cond([0.5, 0.5, 7*pi/4]));
disp(sys_copy.init_cond([0.5, 0.3, pi/3]));
x0_cell = {[0.5; 0.5; 7*pi/4], sys_copy.init_cond([0.5; 0.61; 7.7*pi/4])};
tspan = [0, 149];
jspan = [0, 180];

%% Solve coupled system 
sol = sys.solve(x0_cell, tspan, jspan, config);

close all;

figure(1)
hpb = HybridPlotBuilder().subplots('on')...
    .legend('$x_b$', '$y_b$', '$\theta_b$')...
    .plotFlows(sol('Billard').select(1:3));
grid on;

figure(2)
hpb = HybridPlotBuilder().subplots('on')...
    .plotPhase(sol('Billard'));
grid on;

figure(3)
plot(sol('Billard').x(:,1), sol('Billard').x(:,2));
grid on;
hold on;
plot(sol('Observer').x(:,1), sol('Observer').x(:,3));
hold on;
theta = pi/2;
x_0 = cos(2*pi/3);
y_0 = sin(2*pi/3);
plot([x_0 - cos(theta); x_0 ], [y_0 - sin(theta); y_0 ], "Color", 'r');
x_plus = sys_billard.jumpMap([x_0; y_0; theta]);
theta_plus = x_plus(3);
hold on;
plot([x_0; x_0 + cos(theta_plus)], [y_0; y_0 + sin(theta_plus)],  "Color", 'b');
hold on;
theta = linspace(-pi, pi);
plot(cos(theta), sin(theta));
hold on;
plot([x_0, 0], [y_0, 0], "LineStyle", '--')

figure(4)
plot(sol('Billard').t, sol('Billard').x(:,1).^2+ sol('Billard').x(:,2).^2, 'o-', 'LineWidth', 2);
disp(sys_billard.normal_angle([1,1]))

figure(6)
plot(sol('Observer').t, sol('Observer').x(:,5))
hold on;
plot(sol('Billard').t, sol('Billard').x(:,3))

figure(7)
plot(sol('Observer').t, sol('Observer').x(:,3))
hold on;
plot(sol('Billard').t, sol('Billard').x(:,2))

figure(9)
subplot(3,1,1);
plot(sol('Observer').t, sol('Observer').x(:,3) - sol('Billard').x(:,2));
subplot(3, 1, 2);
plot(sol('Observer').t, sol('Observer').x(:,1) - sol('Billard').x(:,1));
subplot(3, 1, 3);
plot(sol('Observer').t, sol('Observer').x(:,5) - sol('Billard').x(:,3));

figure(8)
plot(sol('Observer').t, sol('Observer').x(:,1))
hold on;
plot(sol('Billard').t, sol('Billard').x(:,1))

figure(5)
% Define a 3D grid
[x, y, z] = meshgrid(-1:0.5:1, -1:0.5:1, -pi:0.5:pi);

% Define a 3D vector field (example: F = [-y, x, z])
u = cos(z);  % x-component
v = sin(z);   % y-component
w = 0*sin(z);   % z-component

% Plot the vector field
quiver3(x, y, z, u, v, w, 'AutoScale', 'on', 'LineWidth', 1.5, 'Color', 'b');
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Flow Vectors (Vector Field: F = [-y, x, z])');
grid on;
axis equal;
hold on;

% Add streamlines for better visualization (optional)
start_points = [0, 0, -2; 0, 0, -1; 0, 0, 0; 0, 0, 1; 0, 0, 2];
streamline(x, y, z, u, v, w, start_points(:,1), start_points(:,2), start_points(:,3), 'LineWidth', 2, 'Color', 'r');
hold on;
theta = linspace(-pi, pi);
plot3(cos(theta), sin(theta), theta+pi/2)