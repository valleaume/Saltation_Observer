addpath('./utils');
close;

sys_billard = Billard_sys();
sys_copy = Billard_linear_obs();
P = [ 804.45482198, -445.84084471,  146.59587562; -445.84084471,  441.69627454, -144.6045289 ; 146.59587562, -144.6045289,   144.60452151]

sys_copy.L_c = [-1.96960149e-03; 8.71696722e-01; 2.60701729e+00];  %[3.20832993e+05; 1.58061299e-03; -1.23868244e-02];
Ld = [-0.65712493; 1.35260778; 4.01878194];
sys_copy.L_d_square = [Ld, Ld, Ld, Ld];


% L_c = [6.66784254e+04; 4.48499443e-04; 7.53808882e-04];
% sys_copy.L_d_square = [-1.33557158e+00,  1.40459147, -1.34343837e+00,  1.40459147;
%                         -1.33734220e-09,  0.00437211,  6.18360175e-09, -0.00437210;
%                         2.34965987e-07, -3.02608526, -1.20290005e-07,  3.02608526
%                     ];

% Define solver's parameter
max_dt_step = 0.01;
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-5, 'MaxStep', max_dt_step);
sys = CompositeHybridSystem('Billard', sys_billard, 'Observer', sys_copy);
obs_input = @(y_ball, ~) y_ball;
sys.setInput('Observer', obs_input);

% X_0 is first element of cell

x0_cell = {[0.5; 0.5; -pi/4], sys_copy.init_cond([0.500028; 0.50001; -pi/4])};
tspan = [0, 2];
jspan = [0, 180];

%% Solve coupled system 
sol = sys.solve(x0_cell, tspan, jspan, config);

close all;

figure(1)
hpb = HybridPlotBuilder().subplots('on')...
    .legend('$x_b$', '$y_b$', '$\theta_b$')...
    .plotFlows(sol('Billard').select(1:3));
grid on;
title('Billard plant states');
xlabel('Time');
ylabel('State');

figure(2)
hpb = HybridPlotBuilder().subplots('on')...
    .plotPhase(sol('Billard'));
grid on;
title('Billard phase portrait');
xlabel('x_1');
ylabel('x_2');

figure(3)
plot(sol('Billard').x(:,1), sol('Billard').x(:,2), 'LineWidth', 1.5);
grid on;
hold on;
plot(sol('Observer').x(:,1), sol('Observer').x(:,2), 'LineWidth', 1.5);
hold on;
theta = pi/2;
x_0 = cos(2*pi/3);
y_0 = sin(2*pi/3);
plot([x_0 - cos(theta); x_0 ], [y_0 - sin(theta); y_0 ], "Color", 'r', 'LineWidth', 1.5);
x_plus = sys_billard.jumpMap([x_0; y_0; theta]);
theta_plus = x_plus(3);
hold on;
plot([x_0; x_0 + cos(theta_plus)], [y_0; y_0 + sin(theta_plus)],  "Color", 'b', 'LineWidth', 1.5);
hold on;
theta = linspace(-pi, pi);
plot(cos(theta), sin(theta), 'LineWidth', 1);
hold on;
plot([x_0, 0], [y_0, 0], "LineStyle", '--', 'LineWidth', 1);
legend('Billard plant', 'Observer', 'Normal direction', 'Post-jump direction', 'Guard circle', 'Reference line', 'Location', 'best');
title('Trajectory comparison in the plane');
xlabel('x_1');
ylabel('x_2');
axis equal;

figure(4)
plot(sol('Billard').t, sol('Billard').x(:,1).^2+ sol('Billard').x(:,2).^2, 'o-', 'LineWidth', 2);
legend('r^2 = x_1^2 + x_2^2', 'Location', 'best');
title('Distance to the origin');
xlabel('Time');
ylabel('x_1^2 + x_2^2');

figure(6)
plot(sol('Observer').t, sol('Observer').x(:,3), 'LineWidth', 1.5);
hold on;
plot(sol('Billard').t, sol('Billard').x(:,3), 'LineWidth', 1.5);
legend('Observer \theta', 'Billard \theta', 'Location', 'best');
title('Angular state comparison');
xlabel('Time');
ylabel('\theta');

figure(7)
plot(sol('Observer').t, sol('Observer').x(:,2), 'LineWidth', 1.5);
hold on;
plot(sol('Billard').t, sol('Billard').x(:,2), 'LineWidth', 1.5);
legend('Observer x_2', 'Billard x_2', 'Location', 'best');
title('Velocity component comparison');
xlabel('Time');
ylabel('x_2');

figure(9)
subplot(3,1,1);
plot(sol('Observer').t, abs(sol('Observer').x(:,1) - sol('Billard').x(:,1)), 'LineWidth', 1.5);
legend('$||x_1 - \hat{x}_1||$', 'Location', 'best', 'interpreter', 'latex', 'FontSize', 14);
title('Observation errors');
xlabel('Time');
ylabel('e_{x_1}');
subplot(3, 1, 2);
plot(sol('Observer').t, abs(sol('Observer').x(:,2) - sol('Billard').x(:,2)), 'LineWidth', 1.5);
legend('$||x_2 - \hat{x}_2||$', 'Location', 'best', 'interpreter', 'latex', 'FontSize', 14);
xlabel('Time');
ylabel('e_{x_2}');
subplot(3, 1, 3);
plot(sol('Observer').t, abs(sol('Observer').x(:,3) - sol('Billard').x(:,3)), 'LineWidth', 1.5);
legend('$||\theta - \hat{\theta}||$', 'Location', 'best', 'interpreter', 'latex', 'FontSize', 14);
xlabel('Time');
ylabel('e_{\theta}');

figure(8)
plot(sol('Observer').t, sol('Observer').x(:,1), 'LineWidth', 1.5);
hold on;
plot(sol('Billard').t, sol('Billard').x(:,1), 'LineWidth', 1.5);
legend('Observer x_1', 'Billard x_1', 'Location', 'best');
title('Position comparison');
xlabel('Time');
ylabel('x_1');

figure(10)
e = sol('Observer').x - sol('Billard').x;
norm_p_error = (e*P*e');
plot(sol('Billard').t, diag(norm_p_error));



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