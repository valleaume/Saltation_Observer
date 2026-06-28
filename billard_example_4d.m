addpath('./utils');
close;

sys_billard = Billard_sys_4d();
sys_copy = Billard_obs_4d();

sys_copy.L_c = [1; 1.41; 0; 0];
sys_copy.l = 1;
sys_copy.L_d_all = zeros(4);

% Define solver's parameter
max_dt_step = 0.1;
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', max_dt_step);
sys = CompositeHybridSystem('Billard', sys_billard, 'Observer', sys_copy);

% map plant outputs to observer input (use first output/component)
obs_input = @(y_ball, ~) y_ball(1);
sys.setInput('Observer', obs_input);

% initial conditions: [x1; x1dot; x2; x2dot]
x0_plant = [0.5; 1/sqrt(2); 0.5; -1/sqrt(2)];
%x0_plant = [1; 0.99 ; 0; -0.0298];
x0_obs = [0.4997; 1.0003; 0.5007; -1.0002];
%x0_obs = sys_copy.init_cond(x0_plant);
x0_cell = {x0_plant, 0.99999*x0_plant};

tspan = [0, 79];
jspan = [0, 180];

%% Solve coupled system
sol = sys.solve(x0_cell, tspan, jspan, config);

close all;

% Positions over time
figure(1);
plot(sol('Billard').t, sol('Billard').x(:,1), '-','LineWidth',1.5); hold on;
plot(sol('Observer').t, sol('Observer').x(:,1),'--','LineWidth',1.5);
plot(sol('Billard').t, sol('Billard').x(:,3), '-.','LineWidth',1.2);
plot(sol('Observer').t, sol('Observer').x(:,3), ':','LineWidth',1.2);
legend('x1 (plant)','x1 (obs)','x2 (plant)','x2 (obs)');
grid on;

% Phase plot (x1 vs x2)
figure(2);
plot(sol('Billard').x(:,1), sol('Billard').x(:,3), '-','LineWidth',1.5); hold on;
plot(sol('Observer').x(:,1), sol('Observer').x(:,3),'--','LineWidth',1.2);
legend('plant','observer'); axis equal; grid on;

% Norm squared (distance from origin)
figure(3);
plot(sol('Billard').t, sol('Billard').x(:,1).^2 + sol('Billard').x(:,3).^2,'-o'); hold on;
plot(sol('Observer').t, sol('Observer').x(:,1).^2 + sol('Observer').x(:,3).^2,'-x');
legend('plant','observer'); grid on;

% Velocities comparison
figure(4);
subplot(2,1,1);
plot(sol('Billard').t, sol('Billard').x(:,2),'-'); hold on;
plot(sol('Observer').t, sol('Observer').x(:,2),'--');
legend('x1dot (plant)','x1dot (obs)'); grid on;
subplot(2,1,2);
plot(sol('Billard').t, sol('Billard').x(:,4),'-'); hold on;
plot(sol('Observer').t, sol('Observer').x(:,4),'--');
legend('x2dot (plant)','x2dot (obs)'); grid on;


% Errors
figure(5);
subplot(4,1,1);
plot(sol('Billard').t, sol('Billard').x(:,2) - sol('Observer').x(:,2),'-'); hold on;
legend('x1dot (plant) - x1dot (obs)'); grid on;
subplot(4,1,2);
plot(sol('Billard').t, sol('Billard').x(:,4) - sol('Observer').x(:,4),'-'); hold on;
legend('x2dot (plant) - x2dot (obs)'); grid on;
subplot(4,1,3);
plot(sol('Billard').t, sol('Billard').x(:,3) - sol('Observer').x(:,3),'-'); hold on;
legend('x2 (plant) - x2 (obs)'); grid on;

subplot(4,1,4);
plot(sol('Billard').t, sol('Billard').x(:,1) - sol('Observer').x(:,1),'-'); hold on;
legend('x1 - x1 (obs)'); grid on;