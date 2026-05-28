addpath('./utils');

sys_aslip = ASLIP_Hybrid();
sys_aslip.k_h = 400;
sys_aslip.k = 1000;
sys_aslip.theta_0 = -pi/8;

% Define solver's parameter
max_dt_step = 0.1;
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', max_dt_step);
sys = CompositeHybridSystem('ASLIP', sys_aslip);

% X_0 is first element of cell
qt = sys_aslip.T_bt([0.0; 1.8; 1.3]);
x0_cell = {[0.; 1.8; 1.3; 0; -1; 0; qt(1); qt(2); 0]};
tspan = [0, 2900];
jspan = [0, 180];

%% Solve coupled system 
sol = sys.solve(x0_cell, tspan, jspan, config);

figure(1)
hpb = HybridPlotBuilder().subplots('on')...
    .legend('$x_b$', '$y_b$', '$\theta_b$')...
    .plotFlows(sol('ASLIP').select(1:3));
grid on;

figure(2)
hpb = HybridPlotBuilder().subplots('on')...
    .legend('$\dot{x}_b$', '$\dot{y}_b$', '$\dot{\theta}_b$')...
    .plotFlows(sol('ASLIP').select(4:6));
grid on;

figure(3)
hpb = HybridPlotBuilder().subplots('on')...
    .legend('$x_t$', '$y_t$', '$q$')...
    .plotFlows(sol('ASLIP').select(7:9));
grid on;

ql = zeros(length(sol('ASLIP').t), 3);
for i=1:length(sol('ASLIP').t)
    qb = sol('ASLIP').x(i, 1:3)';
    qt = sol('ASLIP').x(i, 7:8)';
    ql(i,:) = sys_aslip.T_bl(qb, qt)';
end

E = zeros(length(sol('ASLIP').t), 1);
lagr = zeros(length(sol('ASLIP').t), 1);
for i=1:length(sol('ASLIP').t)
    qb = sol('ASLIP').x(i, 1:3)';
    qt = sol('ASLIP').x(i, 7:8)';
    q_dot = sol('ASLIP').x(i, 4:6)';
    ql_i = sys_aslip.T_bl(qb, qt);
    E(i) = sys_aslip.m_b * sol('ASLIP').x(i, 5)^2/2 + sys_aslip.m_b*sys_aslip.a_g*sol('ASLIP').x(i, 2);
    lagr(i) = sys_aslip.lagrangian(sol('ASLIP').x(i, :));
end

figure(4)
clf;
plot(sol('ASLIP').t, ql(:,1))
hold on;
plot(sol('ASLIP').t, ql(:,2));
hold on;
plot(sol('ASLIP').t, ql(:,3));
hold on;
plot(sol('ASLIP').t, lagr);
legend('$\theta_t$', '$\theta_h$', '$l_l$', 'lagrangian', 'Interpreter', 'latex');
grid on;

q_trajectory = sol('ASLIP').x;
l_b = sys_aslip.l_b;
l_l0 = sys_aslip.l_l0;
dt = - sol('ASLIP').t(1:end-1) + sol('ASLIP').t(2:end);
save('../wip_ ASLIP/aslip_trajectory.mat', 'q_trajectory', 'l_b', 'l_l0', 'dt');