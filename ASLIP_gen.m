addpath('./utils');

sys_aslip = ASLIP_Hybrid();

% Define solver's parameter
max_dt_step = 0.1;
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', max_dt_step);
sys = CompositeHybridSystem('ASLIP', sys_aslip);

% X_0 is first element of cell
qt = sys_aslip.T_bt([0.; 1.5; 1.3]);
x0_cell = {[0.; 2.5; pi/2; 0; -1; 0; qt(1); qt(2); 0]};
tspan = [0, 7];
jspan = [0, 2450];

%% Solve coupled system 
sol = sys.solve(x0_cell, tspan, jspan, config);

figure(1)
hpb = HybridPlotBuilder().subplots('on')...
    .plotFlows(sol('ASLIP').select(1:3));

figure(2)
hpb = HybridPlotBuilder().subplots('on')...
    .plotFlows(sol('ASLIP').select(7:9));

grid on;