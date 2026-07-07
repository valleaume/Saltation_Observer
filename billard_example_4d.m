addpath('./utils');
close;

sys_billard = Billard_sys_4d();
sys_copy = Billard_obs_4d();
sys_copy.L_c = [1; 1.41; 0; 0];
sys_copy.l = 1;
Ld = [2.97422105e+01  2.97187971e-02  2.97422105e+01  2.97187971e-02;
      -7.07050177e-01  2.65473941e-02 -7.07050177e-01  2.65473941e-02;
      -1.15844305e+01 -6.96781337e-02 -1.15844305e+01 -6.96781337e-02;
      -8.85491013e-01  3.44760465e-02 -8.85491013e-01  3.44760465e-02];
Ld0 = [ 1.00571217  0.22962766  0.36933747 -0.00907567];
Ld1 = [ 0.0297188   0.02654739 -0.06967813  0.03447605];
Ld2 = [ 1.00571217  0.22962766  0.36933747 -0.00907567];
Ld3 = [ 0.0297188   0.02654739 -0.06967813  0.03447605];

Ld = [Ld0', Ld1', Ld2', Ld3'];
disp(Ld)
Fc = [5;
      35.25;
      0;
      0];
sys_copy.L_c = Fc;
sys_copy.L_d_all = Ld;

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
x0_obs = [0.647; 1*2/sqrt(2); 0.53; -1.21/sqrt(2)];
%x0_obs = sys_copy.init_cond(x0_plant);
x0_cell = {x0_plant, x0_obs};

tspan = [0, 527.9];
jspan = [2, 2180];

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
far_jump_mask = sol('Observer').j - sol('Billard').j == 0;
subplot(4,1,1);
plot(sol('Billard').t(far_jump_mask), sol('Billard').x(far_jump_mask,2) - sol('Observer').x(far_jump_mask,2),'-'); hold on;
legend('x1dot (plant) - x1dot (obs)'); grid on;
subplot(4,1,2);
plot(sol('Billard').t(far_jump_mask), sol('Billard').x(far_jump_mask,4) - sol('Observer').x(far_jump_mask,4),'-'); hold on;
legend('x2dot (plant) - x2dot (obs)'); grid on;
subplot(4,1,3);
plot(sol('Billard').t, sol('Billard').x(:,3) - sol('Observer').x(:,3),'-'); hold on;
legend('x2 (plant) - x2 (obs)'); grid on;

subplot(4,1,4);
plot(sol('Billard').t, sol('Billard').x(:,1) - sol('Observer').x(:,1),'-'); hold on;
legend('x1 - x1 (obs)'); grid on;

P1 = [15670.37129803, 31958.78684083, 11512.78474076, 5420.39682666;
      31958.78684083, 1074834.54201758, 67032.37074678, -701911.94725101;
      11512.78474076, 67032.37074678, 25344.67883937, -24444.76822552;
      5420.39682666, -701911.94725101, -24444.76822552, 592497.34751836];

P2 = [300884.56762379, 12382.83103850, 10171.49423448, 429409.52817528;
      12382.83103850, 63927.85897297, 22562.35789744, -11194.67507341;
      10171.49423448, 22562.35789744, 20846.33141367, 5233.70854140;
      429409.52817528, -11194.67507341, 5233.70854140, 1674322.52612149];

P3 = [15670.37129800, 31958.78684080, 11512.78474075, 5420.39682660;
      31958.78684080, 1074834.54201630, 67032.37074675, -701911.94725024;
      11512.78474075, 67032.37074675, 25344.67883934, -24444.76822554;
      5420.39682660, -701911.94725024, -24444.76822554, 592497.34751794];

P4 = [300884.56762412, 12382.83103851, 10171.49423449, 429409.52817598;
      12382.83103851, 63927.85897276, 22562.35789742, -11194.67507323;
      10171.49423449, 22562.35789742, 20846.33141366, 5233.70854142;
      429409.52817598, -11194.67507323, 5233.70854142, 1674322.52612318];

P5 = [ 124407.19766558,   166949.8890977 ,    82824.95200479, 73160.59704322;
       166949.8890977 ,  7747467.140421  ,   349072.44379371, -5171351.65932725;
       82824.95200479,   349072.44379371,   175790.65663321, -90951.28479053;
       73160.59704322, -5171351.65932725,   -90951.28479053, 4597145.26385606];

figure(10)
e = sol('Observer').x - sol('Billard').x;
far_jump_mask = (abs(e(:,2)) + abs(e(:,4)) < 1)';
far_jump_mask = sol('Observer').j - sol('Billard').j == 0;
e_peakless = e(far_jump_mask,:);

function V = Lyap_piecewise(P_list, e, j)
    assert(numel(P_list) == 4, 'P_list must contain 4 matrices for the 4 jump modes.');
    V_1 = diag(e*P_list{1}*e');
    V_2 = diag(e*P_list{2}*e');
    V_3 = diag(e*P_list{3}*e');
    V_4 = diag(e*P_list{4}*e');
    jump = mod(j, 4)+1;
    V = nan(size(jump));
    V(jump==1) = V_1(jump==1);
    V(jump==2) = V_2(jump==2);
    V(jump==3) = V_3(jump==3);
    V(jump==4) = V_4(jump==4);
end

function V = Lyap_discrete(P_list, e, j, t, A)
    assert(length(t) == length(j), 't and j must have the same size.');
    tau = mod(t, sqrt(2));
    e_discrete = zeros(size(e));

    for i = 1:length(t)
        e_discrete(i,:) = e(i,:) * exp(-A * tau(i))';
    end
    V_1 = diag(e_discrete*P_list{1}*e_discrete');
    V_2 = diag(e_discrete*P_list{2}*e_discrete');
    V_3 = diag(e_discrete*P_list{3}*e_discrete');
    V_4 = diag(e_discrete*P_list{4}*e_discrete');
    jump = mod(j, 4)+1;
    
    V = nan(size(jump));
    V(jump==1) = V_1(jump==1);
    V(jump==2) = V_2(jump==2);
    V(jump==3) = V_3(jump==3);
    V(jump==4) = V_4(jump==4);
end

%norm_p_error = (e*P*e');
A = sys_billard.A_c - sys_copy.L_c*sys_copy.C;

norm_p_error = Lyap_discrete({P1, P2, P3, P4}, e_peakless, sol('Observer').j(far_jump_mask), sol('Observer').t(far_jump_mask), A);
plot(sol('Billard').t(far_jump_mask), norm_p_error);