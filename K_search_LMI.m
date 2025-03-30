sys_ball = BouncingBallSubSystemClass();

sys_obs = BouncingBallObserver();
sys_obs.L_c = 4.85e-1*[1.4; 1.25];
sys_obs.L_c = [0.8; 0.6];
sys_obs.L_d = 1*[0.1; 0.1];

sys_ball.mu = 0; % Additional velocity at each impact

sys_ball.lambda = 1;
sys_ball.f_air = 0;

sys_obs.mu = sys_ball.mu;
sys_obs.lambda = sys_ball.lambda;
sys_obs.f_air = sys_ball.f_air;



% BEWARE : only true for no air friction
F = [0, 1; 0, 0];
J = [1, 0; 0, -sys_ball.lambda];

H = [1, 0];
w = [1; 0];
tau = 15.6718 - 13.6045;
x = [0; -10.0995];

setlmis([])
P = lmivar(1, [2 1]);
MC_before = J - (J*sys_ball.flowMap(x, 0, 0, 0) - sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) )/x(2)*w';
MC_after = MC_before;
PLd = lmivar(2, [2 1]);

MLd_before = -H;
MLd_after = MLd_before - H*(sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) - sys_ball.flowMap(x, 0, 0, 0))/x(2)*w';

flow = expm((F - sys_obs.L_c*H)*tau);

M_after = newlmi;
lmiterm([-M_after 1 2 P], 1, MC_after*flow); 
lmiterm([-M_after 1 2 PLd], 1, MLd_after*flow); 
lmiterm([-M_after 1 1 P], 1, 1); 
lmiterm([-M_after 2 2 P], 1, 1); 

M_before = newlmi;
lmiterm([-M_before 1 2 P], 1, MC_before*flow); 
lmiterm([-M_before 1 2 PLd], 1, MLd_before*flow); 
lmiterm([-M_before 2 2 P], 1, 1); 
lmiterm([-M_before 1 1 P], 1, 1); 

P_lmi = newlmi;
lmiterm([-P_lmi 1 1 P], 1, 1);

LMI_full = getlmis;
[t_min, x] = feasp(LMI_full);
t_min
P = dec2mat(LMI_full, x, P)
PLd = dec2mat(LMI_full, x, PLd);
Ld = linsolve(P, PLd)