yalmip('clear');

%%% Script searching for a stable L_d and its corresponding P given L_c

%It is done using LMIs solving the matrix inequality of section IV 

%% Define the plant subsystem
sys_plant = CustomStick();

sys_plant.w = sqrt(10);        % frequency
sys_plant.v_t = 0.5;           % treadmill speed
sys_plant.mu_d = 0.5;

% Define the observer subsystem
sys_obs = CustomStickObserver();

% Its dynamic is a copy of the plant's dynamic
sys_obs.w = sys_plant.w;
sys_obs.v_t = sys_plant.v_t;
sys_obs.mu_d = sys_plant.mu_d;

% Choose the observer gains
sys_obs.L_c = [10; 100; 0; 0; 10; 100];   % Flow gain
sys_obs.L_d = 1*[0.; 0.; 0; 0; 0; 0]; % Jump gain
sys_obs.K = [0, 0, 0, 0, 0, 0];         % Gain on jump detection


F_stick = zeros(4);
F_stick(1,2) = 1;

F_slip = F_stick;
F_slip(2,1) = -sys_plant.w^2;



J = eye(6);
J(4,4) = -1;

proj = [eye(4); zeros(2,4)];


H = [1, 0, 0, 0];
w_slip = [0; -1; 0; 0; 0; 0];

w_stick = [-sys_plant.w^2; 0; sys_plant.g; 0; 0; 0];
tau_stick = 14.6127 - 13.8302;
x_stick = [0.686; 0.5; 0.72; 0; 0; 0];


tau_slip = 16.037- 14.6127;
x_slip = [0.294608; 0.5; 0.72; -1; -0.294608; -0.5];
MC_before_stick = proj'*(J  - (J*sys_plant.flowMap(x_stick, 0, 0, 0) - sys_plant.flowMap(sys_plant.jumpMap(x_stick, 0, 0, 0), 0, 0, 0))/(w_stick'*sys_plant.flowMap(x_stick, 0, 0, 0))*w_stick')*proj;

MC_before_slip = proj'*(J - (J*sys_plant.flowMap(x_slip, 0, 0, 0) - sys_plant.flowMap(sys_plant.jumpMap(x_slip, 0, 0, 0), 0, 0, 0))/(w_slip'*sys_plant.flowMap(x_slip, 0, 0, 0))*w_slip')*proj;

MLd_before = -H;


Lc_stick = [10, 100 ,0, 0]';
Lc_slip = [10, 100, 0, 0]',
%%
P = sdpvar(4,4);
PLd_stick = sdpvar(4,1);
PLd_slip = sdpvar(4,1);
flow_stick = expm((F_stick-Lc_stick*H)*tau_stick);
flow_slip = expm((F_slip-Lc_slip*H)*tau_slip);

Constraints = [P>=0.01*eye(4)];
Constraints = [Constraints, [P (P*MC_before_slip+PLd_slip*MLd_before)*flow_slip; ((P*MC_before_slip+PLd_slip*MLd_before)*flow_slip)' P] >= 0*0.000001*eye(8)];
Constraints = [Constraints, [P (P*MC_before_stick+PLd_stick*MLd_before)*flow_stick; ((P*MC_before_stick+PLd_stick*MLd_before)*flow_stick)' P] >= 0*0.00001*eye(8)];

options = sdpsettings('verbose',1);

sol_int = optimize(Constraints,0, options);

P = value(P)
eig(P)
Ld_slip = linsolve(P, value(PLd_slip))
Ld_stick = linsolve(P, value(PLd_stick))

eig(((MC_before_stick+Ld_stick*MLd_before)*flow_stick)'*P*((MC_before_stick+Ld_stick*MLd_before)*flow_stick))
%%

%{

% Define variables
P = sdpvar(4,4);
Ad_slip = sdpvar(1,1);
Ad_stick = sdpvar(1,1);
Ac_stick = sdpvar(1);
Ac_slip = sdpvar(1);
PLc_stick = sdpvar(4,1);
PLc_slip = sdpvar(4,1);
PLd_stick = sdpvar(4,1);
PLd_slip = sdpvar(4,1);
Ld_stick = sdpvar(4,1);
Ld_slip = sdpvar(4,1);

warm_start = true;

% Define constraints 
Constraints = [P>=0.01*eye(4)];
Constraints = [Constraints, F_stick'*P + P*F_stick - (PLc_stick*H)' - PLc_stick*H <= Ac_stick*P ];
Constraints = [Constraints, F_slip'*P + P*F_slip - (PLc_slip*H)' - PLc_slip*H <= Ac_slip*P];
Constraints = [Constraints, MC_before_slip'*P*MC_before_slip + MC_before_slip'*PLd_slip*MLd_before + (PLd_slip*MLd_before)'*MC_before_slip + (Ld_slip*MLd_before)'*PLd_slip*MLd_before<= Ad_slip*P]; %+ (Ld_slip*MLd_before)'*PLd_slip*MLd_before
Constraints = [Constraints, MC_before_stick'*P*MC_before_stick + MC_before_stick'*PLd_stick*MLd_before + (PLd_stick*MLd_before)'*MC_before_stick  + (Ld_stick*MLd_before)'*PLd_stick*MLd_before  <= Ad_stick*P]; % + (Ld_stick*MLd_before)'*PLd_stick*MLd_before 
Constraints = [Constraints, PLd_stick == P*Ld_stick];
Constraints = [Constraints, PLd_slip == P*Ld_slip];

Objective =  -(tau_slip*Ac_slip+tau_stick*Ac_stick + log(Ad_slip) + log(Ad_stick));
Constraints = [Constraints, Objective>=1]
% Set some options for YALMIP and solver

%options = optimoptions(options,"EnableFeasibilityMode",true,"SubproblemAlgorithm","cg")
options = sdpsettings('verbose',1);
options.fmincon.EnableFeasibilityMode = true;
options = sdpsettings(options, 'solver','fmincon');

% Define an objective
if warm_start

    sol_int = optimize(Constraints,0, options);
end
Objective =  -(tau_slip*Ac_slip+tau_stick*Ac_stick + log(Ad_slip) + log(Ad_stick));
value(P)
Objective
%"fmincon.EnableFeasibilityMode",true);
% Solve the problem
sol = optimize(Constraints,Objective,options);

% Analyze error flags
if sol.problem == 0
 % Extract and display value
    disp('yeah')
else
 display('Hmm, something went wrong!');
 sol.info
 yalmiperror(sol.problem)
end

P = value(P)
Lc_slip = linsolve(P, value(PLc_slip))
Lc_stick = linsolve(P, value(PLc_stick))
Ld_slip = linsolve(P, value(PLd_slip))
Ld_stick = linsolve(P, value(PLd_stick))
Objective
%}