addpath('utils');
close all;

% Define the plant subsystem
sys_ball = BouncingBallSubSystemClass();

sys_ball.mu = 0;        % Additional velocity at each impact
sys_ball.lambda = 1;    % Restitution coefficient
sys_ball.f_air = 0;     % Friction 

% Define the observer subsystem
sys_obs = BouncingBallKallmanObserver();

% Its dynamic is a copy of the plant's dynamic
sys_obs.mu = sys_ball.mu;
sys_obs.lambda = sys_ball.lambda;
sys_obs.f_air = sys_ball.f_air;

% Choose the observer gains
sys_obs.gain = 0.23;
sys_obs.lambda_kallman = 0.4;
sys_obs.gamma_kallman = 1;

sys_obs.L_c = [0.1; 0.25];   % Flow gain
sys_obs.L_d = 1*[0.1; 0.1]; % Jump gain
sys_obs.K = [0, 0];         % Gain on jump detection
%BEWARE: K(1) < 0.5 is necessary to enforce transversality


% Define the coupled observerver-plant system 
sys = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs);
obs_input = @(y_ball, ~) y_ball;
sys.setInput('Observer', obs_input);

sys

% Define solver's parameter
max_dt_step = 0.1;
config = HybridSolverConfig('AbsTol', 1e-3, 'RelTol', 1e-7, 'MaxStep', max_dt_step);

% X_0 is first element of cell, hat{X_0} is the second
x0_cell = {[1; 2]; (1 - 0.6e0)*[1; -2; reshape(eye(2), [4,1])]};
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
    .plotFlows(sol('Observer').select(1:2))
 
% Plot Phase
figure(2)
hpb = HybridPlotBuilder().subplots('on')...
    .legend('$x$')...
    .plotPhase(sol('Ball'));
grid on;    
hold on;
hpb.subplots('on')...
    .flowColor('#FF8800')...
    .jumpColor('m')...
    .jumpEndMarker('o')...
    .legend('$\hat{x}$')...
    .plotPhase(sol('Observer').select(1:2))
hold on;
% Vector field
x_ = -2:0.5:10; % Coordonnées en x
y_ = -15:1:15; % Coordonnées en y

[X, Y] = meshgrid(x_, y_); % Création de la grille
U =   Y ; %+ 3*(0.0-X); % Composante x du vecteur, can put a K
V =  -sys_ball.gamma - sign(Y) .* sys_ball.f_air.*Y.^2;  % Composante y du vecteur

% Tracé du champ de vecteurs
quiver(X, Y, U, V);
xlim([-2,10]);
ylim([-15,15]);
title('Phase Space', 'Interpreter', 'latex')

% Plot position error
figure(3)
plot(sol('Ball').t, sol('Ball').x(:,1) - sol('Observer').x(:,1));
grid on;
xlabel('$t$', Interpreter= 'latex');
ylabel('$x_1- \hat{x}_1$', Interpreter= 'latex');
title( "Position error");

% Plot velocity error
figure(4)
plot(sol('Ball').t, sol('Ball').x(:,2) - sol('Observer').x(:,2));
grid on;
xlabel('$t$', Interpreter= 'latex');
ylabel('$x_2- \hat{x}_2$', Interpreter= 'latex');
title( "Velocity error");

% Pot norm of error
figure(5)
e = sol('Ball').x - sol('Observer').x(:,1:2);
P = eye(2);  %Here you can put a P calculated in K_search_LMI 
far_jump_mask = ~ismember(sol('Ball').t, sol.jump_times)';
% Trying to get rid of spikes
% Arbitrary treshold on velocity error to get rid of them
far_jump_mask = (abs(e(:,2))<10)';
e_after = e(sign_jump==1 & far_jump_mask,:);
e_before = e(sign_jump==-1 & far_jump_mask,:);  

plot(sol('Ball').t(sign_jump==-1 & far_jump_mask), diag(e_before*P*e_before'), color='green'); % When jumping before
hold on;
grid on;
plot(sol('Ball').t(sign_jump==1 & far_jump_mask), diag(e_after*P*e_after'), color='red'); % When jumping after
legend('Observer jumps before', 'Observer jumps after');
xlabel('$t$', 'Interpreter', 'Latex')
ylabel('$\|x-\hat{x}\|^2$',  'Interpreter', 'Latex')
title("Norm error");

% Plot norm of error
figure(6)
e = sol('Ball').x - sol('Observer').x(:,1:2);

far_jump_mask = ~ismember(sol('Ball').t, sol.jump_times)';
disp(size(far_jump_mask))
% Trying to get rid of spikes
% Arbitrary treshold on velocity error to get rid of them
far_jump_mask = (abs(e(:,2))<10)';


e_shaped = reshape(e, 2, 1, []);


P_t =  reshape( sol('Observer').x(:,3:6)', 2, 2, []);
P_inv_t = 1/(P_t(1,1,:).*P_t(2,2,:) -  P_t(1,2,:).*P_t(2,1,:)).*[P_t(2,2,:),-P_t(2,1,:) ; -P_t(1,2,:), P_t(1,1,:) ];

P_t = P_inv_t;

disp(P_t(:,:,1))
disp(size(pagemtimes(pagemtimes(permute(e_shaped, [2,1,3]), P_t), e_shaped)))
e_normed = reshape(pagemtimes(pagemtimes(permute(e_shaped, [2,1,3]), P_t), e_shaped), [], 1);

e_normed_no_jump = e_normed(~far_jump_mask,:);

hold on;
grid on;

scatter(sol('Ball').t(~far_jump_mask), e_normed_no_jump, color='red'); 

plot(sol('Ball').t, reshape(P_t(1,1,:).^2 + P_t(2,2,:).^2 + P_t(2,1,:).^2+ P_t(1,2,:).^2, [],1)) % l2 norm
legend('Observer jumps before', 'Observer jumps after', 'L_2 norm of \Pi');
xlabel('$t$', 'Interpreter', 'Latex')
ylabel('$\|x-\hat{x}\|^2$',  'Interpreter', 'Latex')
%plot(sol('Ball').t, reshape(P_t(1,1,:).^2 + P_t(2,2,:).^2 + P_t(2,1,:).^2+ P_t(1,2,:).^2, [],1))
%plot(sol('Ball').t, reshape(P_t(1,1,:).*P_t(2,2,:) -  P_t(1,2,:).*P_t(2,1,:), [],1)) det
%plot(sol('Ball').t, reshape( P_t(1,2,:)- P_t(2,1,:), [],1))  symmetric?
title("Lyapunov function $(x-\hat{x})^t\Pi(x-\hat{x})$ over time", 'Interpreter', 'Latex');

% Plot who jump first using j
figure(7)
plot(sol('Ball').t, sol('Ball').j - sol("Observer").j);

%% Compute M_before, M_after

% BEWARE : only true for no air friction
F = [0, 1; 0, 0];
J = [1, 0; 0, -sys_ball.lambda];

H = [1, 0];
w = [1; 0];
tau = 15.6718 - 13.6045;
x = [0; -10.0995];
M_before = J - sys_obs.L_d*H - (J*sys_ball.flowMap(x, 0, 0, 0) - sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) )/x(2)*w';
M_after = M_before - sys_obs.L_d*H*(sys_ball.flowMap(sys_ball.jumpMap(x, 0, 0, 0), 0, 0, 0) - sys_ball.flowMap(x, 0, 0, 0))/x(2)*w';

% Print eigenvalues to theoretically verify stability/unstability
abs(eig(M_before*expm((F-sys_obs.L_c*H)*tau)))
abs(eig(M_after*expm((F-sys_obs.L_c*H)*tau)))


