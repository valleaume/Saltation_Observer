function [sys, config, sys_ball, sys_obs, sys_obs_ref] = observersCovarianceConfig()
% OBSERVERSCOVARIANCECONFIG - Configuration and system setup for observer covariance analysis
%
% This function initializes all hybrid subsystems and solver configuration needed for
% the bouncing ball observer analysis. It ensures consistent setup across data generation
% and plotting scripts.
%
% Usage:
%   [sys, config, sys_ball, sys_obs, sys_obs_ref] = observersCovarianceConfig()
%
% Outputs:
%   sys          - CompositeHybridSystem combining ball, observer, and Kalman observer
%   config       - HybridSolverConfig with numerical tolerances and step size
%   sys_ball     - BouncingBallSubSystemClass (plant system)
%   sys_obs      - BouncingBallObserver (observer system)
%   sys_obs_ref  - BouncingBallKallmanObserver (Kalman observer)

% ====== PLANT SYSTEM ======
sys_ball = BouncingBallSubSystemClass();

sys_ball.mu = 0;        % Additional velocity at each impact
sys_ball.lambda = 1;    % Restitution coefficient
sys_ball.f_air = 0;     % Friction 


% ====== OBSERVER SYSTEM ======
sys_obs = BouncingBallObserver();

% Copy plant dynamics to observer
sys_obs.mu = sys_ball.mu;
sys_obs.lambda = sys_ball.lambda;
sys_obs.f_air = sys_ball.f_air;

% Choose the observer gains
sys_obs.L_c = 2.6*[1.8; 1.6];   % Flow gains for a stable observer
                                 % (not enough for convergence in every case)
sys_obs.L_d = 2*[0.0; 1.1];    % Jump gain (make it high to see a discrepancy)
sys_obs.K = [0, 0];             % Gain on jump detection
% BEWARE: K(1) < 0.5 is necessary to enforce transversality


% ====== KALMAN OBSERVER SYSTEM ======
sys_obs_ref = BouncingBallKallmanObserver();

% Copy plant dynamics to Kalman observer
sys_obs_ref.mu = sys_ball.mu;
sys_obs_ref.lambda = sys_ball.lambda;
sys_obs_ref.f_air = sys_ball.f_air;

% Choose the Kalman observer gains
sys_obs_ref.gain = 0.23;
sys_obs_ref.lambda_kallman = 0.4;
sys_obs_ref.gamma_kallman = 1;

% Deactivate saltation
sys_obs_ref.salted = false;


% ====== COMPOSITE SYSTEM ======
sys = CompositeHybridSystem('Ball', sys_ball, 'Observer', sys_obs, 'Kallman', sys_obs_ref);
obs_input = @(y_ball, ~) y_ball;
sys.setInput('Observer', obs_input);
sys.setInput('Kallman', obs_input);


% ====== SOLVER CONFIGURATION ======
max_dt_step = 0.03;
config = HybridSolverConfig('AbsTol', 1e-7, 'RelTol', 1e-7, 'MaxStep', max_dt_step);

end