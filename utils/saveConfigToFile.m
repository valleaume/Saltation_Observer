function saveConfigToFile(sys_ball, sys_obs, sys_obs_ref, config, filename)
% SAVECONFIGTOFILE - Save configuration parameters to a readable text file
%
% This function extracts all relevant configuration parameters from the
% system objects and solver config, then saves them to a timestamped file
% in the 'data/config' directory in a human-readable format.
%
% Usage:
%   saveConfigToFile(sys_ball, sys_obs, sys_obs_ref, config, 'my_config')
%   % Creates: data/config/my_config_YYYY-MM-DD_HHMMSS.txt
%
% Inputs:
%   sys_ball      - BouncingBallSubSystemClass
%   sys_obs       - BouncingBallObserver
%   sys_obs_ref   - BouncingBallKallmanObserver
%   config        - HybridSolverConfig
%   filename      - Base name for the config file (without extension or timestamp)
%
% Output:
%   Saves a timestamped .txt file to data/config/ directory

    % Create config directory if it doesn't exist
    config_dir = 'data/config';
    if ~exist(config_dir, 'dir')
        mkdir(config_dir);
    end
    
    % Generate timestamped filename
    timestamp = string(datetime("now", 'Format', 'yyyy-MM-dd_HHmm'));
    filepath = fullfile(config_dir, sprintf('%s_%s.txt', filename, timestamp));
    
    % Open file for writing
    fid = fopen(filepath, 'w');
    
    % Write header
    fprintf(fid, '================================================================================\n');
    fprintf(fid, 'OBSERVER COVARIANCE ANALYSIS - CONFIGURATION FILE\n');
    fprintf(fid, 'Generated: %s\n', datetime("now"));
    fprintf(fid, '================================================================================\n\n');
    
    % PLANT SYSTEM PARAMETERS
    fprintf(fid, '--- PLANT SYSTEM (BouncingBallSubSystemClass) ---\n');
    fprintf(fid, 'mu:       %.6f\n', sys_ball.mu);
    fprintf(fid, 'lambda:   %.6f\n', sys_ball.lambda);
    fprintf(fid, 'f_air:    %.6f\n', sys_ball.f_air);
    fprintf(fid, '\n');
    
    % OBSERVER SYSTEM PARAMETERS
    fprintf(fid, '--- OBSERVER SYSTEM (BouncingBallObserver) ---\n');
    fprintf(fid, 'L_c (flow gains):\n');
    fprintf(fid, '  L_c_1: %.6f\n', sys_obs.L_c(1));
    fprintf(fid, '  L_c_2: %.6f\n', sys_obs.L_c(2));
    fprintf(fid, 'L_d (jump gain):\n');
    fprintf(fid, '  L_d_1: %.6f\n', sys_obs.L_d(1));
    fprintf(fid, '  L_d_2: %.6f\n', sys_obs.L_d(2));
    fprintf(fid, 'K (jump detection gains):\n');
    fprintf(fid, '  K_1: %.6f\n', sys_obs.K(1));
    fprintf(fid, '  K_2: %.6f\n', sys_obs.K(2));
    fprintf(fid, '\n');
    
    % KALMAN OBSERVER PARAMETERS
    fprintf(fid, '--- KALMAN OBSERVER (BouncingBallKallmanObserver) ---\n');
    fprintf(fid, 'gain:                     %.6f\n', sys_obs_ref.gain);
    fprintf(fid, 'lambda_kallman:           %.6f\n', sys_obs_ref.lambda_kallman);
    fprintf(fid, 'gamma_kallman:            %.6f\n', sys_obs_ref.gamma_kallman);
    fprintf(fid, 'salted:                   %s\n', onoff(sys_obs_ref.salted));
    fprintf(fid, '\n');
    
    % SOLVER PARAMETERS
    fprintf(fid, '--- SOLVER CONFIGURATION (HybridSolverConfig) ---\n');
    fprintf(fid, 'AbsTol:            %.3e\n', config.ode_options.AbsTol);
    fprintf(fid, 'RelTol:            %.3e\n', config.ode_options.RelTol);
    fprintf(fid, 'MaxStep:           %.6f\n', config.ode_options.MaxStep);
    fprintf(fid, '\n');
    
    % Footer
    fprintf(fid, '================================================================================\n');
    fprintf(fid, 'End of configuration file\n');
    fprintf(fid, '================================================================================\n');
    
    fclose(fid);
    
    fprintf('Configuration saved to: %s\n', filepath);
end

function str = onoff(value)
    % Simple helper to convert boolean to on/off string
    if value
        str = 'on';
    else
        str = 'off';
    end
end