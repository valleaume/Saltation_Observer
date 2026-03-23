function [sys_ball, sys_obs, sys_obs_ref, config] = loadConfigFromFile(filepath)
% LOADCONFIGFROMFILE - Load configuration parameters from a saved text file
%
% This function reads a configuration file previously saved by saveConfigToFile()
% and reconstructs the system objects with the saved parameters.
%
% Usage:
%   [sys_ball, sys_obs, sys_obs_ref, config] = loadConfigFromFile('data/config/my_config_2026-03-23_143022.txt')
%
% Inputs:
%   filepath - Full or relative path to the configuration .txt file
%
% Outputs:
%   sys_ball      - BouncingBallSubSystemClass with loaded parameters
%   sys_obs       - BouncingBallObserver with loaded parameters
%   sys_obs_ref   - BouncingBallKallmanObserver with loaded parameters
%   config        - HybridSolverConfig with loaded parameters

    % Check if file exists
    if ~isfile(filepath)
        error('Configuration file not found: %s', filepath);
    end
    
    % Initialize systems
    sys_ball = BouncingBallSubSystemClass();
    sys_obs = BouncingBallObserver();
    sys_obs_ref = BouncingBallKallmanObserver();
    
    % Read the file
    fid = fopen(filepath, 'r');
    lines = {};
    while ~feof(fid)
        line = fgetl(fid);
        if ~ischar(line)
            break;
        end
        lines{end+1} = line;
    end
    fclose(fid);
    
    % Parse parameters from lines
    config_params = struct();
    
    for i = 1:length(lines)
        line = lines{i};
        
        % Skip headers and empty lines
        if isempty(line) || startsWith(line, '-') || startsWith(line, '=')
            continue;
        end
        
        % Parse key = value lines
        if contains(line, '=')
            parts = split(line, '=');
            if length(parts) >= 2
                key = strtrim(parts{1});
                value_str = strtrim(parts{2});
                
                % Extract numeric value (before any comment)
                value_str = regexprep(value_str, '(%|#).*', '');
                value_str = strtrim(value_str);
                
                % Try to parse as number
                value = str2double(value_str);
                if ~isnan(value)
                    config_params.(matlab.lang.makeValidName(key)) = value;
                elseif strcmp(value_str, 'true') || strcmp(value_str, 'on')
                    config_params.(matlab.lang.makeValidName(key)) = true;
                elseif strcmp(value_str, 'false') || strcmp(value_str, 'off')
                    config_params.(matlab.lang.makeValidName(key)) = false;
                end
            end
        elseif contains(line, ':') && ~contains(line, '---') && ~contains(line, '===')
            % Handle lines with : separator (for array elements like "L_c(1) = value")
            parts = split(line, ':');
            if length(parts) >= 2
                key = strtrim(parts{1});
                value_str = strtrim(parts{2});
                
                % Extract numeric value (before any comment)
                value_str = regexprep(value_str, '(%|#).*', '');
                value_str = strtrim(value_str);
                
                % Try to parse as number
                value = str2double(value_str);
                if ~isnan(value)
                    config_params.(matlab.lang.makeValidName(key)) = value;
                elseif strcmp(value_str, 'true') || strcmp(value_str, 'on')
                    config_params.(matlab.lang.makeValidName(key)) = true;
                elseif strcmp(value_str, 'false') || strcmp(value_str, 'off')
                    config_params.(matlab.lang.makeValidName(key)) = false;
                end
            end
        end
    end
    
    disp(config_params);

    % Apply parameters to systems
    if isfield(config_params, 'mu')
        disp(config_params.mu)
        sys_ball.mu = config_params.mu;
    end
    if isfield(config_params, 'lambda')
        sys_ball.lambda = config_params.lambda;
    end
    if isfield(config_params, 'f_air')
        sys_ball.f_air = config_params.f_air;
    end
    
    % Observer gains (L_c)
    if isfield(config_params, 'L_c_1')
        sys_obs.L_c(1) = config_params.L_c_1;
    end
    if isfield(config_params, 'L_c_2')
        sys_obs.L_c(2) = config_params.L_c_2;
    end
    
    % Observer gains (L_d)
    if isfield(config_params, 'L_d_1')
        sys_obs.L_d(1) = config_params.L_d_1;
    end
    if isfield(config_params, 'L_d_2')
        sys_obs.L_d(2) = config_params.L_d_2;
    end
    
    % Observer guard gains (K)
    if isfield(config_params, 'K_1')
        sys_obs.K(1) = config_params.K_1;
    end
    if isfield(config_params, 'K_2')
        sys_obs.K(2) = config_params.K_2;
    end
    
    % Kalman observer gains
    if isfield(config_params, 'gain')
        sys_obs_ref.gain = config_params.gain;
    end
    if isfield(config_params, 'lambda_kallman')
        sys_obs_ref.lambda_kallman = config_params.lambda_kallman;
    end
    if isfield(config_params, 'gamma_kallman')
        sys_obs_ref.gamma_kallman = config_params.gamma_kallman;
    end
    if isfield(config_params, 'salted')
        sys_obs_ref.salted = config_params.salted;
    end
    
    % Copy plant dynamics to observers (consistency)
    sys_obs.mu = sys_ball.mu;
    sys_obs.lambda = sys_ball.lambda;
    sys_obs.f_air = sys_ball.f_air;
    sys_obs_ref.mu = sys_ball.mu;
    sys_obs_ref.lambda = sys_ball.lambda;
    sys_obs_ref.f_air = sys_ball.f_air;
    

    % Create solver config
    if isfield(config_params, 'AbsTol')
        AbsTol = config_params.AbsTol;
    end
    if isfield(config_params, 'RelTol')
        RelTol = config_params.RelTol;
    end
    if isfield(config_params, 'MaxStep')
        MaxStep = config_params.MaxStep;
    end
    config = HybridSolverConfig('MaxStep', MaxStep, 'AbsTol', AbsTol, 'RelTol', RelTol);
    
    fprintf('Configuration loaded from: %s\n', filepath);
end
