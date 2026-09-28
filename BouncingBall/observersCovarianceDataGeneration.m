%% observersCovarianceDataGeneration - Generate observer covariance datasets
%
% This script generates random initial conditions and propagates them through
% the coupled ball-observer-Kalman system, collecting data for analysis.
%
% Data is saved as a timestamped .mat file in the data/ folder.
% To skip regeneration and load existing data, set GENERATE_POINTS = false
% and specify the data_to_load filename.

addpath('utils');

% ====== USER CONFIGURATION ======
GENERATE_POINTS = true;              % Set to true to generate new data
data_to_load = 'raw-bouncing-ball-after-before-05-Mar-2026.mat';  % File to load if not generating
n_points = 1000;                      % Number of random initial conditions to generate


% ====== LOAD SYSTEM CONFIGURATION ======
nameString = 'raw-bouncing-ball-after-before-';
[sys, config, sys_ball, sys_obs, sys_obs_ref] = observersCovarianceConfig();
saveConfigToFile(sys_ball, sys_obs, sys_obs_ref, config, nameString);

% ====== DISTRIBUTION OF INITIAL CONDITIONS ======
mu = [1; 2];                    % Mean vector (expectation)
sigma = 1e-6*[2 0; 0 2];       % Covariance matrix


if GENERATE_POINTS
    
    % Generate random points
    rng('default'); % For reproducibility
    points = mvnrnd(mu, sigma, n_points);
    
    % Plot the initial conditions
    figure(1);
    scatter(points(:,1), points(:,2), 'filled');
    xlabel('x');
    ylabel('v');
    title('Initial distribution of points (t=0)');
    axis equal;
    grid on;
    
    
    % Propagate those points through the system
    disp('Solving for random initial conditions...')
    data_x = {};
    data_t = {};
    data_v = {};
    data_x_ref = {};
    data_v_ref = {};
    data_t_ref = {};
    observer_jumps_before = {};
    
    for index = 1:n_points
        % X_0 is plant state, hat{X_0} is observer state
        x0_cell = {...
            [1; 2]; ...                                           % Ball: [x; v]
            [points(index, 1); points(index, 2)]; ...            % Observer: [x; v]
            [points(index, 1); points(index, 2); reshape(eye(2), [4,1])]  % Kalman: [x; v; P_flat]
        };
        
        tspan = [0, 2];   % Time span for integration
        jspan = [0, 15];  % Jump span
        
        % Solve coupled system 
        sol = sys.solve(x0_cell, tspan, jspan, config);
        
        % Collection: observer data
        data_x{end+1}  = sol('Observer').x(:,1);
        data_v{end+1}  = sol('Observer').x(:,2);
        data_t{end+1} = sol('Observer').t;
        
        % Collection: plant data (reference)
        data_x_ref{end+1}  = sol('Ball').x(:,1);
        data_v_ref{end+1}  = sol('Ball').x(:,2);
        data_t_ref{end+1}  = sol('Ball').t;
        
        % Verify time indices match
        assert(isequal(sol('Ball').t, sol('Observer').t), ...
            "discrepancy in time index between Ball and Observer")
        
        % Track if observer jumps before or after plant
        mask_jump_after = (sol('Ball').j - sol('Observer').j) > 0;
        mask_jump_before = (sol('Ball').j - sol('Observer').j) < 0;
        sign_jump = zeros(1,length(mask_jump_after));
        for i=2:length(mask_jump_after)
            if mask_jump_before(i)
                sign_jump(i) = +1;   % Observer jumps before ball
            else
                if mask_jump_after(i)
                    sign_jump(i) = -1;  % Observer jumps after ball
                else
                    sign_jump(i) = sign_jump(i-1);
                end
            end
        end
        observer_jumps_before{end+1} = sign_jump;
        
        if mod(index, 100) == 0
            fprintf('Completed %d / %d initial conditions\n', index, n_points);
        end
    end
    
    % Pad all cell arrays to uniform size for matrix conversion
    data_x = cell2mat(padCellToUniformSize(data_x, NaN));
    data_t = cell2mat(padCellToUniformSize(data_t, NaN));
    data_v = cell2mat(padCellToUniformSize(data_v, NaN));
    
    data_x_ref = cell2mat(padCellToUniformSize(data_x_ref, NaN));
    data_t_ref = cell2mat(padCellToUniformSize(data_t_ref, NaN));
    data_v_ref = cell2mat(padCellToUniformSize(data_v_ref, NaN));
    data_jumps = cell2mat(padCellToUniformSize(observer_jumps_before, NaN));
    
    % Save dataset with timestamped filename
    today = string(datetime("today"));
    datas_filename = strcat('data/',nameString, today);
    fprintf('Saving data to: %s.mat\n', datas_filename);
    save(datas_filename, "data_x", "data_v", "data_t", "data_v_ref", "data_x_ref", "data_jumps")
    
    fprintf('Data generation complete!\n');
    
% else
    
%     % Load previously computed dataset
%     fprintf('Loading data from: data/%s\n', data_to_load);
%     dataset = load("data/"+data_to_load);
%     data_x = dataset.data_x;
%     data_v = dataset.data_v;
%     data_x_ref = dataset.data_x_ref;
%     data_v_ref = dataset.data_v_ref;
%     data_t = dataset.data_t;
%     data_jumps = dataset.data_jumps;
    
%     fprintf('Data loaded successfully!\n');
    
end

% Store data and configuration in workspace for use by analyzing scripts
%assignin('base', 'data_x', data_x);
%assignin('base', 'data_v', data_v);
