%% observersCovarianceAnalysis_pipeline - Integrated workflow for data generation and analysis
%
% This script optionally generates new data and then plots analysis figures.
% It maintains the original observersCovarianceAnalysis.m workflow while using
% the refactored data generation and plotting scripts.
%
% Configuration:
%   GENERATE_POINTS = true   -> generates new data (slow) then plots
%   GENERATE_POINTS = false  -> loads existing data and plots (fast)

% ====== CONFIGURATION ======
GENERATE_POINTS = false;              % Set to true to generate new data
data_to_load = 'raw-bouncing-ball-after-before-05-Mar-2026.mat';

% ====== PHASE 1: DATA GENERATION (if needed) ======
if GENERATE_POINTS
    fprintf('========== PHASE 1: DATA GENERATION ==========\n');
    observersCovarianceDataGeneration;
    % After generation completes, data_to_load should be set to the newly generated file
    % For now, we use the default filename used in the generation script
    data_to_load = sprintf('raw-bouncing-ball-covariance-%s.mat', string(datetime("today")));
else
    fprintf('========== PHASE 1: SKIPPING DATA GENERATION ==========\n');
    fprintf('Using existing data file: %s\n\n', data_to_load);
end

% ====== PHASE 2: PLOTTING & ANALYSIS ======
fprintf('========== PHASE 2: PLOTTING & ANALYSIS ==========\n');
observersCovariancePlots;

fprintf('\n========== PIPELINE COMPLETE ==========\n');
