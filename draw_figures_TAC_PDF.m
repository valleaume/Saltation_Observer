%% draw_figures_TAC_PDF - Generate and export observer-analysis figures
%
% Runs the two observer plotting scripts and saves their figures as PDFs in
% the figures folder. The covariance dataset is selected in
% observersCovariancePlots.m.

project_folder = setupPaths();

% Output folder (can be overridden by defining figures_folder beforehand)
if ~exist('figures_folder', 'var'), figures_folder = fullfile(project_folder, 'figures', 'TAC'); end
if ~isfolder(figures_folder)
    mkdir(figures_folder);
end

%% Observer covariance analysis
close all;
run(fullfile(project_folder, 'BouncingBall','observersCovariancePlots.m'));

myPrintPDF(figure(6), fullfile(figures_folder, 'covariance_error_before_first_jump')); %);
myPrintPDF(figure(7), fullfile(figures_folder, 'covariance_error_after_first_jump')); %);

%% Unknown-ground observer analysis
gain_profiles = {'AfterBeforeContracting', 'BeforeContracting'};
for profile_index = 1:numel(gain_profiles)
    gainProfile = gain_profiles{profile_index};
    close all;
    run(fullfile(project_folder, 'BouncingBallUnknownHeight', 'UnknownGroundObserver.m'));

    myPrintPDF(figure(1), fullfile(figures_folder, ['unknown_ground_states_' gainProfile]));
    myPrintPDF(figure(2), fullfile(figures_folder, ['unknown_ground_height_estimate_' gainProfile]));
    myPrintPDF(figure(5), fullfile(figures_folder, ['unknown_ground_continuous_lyapunov_' gainProfile]));
end

fprintf('Observer figures exported to %s\n', figures_folder);