%% draw_paper_figures - Generate and export observer-analysis figures
%
% Runs the two observer plotting scripts and saves their figures as PDFs in
% the figures folder. The covariance dataset is selected in
% observersCovariancePlots.m.

addpath('utils');

figures_folder = fullfile(pwd, 'figures\TAC');
if ~isfolder(figures_folder)
    mkdir(figures_folder);
end

%% Observer covariance analysis
close all;
run('observersCovariancePlots.m');

myPrintPDF(figure(6), fullfile(figures_folder, 'covariance_error_before_first_jump')); %);
myPrintPDF(figure(7), fullfile(figures_folder, 'covariance_error_after_first_jump')); %);

%% Unknown-ground observer analysis
close all;
run('UnknownGroundObserver.m');

myPrintPDF(figure(1), fullfile(figures_folder, 'unknown_ground_states'));
myPrintPDF(figure(2), fullfile(figures_folder, 'unknown_ground_height_estimate'));
myPrintPDF(figure(3), fullfile(figures_folder, 'unknown_ground_lyapunov_contraction'));

fprintf('Observer figures exported to %s\n', figures_folder);