function draw_figures_ECC(figures_folder)
%% draw_figures_ECC - Generate and export the ECC figures
%
% Local convergence of the observer of the stick-slip oscillator on a table
% at constant speed, with unknown table speed and friction levels
% (FrictionOscillator/FrictionConstantSpeedObserver.m, gain profile 'Both'),
% from an initial error on every state. Saved as PDFs in the figures folder:
% the states and the error in each coordinate, and the Lyapunov function
% theta'*P*theta of the certificate, with the 'big' fonts of myPrintPDF.

project_folder = setupPaths();

if nargin == 0 || isempty(figures_folder)
    figures_folder = fullfile(project_folder, 'figures', 'ECC');
end
if ~isfolder(figures_folder)
    mkdir(figures_folder);
end

%% Stick-slip friction observer, table at constant speed
gainProfile = 'Both';
tspan = [];   % default of the script
close all;
run(fullfile(project_folder, 'FrictionOscillator', 'FrictionConstantSpeedObserver.m'));

% The states and the estimates during the transient only
set(findobj(figure(1), 'Type', 'axes'), 'XLim', [0, 12]);
myPrintPDF(figure(1), fullfile(figures_folder, 'constant_speed_mass_states'), 'Style', 'big');
set(findobj(figure(2), 'Type', 'axes'), 'XLim', [0, 24]);
myPrintPDF(figure(2), fullfile(figures_folder, 'constant_speed_parameter_estimates'), 'Style', 'big');
myPrintPDF(figure(6), fullfile(figures_folder, 'constant_speed_error_per_coordinate'), 'Style', 'big');
myPrintPDF(figure(5), fullfile(figures_folder, 'constant_speed_lyapunov'), 'Style', 'big');

fprintf('ECC figures exported to %s\n', figures_folder);

end
