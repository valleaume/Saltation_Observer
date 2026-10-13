function project_folder = setupPaths()
% SETUPPATHS - Add the project folders to the MATLAB path
%
%   project_folder = setupPaths()
%
% Call it (or any script of the repository, which calls it) from anywhere.
% Returns the absolute path of the repository root, to build data/ and
% figures/ paths that do not depend on the current folder.

project_folder = fileparts(mfilename('fullpath'));
addpath(project_folder, ...
    fullfile(project_folder, 'utils'), ...
    fullfile(project_folder, 'BouncingBall'), ...
    fullfile(project_folder, 'BouncingBallUnknownHeight'));
end
