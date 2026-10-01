classdef ProjectTestCase < matlab.unittest.TestCase
    % Common setup for the project tests: project path, invisible figures,
    % and a skip (not a failure) when the Hybrid Equations Toolbox is missing.

    properties
        ProjectFolder
    end

    methods (TestClassSetup)
        function setupProject(tc)
            tc.ProjectFolder = setupPaths();
            tc.assumeEqual(exist('HybridSubsystem', 'class'), 8, ...
                'Hybrid Equations Toolbox is not installed.');

            old_visibility = get(groot, 'defaultFigureVisible');
            set(groot, 'defaultFigureVisible', 'off');
            tc.addTeardown(@set, groot, 'defaultFigureVisible', old_visibility);
        end
    end

    methods (TestMethodTeardown)
        function closeFigures(~)
            close all;
        end
    end
end
