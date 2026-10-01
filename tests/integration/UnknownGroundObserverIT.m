classdef (TestTags = {'Integration'}) UnknownGroundObserverIT < ProjectTestCase
    % Run the unknown-ground example for both gain profiles of the paper.

    properties (TestParameter)
        gainProfile = {'AfterBeforeContracting', 'BeforeContracting'};
    end

    methods (Test)
        function scriptRunsAndMatchesPaper(tc, gainProfile)
            ws = runScriptIn(fullfile(tc.ProjectFolder, 'BouncingBallUnknownHeight', 'UnknownGroundObserver.m'), ...
                struct('gainProfile', gainProfile));
            G = goldenValues();
            tc.verifyEqual(ws.Mb, G.(gainProfile).Mb, 'AbsTol', 1e-12);
            tc.verifyEqual(ws.Ma, G.(gainProfile).Ma, 'AbsTol', 1e-12);
            for k = [1, 2, 5]
                tc.verifyNotEmpty(findobj('Type', 'figure', 'Number', k), sprintf('Figure %d missing.', k));
            end
        end
    end
end
