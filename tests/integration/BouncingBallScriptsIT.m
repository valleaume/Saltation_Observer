classdef (TestTags = {'Integration'}) BouncingBallScriptsIT < ProjectTestCase
    % Run the known-ground bouncing-ball scripts end to end.

    methods (Test)
        function observersBouncingBallRuns(tc)
            ws = runScriptIn(fullfile(tc.ProjectFolder, 'BouncingBall', 'observersBouncingBall.m'));
            G = goldenValues();
            tc.verifyEqual(ws.M_before, G.obsBB.M_before, 'AbsTol', 1e-12);
            tc.verifyEqual(ws.M_after, G.obsBB.M_after, 'AbsTol', 1e-12);
            tc.verifyGreaterThan(max(ws.sol('Ball').j), 0, 'The ball never bounced.');
        end

        function covariancePlotsReproducePaperValues(tc)
            ws = runScriptIn(fullfile(tc.ProjectFolder, 'BouncingBall', 'observersCovariancePlots.m'));
            G = goldenValues();
            tc.verifyEqual(ws.M_before, G.plots.M_before, 'AbsTol', 1e-12);
            tc.verifyEqual(ws.M_after, G.plots.M_after, 'AbsTol', 1e-12);
            tc.verifyEqual(ws.cov_before, G.plots.cov_before, 'RelTol', 1e-10);
            tc.verifyEqual(ws.cov_after, G.plots.cov_after, 'RelTol', 1e-10);
            for k = [2, 5, 6, 7, 8]
                tc.verifyNotEmpty(findobj('Type', 'figure', 'Number', k), sprintf('Figure %d missing.', k));
            end
        end

        function covariancePipelineGenerateThenPlot(tc)
            % Small data generation in a temporary folder, then plot it.
            folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            gen = runScriptIn(fullfile(tc.ProjectFolder, 'BouncingBall', 'observersCovarianceDataGeneration.m'), ...
                struct('GENERATE_POINTS', true, 'n_points', 20, 'data_folder', folder));

            data_file = fullfile(folder, gen.data_to_load);
            tc.assertTrue(isfile(data_file), 'Dataset was not written.');
            dataset = load(data_file);
            tc.verifyEqual(sort(fieldnames(dataset)), ...
                sort({'data_x'; 'data_v'; 'data_t'; 'data_x_ref'; 'data_v_ref'; 'data_jumps'}));
            tc.verifyEqual(size(dataset.data_x, 2), 20, 'One column per initial condition.');
            tc.verifyNotEmpty(dir(fullfile(folder, 'config', '*.txt')), 'Configuration file not saved.');

            plots = runScriptIn(fullfile(tc.ProjectFolder, 'BouncingBall', 'observersCovariancePlots.m'), ...
                struct('data_folder', folder, 'data_to_load', gen.data_to_load));
            tc.verifySize(plots.cov_before, [2, 2]);
        end
    end
end
