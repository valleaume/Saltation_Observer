classdef (TestTags = {'Integration'}) PaperFiguresIT < ProjectTestCase
    % The figure scripts of the papers run and export their PDFs
    % (into a temporary folder, the committed figures are not touched).

    methods (Test)
        function cdcFigures(tc)
            folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            cdc_folder = fullfile(folder, 'CDC');
            mkdir(fullfile(folder, 'TAC'));
            runScriptIn(fullfile(tc.ProjectFolder, 'draw_figures_CDC_PDF.m'), ...
                struct('figures_folder', cdc_folder));

            expected = {'Miss_jump', 'Miss_jump_x', 'Miss_jump_v', 'Position error', ...
                'Velocity error', 'Norm_error', 'Norm_error_stable_no_mask', 'Position_synchronization'};
            tc.verifyPdfs(cdc_folder, expected);
            tc.verifyPdfs(fullfile(folder, 'TAC'), {'Norm_error_stable_no_mask'});
        end

        function tacFigures(tc)
            folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            runScriptIn(fullfile(tc.ProjectFolder, 'draw_figures_TAC_PDF.m'), ...
                struct('figures_folder', folder));

            expected = {'covariance_error_before_first_jump', 'covariance_error_after_first_jump'};
            for profile = {'AfterBeforeContracting', 'BeforeContracting'}
                expected = [expected, strcat({'unknown_ground_states_', ...
                    'unknown_ground_height_estimate_', 'unknown_ground_continuous_lyapunov_'}, profile{1})]; %#ok<AGROW>
            end
            tc.verifyPdfs(folder, expected);
        end
    end

    methods
        function verifyPdfs(tc, folder, names)
            for k = 1:numel(names)
                file = dir(fullfile(folder, [names{k} '.pdf']));
                tc.verifyNotEmpty(file, sprintf('%s.pdf not exported.', names{k}));
                if ~isempty(file)
                    tc.verifyGreaterThan(file.bytes, 1000, sprintf('%s.pdf is empty.', names{k}));
                end
            end
        end
    end
end
