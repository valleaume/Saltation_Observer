classdef (TestTags = {'Integration', 'Examples'}) ExamplesSmokeIT < ProjectTestCase
    % Exploratory scripts of Examples/: only check that they still run.
    % Excluded from runTests() by default, run with runTests('examples').

    properties (TestParameter)
        script = exampleScripts();
    end

    methods (Test)
        function exampleRuns(tc, script)
            folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;
            % Some examples write to data/ or figures/ relative to the current folder
            tc.applyFixture(matlab.unittest.fixtures.CurrentFolderFixture(folder));
            mkdir(fullfile(folder, 'data'));
            mkdir(fullfile(folder, 'figures'));
            runScriptIn(fullfile(tc.ProjectFolder, 'Examples', script));
        end
    end
end

function scripts = exampleScripts()
files = dir(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'Examples', '*.m'));
scripts = {files.name};
end
