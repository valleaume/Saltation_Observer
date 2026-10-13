classdef (TestTags = {'Integration'}) GainSearchIT < ProjectTestCase
    % Gain search scripts (LMI). K_search_naive.m is a brute-force grid
    % search of several hours and is not run here.

    methods (Test)
        function lmiSearchFindsFeasibleGain(tc)
            tc.assumeTrue(license('test', 'Robust_Toolbox') && exist('feasp', 'file') > 0, ...
                'Robust Control Toolbox (feasp) is not available.');
            ws = runScriptIn(fullfile(tc.ProjectFolder, 'utils', 'K_search_LMI.m'));
            tc.verifyLessThan(ws.t_min, 0, 'LMI reported infeasible.');
            tc.verifySize(ws.Ld, [2, 1]);
        end

        function jsrSearchReturnsCertificate(tc)
            tc.assumeTrue(exist('sdpvar', 'file') > 0, 'YALMIP is not installed.');
            % Coarse grid to keep the test short.
            [~, out] = evalc(['JSR_LMI_search(''zetaGrid'', 0.3, ''omegaGrid'', 6, ' ...
                '''nv'', 3, ''nBisect'', 8, ''verbose'', false)']);
            tc.verifyClass(out, 'struct');
        end
    end
end
