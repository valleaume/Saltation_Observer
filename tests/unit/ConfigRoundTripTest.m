classdef ConfigRoundTripTest < ProjectTestCase
    % saveConfigToFile -> loadConfigFromFile must restore the parameters.

    methods (Test)
        function roundTripRestoresParameters(tc)
            folder = tc.applyFixture(matlab.unittest.fixtures.TemporaryFolderFixture).Folder;

            [~, config, sys_ball, sys_obs, sys_obs_ref] = observersCovarianceConfig();
            sys_ball.lambda = 0.9;
            sys_ball.mu = 1.5;
            sys_obs.L_d = [0.25; -0.5];
            sys_obs_ref.salted = true;

            file = saveConfigToFile(sys_ball, sys_obs, sys_obs_ref, config, 'roundtrip', folder);
            tc.verifyTrue(isfile(file));

            [b, o, r, c] = evalc_load(file);
            tol = {'AbsTol', 1e-6};
            tc.verifyEqual([b.lambda, b.mu, b.f_air], [sys_ball.lambda, sys_ball.mu, sys_ball.f_air], tol{:});
            tc.verifyEqual(o.L_c(:), sys_obs.L_c(:), tol{:});
            tc.verifyEqual(o.L_d(:), sys_obs.L_d(:), tol{:});
            tc.verifyEqual(o.K(:), sys_obs.K(:), tol{:});
            tc.verifyEqual([o.lambda, o.mu], [b.lambda, b.mu]);   % observer copies plant
            tc.verifyEqual([r.gain, r.lambda_kallman, r.gamma_kallman], ...
                [sys_obs_ref.gain, sys_obs_ref.lambda_kallman, sys_obs_ref.gamma_kallman], tol{:});
            tc.verifyEqual(r.salted, sys_obs_ref.salted);
            tc.verifyEqual(c.ode_options.MaxStep, config.ode_options.MaxStep, tol{:});
            tc.verifyEqual(c.ode_options.RelTol, config.ode_options.RelTol, 'RelTol', 1e-3);
        end

        function missingFileErrors(tc)
            tc.verifyError(@() loadConfigFromFile('does_not_exist.txt'), ?MException);
        end
    end
end

function [b, o, r, c] = evalc_load(file)
% loadConfigFromFile prints a message: keep the test log quiet.
[~, b, o, r, c] = evalc('loadConfigFromFile(file)');
end
