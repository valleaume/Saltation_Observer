classdef ObserverTest < ProjectTestCase
    % BouncingBallObserver: copy of the plant plus output injection.

    methods (Test)
        function zeroErrorFlowEqualsPlant(tc)
            [ball, obs] = matchedPair();
            for x = [[1; 2], [3; -1], [0.5; 0]]
                tc.verifyEqual(obs.flowMap(x, x(1), 0, 0), ball.flowMap(x, 0, 0, 0), 'AbsTol', 1e-14);
            end
        end

        function zeroErrorJumpEqualsPlant(tc)
            [ball, obs] = matchedPair();
            x = [0; -3];   % on the ground
            tc.verifyEqual(obs.jumpMap(x, x(1), 0, 0), ball.jumpMap(x, 0, 0, 0), 'AbsTol', 1e-14);
        end

        function flowGainCorrectsError(tc)
            [~, obs] = matchedPair();
            obs.L_c = [0.8; 0.6];
            x_hat = [1; 2];
            y = 1.5;
            tc.verifyEqual(obs.flowMap(x_hat, y, 0, 0) - obs.flowMap(x_hat, x_hat(1), 0, 0), ...
                (y - x_hat(1))*obs.L_c, 'AbsTol', 1e-14);
        end

        function jumpDetectionGainK(tc)
            [~, obs] = matchedPair();
            x_hat = [0.1; -1];   % estimate slightly above the ground
            y = -0.1;            % but the measurement is below
            obs.K = [0; 0];
            tc.verifyFalse(obs.jumpSetIndicator(x_hat, y, 0, 0));
            obs.K = [1; 0];      % detection uses x_hat + K*(y - h)
            tc.verifyTrue(obs.jumpSetIndicator(x_hat, y, 0, 0));
        end

        function nominalPlantCopiesParameters(tc)
            [~, obs] = matchedPair();
            plant = obs.nominalPlant();
            tc.verifyEqual([plant.g, plant.lambda, plant.mu, plant.f_air], ...
                [obs.g, obs.lambda, obs.mu, obs.f_air]);
        end

        function saltationWarnsWhenKIsNonZero(tc)
            [~, obs] = matchedPair();
            obs.K = [0.3; 0];
            tc.verifyWarning(@() obs.saltationMatrices([0; -3]), 'BouncingBallObserver:nonZeroK');
        end
    end
end

function [ball, obs] = matchedPair()
ball = BouncingBallSubSystemClass();
ball.lambda = 0.8;
ball.mu = 2;
ball.f_air = 0.01;
obs = BouncingBallObserver();
obs.lambda = ball.lambda;
obs.mu = ball.mu;
obs.f_air = ball.f_air;
obs.K = [0; 0];
end
