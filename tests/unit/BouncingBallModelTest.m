classdef BouncingBallModelTest < ProjectTestCase
    % Hybrid data (flow/jump maps and sets) of the plant models.

    methods (Test)
        function flowMapIsFreeFall(tc)
            ball = BouncingBallSubSystemClass();
            ball.f_air = 0;
            tc.verifyEqual(ball.flowMap([1; 2], 0, 0, 0), [2; -ball.g]);
        end

        function flowMapAirFrictionOpposesVelocity(tc)
            ball = BouncingBallSubSystemClass();
            ball.f_air = 0.01;
            tc.verifyEqual(ball.flowMap([1; 2], 0, 0, 0), [2; -ball.g - 0.04], 'AbsTol', 1e-14);
            tc.verifyEqual(ball.flowMap([1; -2], 0, 0, 0), [-2; -ball.g + 0.04], 'AbsTol', 1e-14);
        end

        function jumpMapIsRestitution(tc)
            ball = BouncingBallSubSystemClass();
            ball.lambda = 0.8;
            ball.mu = 2;
            tc.verifyEqual(ball.jumpMap([0; -3], 0, 0, 0), [0; 0.8*3 + 2], 'AbsTol', 1e-14);
        end

        function flowAndJumpSets(tc)
            ball = BouncingBallSubSystemClass();
            tc.verifyTrue(ball.flowSetIndicator([1; 1], 0, 0, 0));
            tc.verifyFalse(ball.jumpSetIndicator([1; 1], 0, 0, 0));
            tc.verifyTrue(ball.jumpSetIndicator([-0.1; -1], 0, 0, 0));
            tc.verifyFalse(ball.flowSetIndicator([-0.1; -1], 0, 0, 0));
            tc.verifyFalse(ball.jumpSetIndicator([0; 1], 0, 0, 0)); % moving up from the ground
        end

        function jumpJacobianMatchesJumpMap(tc)
            ball = BouncingBallSubSystemClass();
            ball.lambda = 0.8;
            ball.mu = 2;
            x = [0; -3];
            tc.verifyEqual(ball.jumpJacobian(x), ...
                numericalJacobian(@(z) ball.jumpMap(z, 0, 0, 0), x), 'AbsTol', 1e-8);
        end

        function unknownGroundJumpJacobianMatchesJumpMap(tc)
            ball = UnknownGroundBallSubSystemClass();
            x = [0.3; -4; 0.3];
            tc.verifyEqual(ball.jumpJacobian(x), ...
                numericalJacobian(@(z) ball.jumpMap(z), x), 'AbsTol', 1e-8);
        end

        function asymptoticOrbitOfUnknownGroundBall(tc)
            ball = UnknownGroundBallSubSystemClass();
            ball.lambda = 0.5;
            ball.mu = 2;
            tc.verifyEqual(ball.vStar(), 4, 'AbsTol', 1e-14);
            % v* is a fixed point of the impact map
            x_plus = ball.jumpMap([0; -ball.vStar(); 0]);
            tc.verifyEqual(x_plus(2), ball.vStar(), 'AbsTol', 1e-14);
        end
    end
end

function J = numericalJacobian(fun, x)
h = 1e-6;
n = numel(x);
J = zeros(numel(fun(x)), n);
for i = 1:n
    d = zeros(n, 1); d(i) = h;
    J(:, i) = (fun(x + d) - fun(x - d))/(2*h);
end
end
