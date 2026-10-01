classdef SaltationTest < ProjectTestCase
    % Saltation matrices: generic formula, class methods, reference values
    % from the papers, and an independent check by finite differences.

    methods (Test)
        % ---- generic formula utils/saltationMatrix.m ----
        function scalarCase(tc)
            % 1D: S = a + (c - a*b)/b = c/b
            tc.verifyEqual(saltationMatrix(0.3, 1, -2, 5), 5/-2, 'AbsTol', 1e-14);
        end

        function errorWhenFlowTangentToGuard(tc)
            tc.verifyError(@() saltationMatrix(eye(2), [1, 0], [0; 1], [1; 1]), ...
                'saltationMatrix:notTransverse');
        end

        function identityWhenJumpIsIdentityAndFlowContinuous(tc)
            f = [-1; 2];
            tc.verifyEqual(saltationMatrix(eye(2), [1, 0], f, f), eye(2), 'AbsTol', 1e-14);
        end

        function jumpJacobianConventionDoesNotMatter(tc)
            % The old scripts used J = [1 0; 0 -lambda] instead of the true
            % Jacobian [0 0; 0 -lambda]: both give the same saltation matrix
            % because they differ only along the guard normal.
            ball = BouncingBallSubSystemClass();
            ball.lambda = 0.8; ball.mu = 2;
            x = [0; -3];
            f_minus = ball.flowMap(x, 0, 0, 0);
            f_plus = ball.flowMap(ball.jumpMap(x, 0, 0, 0), 0, 0, 0);
            S_old = saltationMatrix([1, 0; 0, -0.8], [1, 0], f_minus, f_plus);
            tc.verifyEqual(ball.saltationMatrix(x), S_old, 'AbsTol', 1e-14);
        end

        % ---- reference values (papers) ----
        function covarianceExampleMatchesReference(tc)
            G = goldenValues();
            [~, ~, ~, sys_obs] = observersCovarianceConfig();
            [M_before, M_after] = sys_obs.saltationMatrices([0; -4.85]);
            tc.verifyEqual(M_before, G.plots.M_before, 'AbsTol', 1e-12);
            tc.verifyEqual(M_after, G.plots.M_after, 'AbsTol', 1e-12);
        end

        function knownGroundExampleMatchesReference(tc)
            G = goldenValues();
            ball = BouncingBallSubSystemClass();
            ball.mu = 0; ball.lambda = 1; ball.f_air = 0;
            [M_before, M_after] = ball.errorSaltationMatrices([0; -10.0995], [0.1; 0.1]);
            tc.verifyEqual(M_before, G.obsBB.M_before, 'AbsTol', 1e-12);
            tc.verifyEqual(M_after, G.obsBB.M_after, 'AbsTol', 1e-12);
        end

        function zeroJumpGainGivesPlantSaltation(tc)
            ball = BouncingBallSubSystemClass();
            ball.lambda = 0.8; ball.mu = 2;
            x = [0; -3];
            [M_before, M_after] = ball.errorSaltationMatrices(x, [0; 0]);
            tc.verifyEqual(M_before, ball.saltationMatrix(x), 'AbsTol', 1e-14);
            tc.verifyEqual(M_after, M_before, 'AbsTol', 1e-14);
        end

        function unknownGroundPlantMatchesObserverFactors(tc)
            % UnknownGroundBallObserver.saltationFactors is hand-derived;
            % its Xi must be the plant saltation matrix.
            ball = UnknownGroundBallSubSystemClass();
            obs = UnknownGroundBallObserver();
            obs.lambda = ball.lambda; obs.mu = ball.mu; obs.g = ball.g;
            v = 4;
            Xi = obs.saltationFactors(v);
            tc.verifyEqual(ball.saltationMatrix([0; -v; 0]), Xi, 'AbsTol', 1e-12);
        end

        % ---- independent check by finite differences on simulations ----
        function bouncingBallMatchesFiniteDifferences(tc)
            ball = BouncingBallSubSystemClass();
            ball.lambda = 0.8; ball.mu = 2; ball.f_air = 0;
            x = [0; -3];
            S_fd = finiteDifferenceSaltation(ball, x, @(z) z(1));
            tc.verifyEqual(ball.saltationMatrix(x), S_fd, 'AbsTol', 1e-5);
        end

        function bouncingBallWithFrictionMatchesFiniteDifferences(tc)
            ball = BouncingBallSubSystemClass();
            ball.lambda = 0.8; ball.mu = 2; ball.f_air = 0.01;
            x = [0; -5];
            S_fd = finiteDifferenceSaltation(ball, x, @(z) z(1));
            tc.verifyEqual(ball.saltationMatrix(x), S_fd, 'AbsTol', 1e-5);
        end

        function unknownGroundMatchesFiniteDifferences(tc)
            ball = UnknownGroundBallSubSystemClass();
            x = [0.2; -ball.vStar(); 0.2];
            S_fd = finiteDifferenceSaltation(ball, x, @(z) z(1) - z(3));
            tc.verifyEqual(ball.saltationMatrix(x), S_fd, 'AbsTol', 1e-5);
        end
    end
end
