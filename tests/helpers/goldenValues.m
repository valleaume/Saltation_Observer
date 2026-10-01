function G = goldenValues()
% GOLDENVALUES - Reference results captured before the saltation refactoring
% (commit eac2f34), used as non-regression values by the tests.

% observersCovariancePlots.m: first impact x = [0; -4.85],
% gains of observersCovarianceConfig.m, dataset raw-bouncing-ball-after-before-05-Mar-2026.mat
G.plots.M_before = [-1 0; 1.8412371134020624 -1];
G.plots.M_after = [-1 0; 6.2412371134020628 -1];
G.plots.cov_before = [6.3285455411322134e-08 3.0609571916257326e-07; 3.0609571916257326e-07 1.5779233351258177e-06];
G.plots.cov_after = [6.0220632156514055e-08 4.1875204457033509e-08; 4.1875204457033509e-08 2.4676129601184524e-07];

% observersBouncingBall.m / K_search_*.m: x = [0; -10.0995], L_d = [0.1; 0.1], lambda = 1
G.obsBB.M_before = [-1.1000000000000001 0; 1.8406901331749095 -1];
G.obsBB.M_after = [-0.90000000000000013 0; 2.0406901331749094 -1];

% UnknownGroundObserver.m at (v*, tau*) for both gain profiles
G.AfterBeforeContracting.Mb = [0.37 0 2; 3.265 -0.5 -3.675; -0.28 0 1];
G.AfterBeforeContracting.Ma = [-2.37 0 4.74; 4.085 -0.5 -4.495; 0.28 0 0.44];
G.BeforeContracting.Mb = [-4.7 0 2; 2.575 -0.5 -3.675; 1.2 0 1];
G.BeforeContracting.Ma = [2.7 0 -5.4; 4.775 -0.5 -5.875; -1.2 0 3.4];
end
