function G = goldenValuesCovariancePlots()
% GOLDENVALUESCOVARIANCEPLOTS - Reference values for covariance plots.

% First impact x = [0; -4.85], configured gains, and the 05-Mar-2026 dataset.
G.M_before = [-1 0; 1.8412371134020624 -1];
G.M_after = [-1 0; 6.2412371134020628 -1];
G.cov_before = [6.3285455411322134e-08 3.0609571916257326e-07; 3.0609571916257326e-07 1.5779233351258177e-06];
G.cov_after = [6.0220632156514055e-08 4.1875204457033509e-08; 4.1875204457033509e-08 2.4676129601184524e-07];
end