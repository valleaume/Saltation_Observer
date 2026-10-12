function G = goldenValuesUnknownGroundObserver()
% GOLDENVALUESUNKNOWNGROUNDOBSERVER - Unknown-ground observer reference values.

% Matrices at (v*, tau*) for both gain profiles.
G.AfterBeforeContracting.Mb = [0.37 0 2; 3.265 -0.5 -3.675; -0.28 0 1];
G.AfterBeforeContracting.Ma = [-2.37 0 4.74; 4.085 -0.5 -4.495; 0.28 0 0.44];
G.BeforeContracting.Mb = [-4.7 0 2; 2.575 -0.5 -3.675; 1.2 0 1];
G.BeforeContracting.Ma = [2.7 0 -5.4; 4.775 -0.5 -5.875; -1.2 0 3.4];
end