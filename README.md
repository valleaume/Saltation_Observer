# Saltation_Observer
Hybrid observers for systems with **unknown jump times**: how the saltation
matrices $M_{\rm before}$ and $M_{\rm after}$ drive the estimation error of a
linear observer when the observer and the plant do not jump at the same
time. We show the importance of the transversality hypothesis alongside the
contractiveness of $M_{\rm before}$ and $M_{\rm after}$.

Related papers:
- CDC: [Saltation-Based analysis of estimation error in observers for hybrid systems with unknown jump times](https://ieeexplore.ieee.org/document/11312843) ([HAL](https://hal.parisnanterre.fr/ENSMP_CAS/hal-05273106))
- Journal version (TAC): covariance analysis and unknown-ground example, figures from `draw_figures_TAC_PDF.m`.

## Requirements
- MATLAB R2024b or later
- [Hybrid Equations Toolbox](https://mathworks.com/matlabcentral/fileexchange/41372-hybrid-equations-toolbox)
- Optional, for the gain searches only: Robust Control Toolbox (`K_search_LMI.m`),
  [YALMIP](https://yalmip.github.io/) + an SDP solver (`JSR_LMI_search.m`)

## Quick start
From the repository root in MATLAB:

```matlab
setupPaths            % add the project folders to the path
runTests('unit')      % ~30 s sanity check
draw_figures_TAC_PDF  % regenerate the journal figures in figures/TAC
```

Every script calls `setupPaths` itself, so it can be run from any folder.

## Entry points

| Script | What it does | Output |
|---|---|---|
| `draw_figures_CDC_PDF.m` | Known-ground bouncing ball: missed jumps, position/velocity errors, norm of the error, synchronized case | `figures/CDC/*.pdf` (CDC paper) |
| `draw_figures_TAC_PDF.m` | Runs `observersCovariancePlots` and `UnknownGroundObserver` (both gain profiles) and exports | `figures/TAC/*.pdf` (journal paper) |
| `BouncingBall/observersBouncingBall.m` | Plant + Kalman observer (salted / not salted), Lyapunov plots, `M_before`/`M_after` eigenvalues | figures |
| `BouncingBall/observersCovarianceDataGeneration.m` | Monte-Carlo propagation of random initial errors (slow) | `data/*.mat` |
| `BouncingBall/observersCovariancePlots.m` | Error distributions before/after the first jump, covariance predicted by the saltation matrices | figures |
| `BouncingBallUnknownHeight/UnknownGroundObserver.m` | Ball above an unknown ground height (see below); set `gainProfile` first | figures |
| `utils/K_search_LMI.m`, `utils/K_search_naive.m`, `BouncingBallUnknownHeight/JSR_LMI_search.m` | Search for gains making $M_{\rm before}$ and $M_{\rm after}$ contracting (LMI, grid search, joint spectral radius) | console |

The covariance workflow is detailed in [docs/covariance_workflow.md](docs/covariance_workflow.md).

## Repository layout

```
setupPaths.m, runTests.m      path setup and test runner
draw_figures_*_PDF.m          paper figures
BouncingBall/                 known-ground observer and covariance analysis
BouncingBallUnknownHeight/    unknown-ground observer and JSR gain search
utils/                        hybrid-system classes, observers, helpers
tests/                        automatic tests (unit/ and integration/)
data/                         datasets (.mat) and saved configurations
docs/                         workflow notes
Examples/                     exploratory work (billiards, ASLIP, toy systems),
                              not needed for the papers
legacy/                       superseded scripts kept for reference
```

## Notation and where it lives in the code

| Symbol | Meaning | Code |
|---|---|---|
| $L_c$ | flow gain of the observer | `BouncingBallObserver.L_c` |
| $L_d$ | jump gain of the observer | `BouncingBallObserver.L_d` |
| $K$ | correction of the observer jump set, $\hat x + K(y - h(\hat x))$; $K_1 < 0.5$ is needed for transversality | `BouncingBallObserver.K` |
| $S$ | saltation matrix of the plant, $S = Dg + (f^+ - Dg\, f^-)\nabla h / (\nabla h\, f^-)$ | `utils/saltationMatrix.m`, `BouncingBallSubSystemClass.saltationMatrix` |
| $M_{\rm before}$, $M_{\rm after}$ | error saltation matrices when the observer jumps before / after the plant ($K = 0$) | `BouncingBallObserver.saltationMatrices`, `BouncingBallSubSystemClass.errorSaltationMatrices`, `UnknownGroundBallObserver.saltationMatrices` |

The Jacobian of the jump map and the gradient of the guard are written by
hand in each plant class (`jumpJacobian`, `guardGradient`); flow and jump
maps are evaluated numerically. The tests check these derivatives against
finite differences of the simulated hybrid flow. Automatic differentiation
would remove the hand-written derivatives and is a possible future step.

## Unknown-ground observer

`UnknownGroundObserver.m` studies a bouncing ball whose ground height is
unknown to the observer. The augmented state is

```text
x = [height; velocity; ground height]
```

The observer estimates all three quantities from the measured absolute
position. During continuous flow, the ground-height state is constant and
does not appear in the measured dynamics, so its error cannot be corrected by
the flow gain. Information about the ground height arrives through the impact
events, which makes this script a useful example of jump-driven estimation
and of the difference between flow and jump observability.

Two gain profiles are available through the `gainProfile` variable:
`'AfterBeforeContracting'` (both saltation products contract) and
`'BeforeContracting'`.

## Examples of interest for the bouncing ball

Every computation is made with $x_0 = [5, 2]^\top$.
- $L_c = [0.8, 0.6]^\top, L_d = [0.1, 0.1]^\top, K = [0; 0]^\top$ illustrate local stability of the observer design when all conditions are met. Observer initialized at $\hat{x}_0 = 0.4x_0$.
- $L_c = [0, 0]^\top, L_d = [1, -0.392]^\top$ are adequate gains for synchronized jumps ($K = [1; 0]^\top, \hat{x}_0 = (1+6.10^{-1})x_0$) that cease to work for unknown jump times ($K = [0; 0]^\top, \hat{x}_0 = (1+6.10^{-3})x_0$). $L_d$ was found using the LMI search adapted to the synchronized case.
- $L_c = [10, 25]^\top, L_d = [0, 0]^\top, K = [3; 0]^\top$ show what happens when transversality of the observer is not met. Observer initialized at $\hat{x}_0 = (1-6.10^{-2})x_0$.
- $L_c = [0.1, 0.25]^\top, L_d = [0.1, 0.1]^\top, K = [0; 0]^\top$ show what happens when only $M_{\rm before}$ is contracting and not $M_{\rm after}$. Observer initialized at $\hat{x}_0 = (1-6.10^{-4})x_0$.

> [!NOTE]
> The same gains have been used in the different figures but not necessarily the same starting positions, they have been scaled in order to fit on the same figure.

## Tests

```matlab
runTests            % unit + integration (~3 min)
runTests('unit')    % unit tests only (~30 s)
runTests('integration')
runTests('examples')  % also try the Examples/ scripts (slow, may fail)
```

or from a terminal: `matlab -batch "runTests"` (non-zero exit code on failure).

- `tests/unit/` — plant and observer maps, saltation matrices (closed-form
  cases, reference values of the papers, finite-difference check),
  configuration save/load, helpers.
- `tests/integration/` — run the scripts end to end (bouncing-ball and
  unknown-ground observers, covariance pipeline on a small dataset, export
  of the CDC and TAC figures into a temporary folder, gain searches). Their
  numerical results are compared with the values of the papers
  (`tests/helpers/goldenValues.m`).

Tests that need a missing toolbox are skipped, not failed. To let a test
override a script setting, the scripts use
`if ~exist('name', 'var'), name = default; end` (see `tests/helpers/runScriptIn.m`).
