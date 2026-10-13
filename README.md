# Saltation_Observer
Hybrid observers with **unknown jump times**: when the observer and the plant
do not jump together, the estimation error is driven by the saltation matrices
$M_{\rm before}$ and $M_{\rm after}$. The code illustrates why transversality and
the contraction of both matrices matter.

Papers: [CDC](https://ieeexplore.ieee.org/document/11312843) ([HAL](https://hal.parisnanterre.fr/ENSMP_CAS/hal-05273106)), and its journal version (TAC).

## Getting started
Requires MATLAB R2024b or later and the
[Hybrid Equations Toolbox](https://mathworks.com/matlabcentral/fileexchange/41372-hybrid-equations-toolbox).

```matlab
setupPaths              % add the project folders to the path
draw_figures_CDC_PDF    % CDC figures     -> figures/CDC
draw_figures_TAC_PDF    % journal figures -> figures/TAC
runTests                % automatic tests
```

| Folder | Content |
|---|---|
| `BouncingBall/` | bouncing ball with known ground, covariance analysis |
| `BouncingBallUnknownHeight/` | bouncing ball above an unknown ground |
| `utils/` | system and observer classes, gain searches, helpers |
| `tests/` | unit and integration tests |
| `data/` | datasets  |

## Unknown ground height
`BouncingBallUnknownHeight/UnknownGroundObserver.m` estimates
`x = [height; velocity; ground height]` from the height only. The ground height
cannot be corrected during flow: all its information comes from the impacts.
Set `gainProfile` to `'AfterBeforeContracting'` (both saltation products
contract) or `'BeforeContracting'` before running it. Together with the
covariance figures, it makes up `draw_figures_TAC_PDF.m`.

## Bouncing ball
`draw_figures_CDC_PDF.m` produces the CDC figures
(missed jumps, errors, synchronized case).`BouncingBall/observersBouncingBall.m` compares a Kalman observer with and
without saltation correction. 

### Examples of interest for the bouncing ball

Every computation is made with $x_0 = [5, 2]^\top$.
- $L_c = [0.8, 0.6]^\top, L_d = [0.1, 0.1]^\top, K = [0; 0]^\top$ illustrate local stability of the observer design when all conditions are met. Observer initialized at $\hat{x}_0 = 0.4x_0$.
- $L_c = [0, 0]^\top, L_d = [1, -0.392]^\top$ are adequate gains for synchronized jumps ($K = [1; 0]^\top, \hat{x}_0 = (1+6.10^{-1})x_0$) that cease to work for unknown jump times ($K = [0; 0]^\top, \hat{x}_0 = (1+6.10^{-3})x_0$). $L_d$ was found using the LMI search adapted to the synchronized case.
- $L_c = [10, 25]^\top, L_d = [0, 0]^\top, K = [3; 0]^\top$ show what happens when transversality of the observer is not met. Observer initialized at $\hat{x}_0 = (1-6.10^{-2})x_0$.
- $L_c = [0.1, 0.25]^\top, L_d = [0.1, 0.1]^\top, K = [0; 0]^\top$ show what happens when only $M_{\rm before}$ is contracting and not $M_{\rm after}$. Observer initialized at $\hat{x}_0 = (1-6.10^{-4})x_0$.

> [!NOTE]
> The same gains have been used in the different figures but not necessarily the same starting positions, they have been scaled in order to fit on the same figure.

### Covariance analysis
Monte-Carlo propagation of random initial errors through the first jump,
compared with the covariance predicted by $M_{\rm before}$ and $M_{\rm after}$:
`observersCovarianceDataGeneration.m` (slow, writes `data/*.mat`), then
`observersCovariancePlots.m`. See [docs/covariance_workflow.md](docs/covariance_workflow.md).

## Gain searches
Gains making $M_{\rm before}$ and $M_{\rm after}$ contracting:
`utils/K_search_naive.m` (grid search, very long),
`BouncingBallUnknownHeight/JSR_LMI_search.m` (needs [YALMIP](https://yalmip.github.io/) and an SDP solver)
and `utils/K_search_LMI.m` (needs the Robust Control Toolbox).
None of them is needed for the figures.



## Tests
`runTests` runs everything (~3 min), `runTests('unit')` only the fast tests
(~30 s); from a terminal, `matlab -batch "runTests"`. The integration tests
run the scripts and compare their results with the values of the papers.
Tests needing a missing toolbox are skipped.
