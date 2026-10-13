# Covariance analysis workflow (bouncing ball)

Monte-Carlo study of how the estimation-error covariance is propagated
through a jump by the saltation matrices `M_before` and `M_after`.
Configuration, data generation and plotting are kept separate so the
figures can be redone without rerunning the simulations.

```
utils/observersCovarianceConfig.m         plant, observer, Kalman observer, solver
        │
        ├─► BouncingBall/observersCovarianceDataGeneration.m   (slow, ~5-10 min)
        │       writes data/raw-bouncing-ball-after-before-<date>.mat
        │       and    data/config/<name>_<timestamp>.txt
        │
        └─► BouncingBall/observersCovariancePlots.m            (fast, ~10 s)
                reads the .mat file named by data_to_load

BouncingBall/observersCovarianceAnalysis_pipeline.m   chains both
```

## Running

```matlab
% Plot the dataset used in the paper (committed in data/)
observersCovariancePlots

% Generate a new dataset, then plot it
GENERATE_POINTS = true; n_points = 1000;
observersCovarianceDataGeneration      % sets data_to_load to the new file
observersCovariancePlots
```

The settings at the top of the scripts (`GENERATE_POINTS`, `n_points`,
`data_folder`, `data_to_load`) keep any value already defined in the
workspace. Run `clear` first if you edited them in the file and a previous
value is still in the workspace.

## Parameters

| Where | What |
|---|---|
| `utils/observersCovarianceConfig.m` | plant (`mu`, `lambda`, `f_air`), observer gains (`L_c`, `L_d`, `K`), Kalman gains (`gain`, `lambda_kallman`, `gamma_kallman`, `salted`), solver tolerances |
| `observersCovarianceDataGeneration.m` | `n_points`, distribution of initial errors (`mu`, `sigma`), `tspan`, `jspan` |
| `observersCovariancePlots.m` | snapshot times `t_before`, `t_after`, `t_after_2`, impact point `x = [0; -4.85]` used for `M_before`/`M_after` |

## Dataset content

Each `.mat` file holds `data_x`, `data_v` (observer), `data_x_ref`,
`data_v_ref` (plant), `data_t` and `data_jumps` (+1 if the observer jumped
before the plant, -1 after). One column per initial condition, padded with
`NaN` to a common length (`utils/padCellToUniformSize.m`).

## Saving and reloading a configuration

```matlab
[sys, cfg, sb, so, sk] = observersCovarianceConfig();
file = saveConfigToFile(sb, so, sk, cfg, 'my_experiment');   % data/config/my_experiment_<timestamp>.txt
[sb, so, sk, cfg] = loadConfigFromFile(file);
```

The text file is human-readable (`key: value` lines, 6 decimals).
`example_config_usage.m` shows the full round trip; it is also checked by
`tests/unit/ConfigRoundTripTest.m`.
