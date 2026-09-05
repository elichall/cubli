# V1 Modeling (MATLAB)

MATLAB scripts deriving the edge-balance LQR gains used by the firmware. See the [top-level README](../README.md) for system architecture and project status.

## Files

- `Constants.m` — physical constants: motor/driver/battery ratings, cube geometry, body/wheel inertias at each edge configuration (min/mid/max), and LQR weighting bounds. Run implicitly via `main.m`.
- `main.m` — builds the linearized edge-balance plant matrices (`Ae_max`/`Be_max`, `Ae_min`/`Be_min`) from `Constants.m`, checks the momentum-transfer energy inequality, then solves the LQR problem (`lqr()`) to produce gain matrices `Ke_max`/`Ke_min`, plus a root-locus plot of the closed-loop system.
- `Testing.m` — scratch script for validating the coordinate-transform math (sensor frame → natural body → virtual body → edge frame) used in the firmware's `updateOrientation`/`updateStateVector`.

## Link to firmware

The `Ke_max`/`Ke_min` outputs of `main.m` are hand-transcribed into `../v1-firmware/include/gains.h` as `gainEdgeMax`/`gainEdgeMin` (each padded from a 1x4 single-axis gain vector into the appropriate row of the 3x12 full-state gain matrix). There's currently no automated export step — if you change `Constants.m` or re-tune the LQR weights in `main.m`, you must manually re-copy the resulting `Ke_max`/`Ke_min` values into `gains.h`.
