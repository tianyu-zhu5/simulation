# RESULT — Local RP around onset (1.02285–1.0230)

## Data sources
- `out/pyramid_5x5/rp_20260108_122447/RP_raw.csv` (generated from `out/pyramid_5x5/sim_20260107_171121/`).

## Note on the “3 valid points” assumption
In the current workspace, `RP_raw.csv` files under `out/pyramid_5x5/rp_*` contain **2** finite-`R_total_ohm` (valid) points at:
- `delta_total_um = 1.02285`
- `delta_total_um = 1.0230`

I did **not** find a third valid point at `delta_total_um = 1.0231` in any existing `RP_raw.csv`/`RP_interp.csv`/`Pyramid_5x5_metrics.csv` under `out/pyramid_5x5/`.
This document therefore reports the **local two-point** estimate based on the available RP_raw data.

## Local ΔR/R0–P curve
Definition:
- Let `R0` be the `R_total_ohm` at the **minimum valid** effective pressure point `P0`.
- `ΔR/R0(P) = (R_total(P) - R0) / R0`.

From `out/pyramid_5x5/rp_20260108_122447/RP_raw.csv`:
- `P0 = 572.969 Pa` at `delta_total_um = 1.02285`
- `R0 = 339.787305753358 ohm`

| delta_total_um | P_eff_Pa | R_total_ohm | ΔR/R0 |
|---:|---:|---:|---:|
| 1.02285 | 572.969 | 339.787305753358 | 0 |
| 1.02300 | 576.873 | 339.787305753358 | 0 |

## Local sensitivity S(P)
Definition (finite difference on `ΔR/R0`):
- `S ≈ d(ΔR/R0)/dP`.

Two-point slope (between the two valid points):
- `S = (0 - 0) / (576.873 - 572.969) = 0 1/Pa`

For plotting/usage, you can associate this slope to the midpoint pressure:
- `P_mid ≈ (572.969 + 576.873)/2 = 574.921 Pa`

## Limitations / interpretation
1) **Only two valid points** are currently available (not three). This makes any “curve” and “sensitivity” estimate extremely underdetermined and sensitive to extraction artifacts.
2) **Very narrow pressure span** (~`3.904 Pa`), so even small numerical/rounding effects can dominate the slope estimate.
3) In this dataset, `Ac_m2` is identical at `1.02285` and `1.0230`, producing identical `Rc_ohm` and thus identical `R_total_ohm` under the RP model; consequently `ΔR/R0` is flat here.
4) **Convergence wall / hang risk near ~1.0231**: attempts near and above this neighborhood have repeatedly shown solver non-return (“PTC STARTED” hang) or fail-fast issues in mechanical continuation, limiting RP point density in the onset region.
5) Per current direction: **no further attempts** are made to add points at `1.02288` in this report.

## What would make this actionable
To obtain a meaningful local RP sensitivity in this region, we need at least **one more** valid point with finite `R_total_ohm` (ideally 3–6 points) spanning a larger `P_eff` range, while keeping the mechanical solve stable (no hangs).
