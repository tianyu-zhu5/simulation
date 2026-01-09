# IGNITE v3 RUNBOOK — Pressure-Control Ignite (5x5)

Outputs for every run:
- `out/pyramid_5x5/pressure_ignite_YYYYMMDD_HHMMSS/summary.txt`
- `out/pyramid_5x5/pressure_ignite_YYYYMMDD_HHMMSS/metrics_pressure.csv`
- `out/pyramid_5x5/pressure_ignite_YYYYMMDD_HHMMSS/errors.json`
- `out/pyramid_5x5/pressure_ignite_YYYYMMDD_HHMMSS/checkpoint_last_ok.mph` (only if any step succeeds)

Baseline input (initial values):
- `out/pyramid_5x5/sim_20260107_152913/Pyramid_5x5_checkpoint_last_ok.mph`

## (1) Legacy-compatible (900s, segregated, no ramp)
```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_pressure_ignite_watchdog.ps1 `
  -FromSimDir out\pyramid_5x5\sim_20260107_152913\ `
  -PloadKPa 0.6 `
  -SolverCoupling segregated `
  -IgniteRamp 0 `
  -TimeoutSec 900
```

## (2) Fully coupled single attempt (900s)
```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_pressure_ignite_watchdog.ps1 `
  -FromSimDir out\pyramid_5x5\sim_20260107_152913\ `
  -PloadKPa 0.6 `
  -SolverCoupling fully_coupled `
  -IgniteRamp 0 `
  -TimeoutSec 900
```

## (3) Fully coupled ramp (900s, explicit legacy ramp list)
```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_pressure_ignite_watchdog.ps1 `
  -FromSimDir out\pyramid_5x5\sim_20260107_152913\ `
  -PloadKPa 0.6 `
  -SolverCoupling fully_coupled `
  -IgniteRamp 1 `
  -IgniteRampList "0.25,0.5,0.75,1.0" `
  -TimeoutSec 900
```

## (4) “Half-hour robust ignite” (watchdog 1800s, internal budget 1500s)
Recommended for maximizing the chance of producing a success checkpoint within 30 minutes.
```powershell
powershell -ExecutionPolicy Bypass -File scripts/run_pressure_ignite_watchdog.ps1 `
  -FromSimDir out\pyramid_5x5\sim_20260107_152913\ `
  -PloadKPa 0.6 `
  -SolverCoupling fully_coupled `
  -IgniteTwoStage 1 `
  -IgniteRamp 1 `
  -IgniteRampDefaultMode high_to_low `
  -FcMode robust `
  -FcDamped 1 `
  -FcLineSearch 1 `
  -SolverStabilization 1 `
  -IgniteBudgetSec 1500 `
  -TimeoutSec 1800
```

