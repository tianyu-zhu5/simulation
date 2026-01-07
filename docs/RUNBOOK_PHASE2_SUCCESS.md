# RUNBOOK — Phase2 mechanical success (reach `last_success=1.0227`)

This runbook documents the **validated** Phase2 mechanical continuation that reaches:
- `last_success_delta_total_um = 1.0227`
- `early_exit = true`

Validated output directory:
- `out/pyramid_5x5/sim_20260107_133015/`

Key ingredients (must be enabled together):
- `contact_mode = augmented_lagrange`
- `RESUME_POST_ONSET_ONLY = 1` (skip expensive pre-onset)
- `PTC-first` in `post_onset_micro` (skip stationary via `SIM_POST_SKIP_STATIONARY=1`)
- **sufficient post budget** (`SIM_BUDGET_POST_ONSET_S` large enough; see below)

---

## 0) Preconditions

You must have a usable resume checkpoint+metrics at `last_success=1.0205`:
- `out/pyramid_5x5/sim_20260107_130330/Pyramid_5x5_checkpoint_last_ok.mph`
- `out/pyramid_5x5/sim_20260107_130330/Pyramid_5x5_metrics.csv`

This is the **resume seed** that was used to reach `1.0227`.

---

## 1) One-shot reproduction (PowerShell)

From repo root:

```powershell
$env:PHASE2_CONTACT_MODE='augmented_lagrange';
$env:RESUME_POST_ONSET_ONLY='1';
$env:RESUME_FROM_SIM_DIR=''; # (unused; kept empty intentionally)
$env:RESUME_FROM_DIR='out/pyramid_5x5/sim_20260107_130330/';

# Scheduling (key):
$env:SIM_GLOBAL_TIMEOUT_S='2400';
$env:SIM_BUDGET_POST_ONSET_S='2400';
$env:SIM_PER_SOLVE_TIMEOUT_S='120';

# post_onset_micro policy:
$env:SIM_POST_SKIP_STATIONARY='1'; # PTC-first

matlab -batch "run_pyramid_array_5x5_sim_runbook"
```

Expected: a new directory `out/pyramid_5x5/sim_YYYYMMDD_HHMMSS/` is created and should reach `last_success=1.0227` with `early_exit=true`.

---

## 2) Recommended defaults (scheduler-only, no physics changes)

These values were validated for the success run:
- `SIM_GLOBAL_TIMEOUT_S=2400`
- `SIM_BUDGET_POST_ONSET_S=2400`
- `SIM_PER_SOLVE_TIMEOUT_S=120` (kept, since stationary is skipped in post)
- `SIM_POST_SKIP_STATIONARY=1` (PTC-first in post_onset_micro)

Notes:
- Post-onset PTC steps typically cost **~270–300s per micro target**, so `budget_post_onset_micro_s=300` is too small to reach `>=1.0227`.

---

## 3) Key artifacts and checkpoints (must exist)

In the generated sim output directory (e.g. `out/pyramid_5x5/sim_20260107_133015/`):

- `Pyramid_5x5_summary.txt`
  - must include:
    - `contact_mode_effective: augmented_lagrange`
    - `delta_plan_mode_final: post_onset_micro`
    - `post_skip_stationary: 1`
    - `last_success_delta_total_um: 1.0227`
    - `early_exit: 1`
- `Pyramid_5x5_metrics.csv`
  - must include rows up to `delta_total_um=1.0227`
- `Pyramid_5x5_checkpoint_last_ok.mph`
  - should correspond to the last success delta (>= `1.0227`)
- `fallback_report.json`
  - must contain:
    - `ptc_first: true`
    - `micro_target_attempts` with `attempt_order="PTC"` for targets `1.0210 .. 1.0227`

