# STATUS_PHASE3 — Ac extraction diagnosis

## Update (2026-01-07 14:51) — Diagnose `Ac_m2=0` root cause (no new mechanical solves)

Inputs:
- Mechanical output (validated success run):
  - `out/pyramid_5x5/sim_20260107_133015/`
  - last_success: `1.0227` (early_exit=true)
  - force at last row (from metrics): `Fz_plate_top_int_N ≈ -5.69067e-06 N`
- Diagnosis output:
  - `out/pyramid_5x5/diag_ac_20260107_145744/Ac_diagnose.csv`

### Required checks

1) **Is `solid.dcnt1.Tn` evaluable at the solved state?**
- Result: **NO**
- Evidence (from `Ac_diagnose.csv`):
  - `solid.dcnt1.Tn`:
    - `nonNaN_count = 0`
    - `min/max = NaN/NaN`
    - error: `Undefined variable comp1.solid.dcnt1.Tn`
  - `solid.dcnt1.gap` similarly undefined.

2) **Does the integration selection `pc.destination` cover destination boundaries?**
- Result: **YES (non-empty)**
- Evidence:
  - selection used: `pc.destination.entities`
  - `selection_boundary_count = 126` (IDs preview in `Ac_diagnose.csv`)

3) **Candidate variables (cnt1/dcnt1/contact status) and Ac candidates**
- `solid.cnt1.Tn`:
  - `nonNaN_count = 755`
  - `min ≈ -4.89e-13`, `max ≈ 2.89e4` (Pa)
  - `Ac_candidate_m2 (Tn>0) ≈ 7.04e-09`
  - note: threshold `0` is likely too permissive; should use a positive threshold (e.g. 1–10 Pa) or `incontact`.
- `solid.cnt1.incontact`:
  - `nonNaN_count = 755`
  - `min=0`, `max=1`
  - `Ac_candidate_m2 (incontact>0.5) ≈ 2.324e-10`
- `solid.cnt1.gap`:
  - `nonNaN_count = 100/3846`
  - `min ≈ -1.09e-3`, `max ≈ 9.11e-4`
  - `Ac_candidate_m2 (gap<0) ≈ 1.32e-11`

### Conclusion (must choose A or B)

**A) Variable/selection/expression issue (fix postprocessing)**
- Root cause: current pipeline uses `solid.dcnt1.Tn` for `Ac_m2`, but in this 5×5 solved state `solid.dcnt1.*` is **undefined** while `solid.cnt1.*` is defined.
- Therefore `Ac_m2` in `out/pyramid_5x5/sim_20260107_133015/Pyramid_5x5_metrics.csv` stayed `0`, causing RP `Rc/R_total` to become `NaN` and `valid_points_for_RP=0`.

### Next-step suggestions (A/B routes)

Route A (recommended): fix Ac extraction (no physics changes)
- Change:
  - switch `Ac_m2` to be computed from `solid.cnt1.incontact` (preferred) or `solid.cnt1.Tn` with a positive threshold.
  - keep integrating on `pc.destination.entities`.
- Validate:
  - recompute/emit `Ac_m2>0` for the existing solved checkpoint(s) and then re-run RP; expect `valid_points_for_RP >= 1`.

Route B: continue pushing delta (only if Route A proves wrong)
- Change:
  - extend post_onset micro continuation beyond `1.0227` (e.g. `1.03/1.04`) to grow contact area.
- Validate:
  - mechanical `metrics.csv` contains at least one row where `Ac_m2>0` (under a correct and stable Ac extraction definition).

---

## Update (2026-01-07 20:20) — Why `TD_RELAX` can “grind” for a long time + watchdog fail-fast

### Why `TD_RELAX` can run for a long time

Even when a TD bridge is “just relaxation”, COMSOL’s time-dependent solver can still take a long time because:
- Each time step requires nonlinear iterations with contact constraints; near onset, iterations can be expensive or stall.
- “Consistent initialization” / DAE initialization can trigger repeated internal attempts before giving up.
- Our `per_solve_timeout_s` is an **after-the-fact** check (it can only run after `model.study(...).run()` returns), so it cannot interrupt a stuck solve.

### Watchdog guarantees real fail-fast (external kill)

Use `scripts/run_onepoint_watchdog.ps1` to run a single-point continuation with a hard wall-clock timeout:
- It sets the same env vars as the usual runbook (contact mode / resume dir / single target).
- If wall-clock exceeds `TimeoutSec`, it force-kills the MATLAB process and the COMSOL mphserver it started, preventing infinite runs.

Recommended single-point stepping:
- Don’t jump `1.023 -> 1.024` directly by default.
- Run `1.0235` first; only after success, run `1.0240`.

---

## Update (2026-01-08) — Long-term fix: dcnt1-only (SolidContact) + correct variable names

### What changed

We are standardizing to **SolidContact `dcnt1` only** and removing/disabling obsolete **Contact `cnt1`**.

Key discovery for this model:
- The contact fields to postprocess are **unprefixed**:
  - `solid.Tn`, `solid.incontact`, `solid.gap`
- `solid.dcnt1.*` can remain **undefined** even when the feature tag is `dcnt1`.

Runbook reference:
- `docs/RUNBOOK_DCONTACT_ONLY.md`

### Evidence: dcnt1-only diagnosis (undefined resolved)

Diagnosis output:
- `out/pyramid_5x5/diag_ac_20260108_131619/Ac_diagnose.csv`

Observed:
- `enforce_ok=true` with `enforce_note=cnt1_removed`
- `solid.incontact` evaluable and yields non-zero area:
  - `Ac_candidate_m2 (incontact>0.5) ≈ 2.324e-10`
- `solid.Tn` evaluable with `max ≈ 2.9345e4 Pa`

This confirms the previous “dcnt1 undefined” issue was a **variable-name mismatch**, not “no contact onset”.

### Regression: minimal single-point solve wall (not improved yet)

Attempted mechanical regression (PTC-only, from last_success=1.0230):
- `out/pyramid_5x5/sim_20260108_131902/`
- target: `1.02301`

---

## Update (2026-01-09) — Pressure-control ignite-v3 (30 min budgeted run)

Goal:
- Try to obtain at least one **pressure-control SUCCESS checkpoint** (or a clear, reproducible FAIL classification) using **solver-only** robustness knobs (no physics changes).

Run:
- Output dir: `out/pyramid_5x5/pressure_ignite_20260109_104553/`
- Inputs:
  - initial values from: `out/pyramid_5x5/sim_20260107_152913/Pyramid_5x5_checkpoint_last_ok.mph`
  - `ContactMode=augmented_lagrange`
  - `SolverCoupling=fully_coupled`
  - `LinearSolverMode=direct_pardiso` (switch confirmed)
  - `IgniteTwoStage=1`
  - `IgniteRamp=1`
  - `IgniteRampList="1.0,0.6,0.3"` (interpreted as **kPa** because `PloadKPa=1.0`)
  - `FcMode=robust`, `FcDamped=1`, `FcLineSearch=1`, `SolverStabilization=1`
  - internal `IgniteBudgetSec=1500`, external watchdog `TimeoutSec=1800`

Result:
- `exit_status=FAIL`
- No `checkpoint_last_ok.mph` produced (`checkpoint_last_ok_exists: 0`)
- Steps attempted (fail-fast on first ramp failure):
  - stage=single, `P_load_kPa=1.0`: **FAIL** (reached max Newton iterations)
  - stage=ramp, `P_load_kPa=0.6`: **FAIL** (reached max Newton iterations)
  - `P_load_kPa=0.3` not attempted due to fail-fast
- Budget: `budget_used_s ≈ 826.9 / 1500`

Failure classification (clear + reproducible):
- **Nonlinear nonconvergence (max Newton iterations)** in `sol1/s1` with additional solver message “linear solver has error message”.

Minimal next actions (modeling-level, smallest changes likely to help):
- Add rigid-body mode suppression for the pressure plate (e.g., explicitly constrain remaining rigid DOFs, or add weak springs/penalty regularization for rotation/translation) to avoid near-singular states under pure pressure load.
- Try “preload then switch” workflow: keep a tiny displacement pre-compression (or keep contact established) then switch to pressure control (helps avoid starting from under-contact / ill-conditioned regime).
- If convergence still stalls: increase `fc_maxiter_target` (robust) and/or insert intermediate pressure steps closer to the failing point (e.g. `1.0 → 0.8 → 0.6`) while keeping fail-fast per-step.

Artifacts (audit):
- `out/pyramid_5x5/pressure_ignite_20260109_104553/summary.txt`
- `out/pyramid_5x5/pressure_ignite_20260109_104553/metrics_pressure.csv`
- `out/pyramid_5x5/pressure_ignite_20260109_104553/errors.json`
- watchdog timeout kill at 900s:
  - `out/pyramid_5x5/sim_20260108_131902/watchdog_heartbeat.txt` ends with `KILLED ... attempt_mode=PTC`
  - `out/pyramid_5x5/sim_20260108_131902/fallback_report.json` stayed at `status=STARTED`

Conclusion:
- Undefined contact fields is fixed (dcnt1-only + `solid.*` fields).
- The `1.0230+` wall remains a real convergence/runtime issue.
